//! sketch DSL (`sketch .. in .. end`) の制約検査。
//!
//! sketch ブロックは GUI の 2D sketch workspace と双方向に紐付くため、
//! 逆評価 (頂点ドラッグ → コード書き戻し) が常に可能な形に式を制限する:
//!
//! - `var x = <符号付き Float リテラル>` — 逆評価の書き込み対象
//! - `let y = <スカラー式>` — 読み取り専用スカラー (リテラル / 既出スカラー名 / 四則演算)
//! - `name = <幾何式>` — `p2` / `line` / `polygon` / `circle` の束縛
//! - `/` の右辺は 0 でない Float リテラルのみ (逆評価が常に定義されるように)
//! - ブロック外の変数参照・Int リテラル・if / case / lambda 等は禁止。
//!   例外として同一モジュールのトップレベル `var` はスカラーとして参照できる
//!   (複数 sketch ブロック間での座標共有用。書き込み対象になる)
//! - body は `{ f = 幾何名, ... }` の record か単一の幾何名のみ
//! - polygon の線分列は静的に連結 + 閉路であること
//!
//! トップレベル `var` 宣言自体もここで検査する (RHS は符号付き Float リテラルのみ)。
//!
//! 検査は構文的 (型推論より前に実行できる)。座標値はブロック内で静的に決まるので
//! 連結性検査のためにここで評価する。

use crate::diagnostic::{Diagnostic, Span};
use crate::syntax::ast::*;
use std::collections::HashMap;

/// 連結性検査の許容誤差。
const EPS: f64 = 1e-9;

/// DSL が構文の一部として使う builtin 名。binding 名としては使えない。
const RESERVED_HEADS: [&str; 6] = ["p2", "line", "polygon", "segments", "circle", "translate2d"];

pub fn check_module(m: &Module) -> Vec<Diagnostic> {
    let mut diag = Vec::new();
    // トップレベル `var` の収集 + RHS 検査。sketch ブロックからスカラーとして参照できる。
    let mut top_vars: HashMap<&str, Option<f64>> = HashMap::new();
    for d in &m.decls {
        if let Decl::Var(v) = d {
            if RESERVED_HEADS.contains(&v.name.as_str()) {
                diag.push(Diagnostic::SketchDsl {
                    span: v.span,
                    message: format!("`{}` は sketch 内で予約された名前です", v.name),
                });
            }
            let val = signed_float_lit(&v.body);
            if val.is_none() {
                diag.push(Diagnostic::SketchDsl {
                    span: v.body.span(),
                    message: "トップレベル var の右辺は Float リテラルのみ書けます (例: `var z = 3.0`)"
                        .to_string(),
                });
            }
            top_vars.insert(v.name.as_str(), val);
        }
    }
    for d in &m.decls {
        match d {
            Decl::Value(v) => find_sketches(&v.body, &top_vars, &mut diag),
            Decl::Slider(s) => find_sketches(&s.body, &top_vars, &mut diag),
            _ => {}
        }
    }
    diag
}

/// 式ツリーから sketch ブロックを探して検査する。sketch の内部には再帰しない
/// (内部の制約は `check_sketch` が見る)。
fn find_sketches(e: &Expr, top_vars: &HashMap<&str, Option<f64>>, diag: &mut Vec<Diagnostic>) {
    match e {
        Expr::Sketch {
            bindings,
            body,
            span,
        } => check_sketch(bindings, body, *span, top_vars, diag),
        Expr::Var { .. } | Expr::Ctor { .. } | Expr::Lit(..) | Expr::Error(_) => {}
        Expr::List(items, _) => items.iter().for_each(|x| find_sketches(x, top_vars, diag)),
        Expr::Record(fields, _) => fields
            .iter()
            .for_each(|f| find_sketches(&f.value, top_vars, diag)),
        Expr::RecordUpdate { base, updates, .. } => {
            find_sketches(base, top_vars, diag);
            updates
                .iter()
                .for_each(|f| find_sketches(&f.value, top_vars, diag));
        }
        Expr::Field { receiver, .. } => find_sketches(receiver, top_vars, diag),
        Expr::App { func, arg, .. } => {
            find_sketches(func, top_vars, diag);
            find_sketches(arg, top_vars, diag);
        }
        Expr::Lambda { body, .. } => find_sketches(body, top_vars, diag),
        Expr::Let { bindings, body, .. } => {
            bindings
                .iter()
                .for_each(|b| find_sketches(&b.body, top_vars, diag));
            find_sketches(body, top_vars, diag);
        }
        Expr::If {
            cond,
            then_branch,
            else_branch,
            ..
        } => {
            find_sketches(cond, top_vars, diag);
            find_sketches(then_branch, top_vars, diag);
            find_sketches(else_branch, top_vars, diag);
        }
        Expr::Case {
            scrutinee, arms, ..
        } => {
            find_sketches(scrutinee, top_vars, diag);
            for a in arms {
                if let Some(g) = &a.guard {
                    find_sketches(g, top_vars, diag);
                }
                find_sketches(&a.body, top_vars, diag);
            }
        }
        Expr::BinOp { left, right, .. } => {
            find_sketches(left, top_vars, diag);
            find_sketches(right, top_vars, diag);
        }
        Expr::Negate(inner, _) => find_sketches(inner, top_vars, diag),
        Expr::Range { lo, hi, .. } => {
            find_sketches(lo, top_vars, diag);
            find_sketches(hi, top_vars, diag);
        }
    }
}

/// binding 名 → 束縛の分類。静的値は検査エラー時 `None`。
#[derive(Clone, Debug)]
enum Entry {
    Scalar(Option<f64>),
    Point(Option<(f64, f64)>),
    Segment(Option<((f64, f64), (f64, f64))>),
    /// polygon / circle。
    Shape,
}

struct Ctx<'a> {
    env: HashMap<&'a str, Entry>,
    diag: &'a mut Vec<Diagnostic>,
}

impl<'a> Ctx<'a> {
    fn err(&mut self, span: Span, message: impl Into<String>) {
        self.diag.push(Diagnostic::SketchDsl {
            span,
            message: message.into(),
        });
    }
}

fn check_sketch(
    bindings: &[SketchBinding],
    body: &Expr,
    _span: Span,
    top_vars: &HashMap<&str, Option<f64>>,
    diag: &mut Vec<Diagnostic>,
) {
    // トップレベル var をスカラーとして参照できるよう env に前置きする。
    let mut cx = Ctx {
        env: top_vars
            .iter()
            .map(|(name, v)| (*name, Entry::Scalar(*v)))
            .collect(),
        diag,
    };
    for b in bindings {
        if RESERVED_HEADS.contains(&b.name.as_str()) {
            cx.err(
                b.name_span,
                format!("`{}` は sketch 内で予約された名前です", b.name),
            );
        } else if top_vars.contains_key(b.name.as_str()) {
            cx.err(
                b.name_span,
                format!("`{}` はトップレベル var と同名です (shadow できません)", b.name),
            );
        } else if cx.env.contains_key(b.name.as_str()) {
            cx.err(
                b.name_span,
                format!("`{}` は既に定義されています", b.name),
            );
        }
        let entry = match b.kind {
            SketchBindKind::Var => Entry::Scalar(check_var_rhs(&mut cx, &b.body)),
            SketchBindKind::Let => Entry::Scalar(check_scalar(&mut cx, &b.body)),
            SketchBindKind::Bare => check_geometry(&mut cx, &b.body),
        };
        cx.env.insert(b.name.as_str(), entry);
    }
    check_body(&mut cx, body);
}

/// `var x = <符号付き Float リテラル>` の右辺検査。
fn check_var_rhs(cx: &mut Ctx, e: &Expr) -> Option<f64> {
    match signed_float_lit(e) {
        Some(v) => Some(v),
        None => {
            cx.err(
                span_of(e),
                "var の右辺は Float リテラルのみ書けます (例: `var x = 3.0`)",
            );
            None
        }
    }
}

/// 符号付き Float リテラル (`3.0` / `-3.0`) なら値を返す。
fn signed_float_lit(e: &Expr) -> Option<f64> {
    match e {
        Expr::Lit(Lit::Float(v), _) => Some(*v),
        Expr::Negate(inner, _) => match inner.as_ref() {
            Expr::Lit(Lit::Float(v), _) => Some(-v),
            _ => None,
        },
        _ => None,
    }
}

/// スカラー式 (リテラル / 既出スカラー名 / 四則演算 / 単項マイナス) の検査 + 静的評価。
fn check_scalar(cx: &mut Ctx, e: &Expr) -> Option<f64> {
    match e {
        Expr::Lit(Lit::Float(v), _) => Some(*v),
        Expr::Lit(Lit::Int(_), span) => {
            cx.err(
                *span,
                "Int リテラルは使えません。Float で書いてください (例: `3.0`)",
            );
            None
        }
        Expr::Var {
            module: None,
            name,
            span,
        } => match cx.env.get(name.as_str()) {
            Some(Entry::Scalar(v)) => *v,
            Some(_) => {
                cx.err(*span, format!("`{name}` はスカラーではありません"));
                None
            }
            None => {
                cx.err(
                    *span,
                    format!("未定義の名前 `{name}` (トップレベル var 以外の sketch ブロック外変数は参照できません)"),
                );
                None
            }
        },
        Expr::Negate(inner, _) => Some(-(check_scalar(cx, inner)?)),
        Expr::BinOp {
            op,
            left,
            right,
            span,
        } => {
            let lv = check_scalar(cx, left);
            match op {
                BinOp::Add => Some(lv? + check_scalar(cx, right)?),
                BinOp::Sub => Some(lv? - check_scalar(cx, right)?),
                BinOp::Mul => Some(lv? * check_scalar(cx, right)?),
                BinOp::Div => match signed_float_lit(right) {
                    Some(rv) if rv != 0.0 => Some(lv? / rv),
                    Some(_) => {
                        cx.err(span_of(right), "`/` の右辺に 0 は書けません");
                        None
                    }
                    None => {
                        cx.err(
                            span_of(right),
                            "`/` の右辺は 0 でない Float リテラルのみ書けます (逆評価のため)",
                        );
                        None
                    }
                },
                _ => {
                    cx.err(*span, "スカラー式で使える演算子は + - * / のみです");
                    None
                }
            }
        }
        Expr::Sketch { span, .. } => {
            cx.err(*span, "sketch ブロックは入れ子にできません");
            None
        }
        _ => {
            cx.err(
                span_of(e),
                "この式はスカラー式として使えません (リテラル / スカラー名 / 四則演算のみ)",
            );
            None
        }
    }
}

/// 幾何束縛 (キーワード無し) の右辺検査。
fn check_geometry(cx: &mut Ctx, e: &Expr) -> Entry {
    let (head, args) = app_spine(e);
    match head {
        Some("p2") if args.len() == 2 => {
            let x = check_scalar(cx, args[0]);
            let y = check_scalar(cx, args[1]);
            Entry::Point(x.zip(y))
        }
        Some("line") if args.len() == 2 => {
            let a = check_point_ref(cx, args[0]);
            let b = check_point_ref(cx, args[1]);
            Entry::Segment(a.zip(b))
        }
        Some("polygon") if args.len() == 1 => {
            check_polygon_arg(cx, args[0]);
            Entry::Shape
        }
        Some("circle") if args.len() == 1 => {
            check_scalar(cx, args[0]);
            Entry::Shape
        }
        _ => {
            // `circle r |> translate2d (p2 ..) (p2 ..)` の移動付き circle
            if let Expr::BinOp {
                op: BinOp::ApplyR,
                left,
                right,
                ..
            } = e
            {
                let (lhead, largs) = app_spine(left);
                let (rhead, rargs) = app_spine(right);
                if lhead == Some("circle")
                    && largs.len() == 1
                    && rhead == Some("translate2d")
                    && rargs.len() == 2
                {
                    check_scalar(cx, largs[0]);
                    check_point_ref(cx, rargs[0]);
                    check_point_ref(cx, rargs[1]);
                    return Entry::Shape;
                }
            }
            if let Expr::Sketch { span, .. } = e {
                cx.err(*span, "sketch ブロックは入れ子にできません");
            } else {
                cx.err(
                    span_of(e),
                    "幾何束縛の右辺は `p2 x y` / `line a b` / `polygon [...]` / `circle r` (+ `|> translate2d`) のみ書けます",
                );
            }
            Entry::Shape
        }
    }
}

/// 点参照: 点 binding 名か `p2 x y`。
fn check_point_ref(cx: &mut Ctx, e: &Expr) -> Option<(f64, f64)> {
    match e {
        Expr::Var {
            module: None,
            name,
            span,
        } => match cx.env.get(name.as_str()) {
            Some(Entry::Point(v)) => *v,
            Some(_) => {
                cx.err(*span, format!("`{name}` は点 (p2) ではありません"));
                None
            }
            None => {
                cx.err(
                    *span,
                    format!("未定義の名前 `{name}` (トップレベル var 以外の sketch ブロック外変数は参照できません)"),
                );
                None
            }
        },
        _ => {
            let (head, args) = app_spine(e);
            if head == Some("p2") && args.len() == 2 {
                let x = check_scalar(cx, args[0]);
                let y = check_scalar(cx, args[1]);
                x.zip(y)
            } else {
                cx.err(
                    span_of(e),
                    "点の位置は点 binding 名か `(p2 x y)` で書いてください",
                );
                None
            }
        }
    }
}

/// 線分参照: 線分 binding 名か `line a b`。
fn check_segment_item(cx: &mut Ctx, e: &Expr) -> Option<((f64, f64), (f64, f64))> {
    match e {
        Expr::Var {
            module: None,
            name,
            span,
        } => match cx.env.get(name.as_str()) {
            Some(Entry::Segment(v)) => *v,
            Some(_) => {
                cx.err(*span, format!("`{name}` は線分 (line) ではありません"));
                None
            }
            None => {
                cx.err(
                    *span,
                    format!("未定義の名前 `{name}` (トップレベル var 以外の sketch ブロック外変数は参照できません)"),
                );
                None
            }
        },
        _ => {
            let (head, args) = app_spine(e);
            if head == Some("line") && args.len() == 2 {
                let a = check_point_ref(cx, args[0]);
                let b = check_point_ref(cx, args[1]);
                a.zip(b)
            } else {
                cx.err(
                    span_of(e),
                    "polygon の要素は線分 binding 名か `line a b` で書いてください",
                );
                None
            }
        }
    }
}

/// polygon の引数: `[<線分>, ...]` か `(segments [<点>, ...])`。
fn check_polygon_arg(cx: &mut Ctx, e: &Expr) {
    match e {
        Expr::List(items, span) => {
            let mut segs: Vec<((f64, f64), (f64, f64))> = Vec::new();
            let mut all_known = true;
            for it in items {
                match check_segment_item(cx, it) {
                    Some(s) => segs.push(s),
                    None => all_known = false,
                }
            }
            if all_known && segs.len() >= 2 {
                check_chain(cx, &segs, *span);
            }
        }
        _ => {
            let (head, args) = app_spine(e);
            if head == Some("segments") && args.len() == 1 {
                match args[0] {
                    Expr::List(items, _) => {
                        for it in items {
                            check_point_ref(cx, it);
                        }
                    }
                    other => cx.err(
                        span_of(other),
                        "segments の引数は `[p2 .., ...]` のリストリテラルのみ書けます",
                    ),
                }
            } else {
                cx.err(
                    span_of(e),
                    "polygon の引数は `[line .., ...]` のリストか `(segments [p2 .., ...])` のみ書けます",
                );
            }
        }
    }
}

/// 線分列の連結性 + 閉路検査。
fn check_chain(cx: &mut Ctx, segs: &[((f64, f64), (f64, f64))], span: Span) {
    let near = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS;
    for w in segs.windows(2) {
        if !near(w[0].1, w[1].0) {
            cx.err(span, "polygon の線分が連結していません (前の線分の終点 = 次の線分の始点 にしてください)");
            return;
        }
    }
    let last = segs[segs.len() - 1];
    let first = segs[0];
    if !near(last.1, first.0) {
        cx.err(span, "polygon が閉じていません (最後の線分の終点 = 最初の線分の始点 にしてください)");
    }
}

/// body: `{ f = 幾何名, ... }` の record か単一の幾何名。
fn check_body(cx: &mut Ctx, body: &Expr) {
    match body {
        Expr::Record(fields, _) => {
            for f in fields {
                check_body_ref(cx, &f.value);
            }
        }
        Expr::Var { .. } => check_body_ref(cx, body),
        _ => cx.err(
            span_of(body),
            "body は `{ 名前 = 幾何名, ... }` の record か単一の幾何名のみ書けます",
        ),
    }
}

fn check_body_ref(cx: &mut Ctx, e: &Expr) {
    match e {
        Expr::Var {
            module: None,
            name,
            span,
        } => match cx.env.get(name.as_str()) {
            Some(Entry::Shape | Entry::Point(_) | Entry::Segment(_)) => {}
            Some(Entry::Scalar(_)) => cx.err(
                *span,
                format!("body で参照できるのは幾何束縛のみです (`{name}` はスカラー)"),
            ),
            None => cx.err(*span, format!("未定義の名前 `{name}`")),
        },
        _ => cx.err(
            span_of(e),
            "body の field は sketch 内の幾何束縛の名前のみ書けます",
        ),
    }
}

fn span_of(e: &Expr) -> Span {
    e.span()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::parse::parse;

    fn check_src(src: &str) -> Vec<Diagnostic> {
        let m = parse(src).unwrap_or_else(|e| panic!("parse failed: {e:?}"));
        check_module(&m)
    }

    fn assert_ok(src: &str) {
        let diags = check_src(src);
        assert!(diags.is_empty(), "expected no diags, got: {diags:?}");
    }

    fn assert_err_contains(src: &str, needle: &str) {
        let diags = check_src(src);
        assert!(
            diags.iter().any(|d| d.message().contains(needle)),
            "expected err containing `{needle}`, got: {diags:?}"
        );
    }

    #[test]
    fn minimal_sketch_ok() {
        assert_ok(
            "sk = sketch\n    var x1 = 0.0\n    let y1 = 3.0\n    poly1 = polygon (segments [p2 x1 y1, p2 4.0 y1, p2 x1 7.0])\n    in { poly1 = poly1 }\nend\n",
        );
    }

    #[test]
    fn line_based_polygon_ok() {
        assert_ok(
            "sk = sketch\n    v1 = p2 0.0 0.0\n    v2 = p2 4.0 0.0\n    v3 = p2 4.0 3.0\n    l1 = line v1 v2\n    poly1 = polygon [l1, line v2 v3, line v3 v1]\n    in poly1\nend\n",
        );
    }

    #[test]
    fn circle_forms_ok() {
        assert_ok(
            "sk = sketch\n    var r = 2.0\n    circ1 = circle r\n    circ2 = circle 3.0 |> translate2d (p2 0.0 0.0) (p2 4.0 5.0)\n    in { circ1 = circ1, circ2 = circ2 }\nend\n",
        );
    }

    #[test]
    fn var_rhs_must_be_literal() {
        assert_err_contains(
            "sk = sketch\n    var x = 1.0 + 2.0\n    p = p2 x 0.0\n    in p\nend\n",
            "var の右辺は Float リテラル",
        );
    }

    #[test]
    fn int_literal_rejected() {
        assert_err_contains(
            "sk = sketch\n    var x = 3.0\n    p = p2 x 1\n    in p\nend\n",
            "Int リテラル",
        );
    }

    #[test]
    fn div_rhs_must_be_nonzero_literal() {
        assert_err_contains(
            "sk = sketch\n    var x = 4.0\n    let y = x / 0.0\n    p = p2 x y\n    in p\nend\n",
            "0 は書けません",
        );
        assert_err_contains(
            "sk = sketch\n    var x = 4.0\n    let z = 2.0\n    let y = x / z\n    p = p2 x y\n    in p\nend\n",
            "Float リテラルのみ",
        );
    }

    #[test]
    fn external_reference_rejected() {
        assert_err_contains(
            "w = 10.0\nsk = sketch\n    p = p2 w 0.0\n    in p\nend\n",
            "未定義の名前 `w`",
        );
    }

    #[test]
    fn top_level_var_reference_ok() {
        // 宣言順に関係なく (後方の var も) 参照できる。let のオペランドにも使える。
        assert_ok(
            "sk = sketch\n    let y = z1 + 1.0\n    p = p2 z1 y\n    in p\nend\nvar z1 = 50.0\n",
        );
    }

    #[test]
    fn top_level_var_shared_by_two_sketches_ok() {
        assert_ok(
            "var z1 = 50.0\nskA = sketch\n    p = p2 0.0 z1\n    in p\nend\nskB = sketch\n    q = p2 1.0 z1\n    in q\nend\n",
        );
    }

    #[test]
    fn top_level_var_rhs_must_be_literal() {
        assert_err_contains(
            "var z1 = 1.0 + 2.0\nsk = sketch\n    p = p2 z1 0.0\n    in p\nend\n",
            "トップレベル var の右辺は Float リテラル",
        );
    }

    #[test]
    fn top_level_var_shadowing_rejected() {
        assert_err_contains(
            "var z1 = 50.0\nsk = sketch\n    var z1 = 1.0\n    p = p2 z1 0.0\n    in p\nend\n",
            "shadow できません",
        );
    }

    #[test]
    fn top_level_var_reserved_name_rejected() {
        assert_err_contains(
            "var p2 = 1.0\nsk = sketch\n    q = p2 1.0 1.0\n    in q\nend\n",
            "予約された名前",
        );
    }

    #[test]
    fn forward_reference_rejected() {
        assert_err_contains(
            "sk = sketch\n    p = p2 x 0.0\n    var x = 1.0\n    in p\nend\n",
            "未定義の名前 `x`",
        );
    }

    #[test]
    fn if_rejected_in_scalar() {
        assert_err_contains(
            "sk = sketch\n    let y = if True then 1.0 else 2.0\n    p = p2 y 0.0\n    in p\nend\n",
            "スカラー式として使えません",
        );
    }

    #[test]
    fn disconnected_polygon_rejected() {
        assert_err_contains(
            "sk = sketch\n    poly1 = polygon [line (p2 0.0 0.0) (p2 1.0 0.0), line (p2 5.0 5.0) (p2 6.0 5.0)]\n    in poly1\nend\n",
            "連結していません",
        );
    }

    #[test]
    fn open_polygon_rejected() {
        assert_err_contains(
            "sk = sketch\n    poly1 = polygon [line (p2 0.0 0.0) (p2 1.0 0.0), line (p2 1.0 0.0) (p2 1.0 1.0)]\n    in poly1\nend\n",
            "閉じていません",
        );
    }

    #[test]
    fn body_must_be_record_or_name() {
        assert_err_contains(
            "sk = sketch\n    var r = 2.0\n    circ1 = circle r\n    in union2d circ1 circ1\nend\n",
            "body は",
        );
    }

    #[test]
    fn body_cannot_reference_scalar() {
        assert_err_contains(
            "sk = sketch\n    var x = 2.0\n    p = p2 x x\n    in { p = p, x = x }\nend\n",
            "スカラー",
        );
    }

    #[test]
    fn duplicate_binding_rejected() {
        assert_err_contains(
            "sk = sketch\n    var x = 1.0\n    var x = 2.0\n    p = p2 x x\n    in p\nend\n",
            "既に定義されています",
        );
    }

    #[test]
    fn reserved_name_rejected() {
        assert_err_contains(
            "sk = sketch\n    var line = 1.0\n    p = p2 line line\n    in p\nend\n",
            "予約された名前",
        );
    }

    #[test]
    fn nested_sketch_rejected() {
        assert_err_contains(
            "sk = sketch\n    let y = sketch\n            var z = 1.0\n            q = p2 z z\n        in q\n        end\n    p = p2 y y\n    in p\nend\n",
            "入れ子",
        );
    }

    #[test]
    fn scalar_referencing_geometry_rejected() {
        assert_err_contains(
            "sk = sketch\n    p = p2 1.0 2.0\n    let y = p + 1.0\n    q = p2 y y\n    in q\nend\n",
            "スカラーではありません",
        );
    }

    #[test]
    fn sketch_nested_in_let_is_found() {
        assert_err_contains(
            "sk =\n    let\n        inner = sketch\n                var x = 1\n                p = p2 x x\n            in p\n            end\n    in\n    inner\n",
            "var の右辺は Float リテラル",
        );
    }
}
