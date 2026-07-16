//! sketch DSL (`sketch .. in .. end`) と GUI sketch workspace の双方向連携。
//!
//! - [`model`] / [`model_from_source`]: forward eval。sketch ブロックから描画・
//!   ハンドル表示用の [`SketchModel`] を作る (manifold 不要・同期・軽量)。
//! - [`drag`]: 逆評価。ハンドルのドラッグ (新しい座標) を var / リテラルへの
//!   テキスト書き換えとして解決する。書き込み先の決定規則:
//!     - 座標式全体が符号付き Float リテラル → そのリテラル (匿名 var 扱い)
//!     - `var` 束縛への参照 → その定義リテラル (ブロック内 var / トップレベル var とも。
//!       トップレベル var は複数 sketch ブロックから共有されるので、書き込むと
//!       参照している全ブロックが連動する)
//!     - `let` 束縛への参照 → RHS を辿って var へ押し込む (導出値。
//!       `let t = b + 150.0` の t をドラッグすると b が書き換わる)。
//!       RHS に var が無い (`let y = 3.0` 等) 場合は書き込み不可
//!     - 二項演算 → 書き込み可能な側がちょうど 1 つならそちらへ押し込む
//!       (もう一方は現在値で定数化)。両方可 / 両方不可なら拒否。
//!   共有頂点 (junction) は構成する全ての座標式へ書き込む。軸ごとに独立で、
//!   片軸だけ固定されている場合は動かせる軸のみ適用し `pinned` として報告する。
//! - 構造編集 ([`add_point`] など): binding の挿入 / 削除と body record の更新。
//!
//! 将来の拘束ソルバー導入時は「新しい値の割り当てを決める」部分 (invert) を
//! 差し替え、リテラル書き換え ([`TextEdit`] / [`apply_edits`]) はそのまま使う。

use crate::diagnostic::Span;
use crate::syntax::ast::*;
use crate::syntax::free_vars::free_var_names_in;
use crate::syntax::parse;
use std::collections::{HashMap, HashSet};

// ---------------------------------------------------------------------------
// 公開 model 型
// ---------------------------------------------------------------------------

/// GUI へ渡す sketch ブロックの評価済み形状。
#[derive(Clone, Debug, PartialEq)]
pub struct SketchModel {
    pub binding: String,
    /// sketch 式全体の span。
    pub span: Span,
    pub geoms: Vec<SketchGeom>,
    /// ハンドルごとの連動情報。
    pub links: Vec<HandleLink>,
}

/// ハンドル 1 つ分の連動情報。`leaves` はドラッグで書き込まれるリテラル葉の
/// ソース上の位置 (span start)。葉を共有するハンドル同士はドラッグで連動して動く。
#[derive(Clone, Debug, PartialEq)]
pub struct HandleLink {
    pub target: DragTarget,
    pub leaves: Vec<usize>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SketchGeom {
    /// 参照点 (`p2`)。
    Point { name: String, pos: [f64; 2] },
    /// 名前付き線分 (`line`)。polygon から参照されていても列挙する。
    Segment {
        name: String,
        a: [f64; 2],
        b: [f64; 2],
    },
    /// 頂点列 (暗黙に閉じる)。
    Polygon { name: String, verts: Vec<[f64; 2]> },
    Circle {
        name: String,
        center: [f64; 2],
        radius: f64,
    },
}

impl SketchGeom {
    pub fn name(&self) -> &str {
        match self {
            SketchGeom::Point { name, .. }
            | SketchGeom::Segment { name, .. }
            | SketchGeom::Polygon { name, .. }
            | SketchGeom::Circle { name, .. } => name,
        }
    }
}

/// ドラッグ対象。`geom` は [`SketchModel::geoms`] の index。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DragTarget {
    Point { geom: usize },
    /// `end`: 0 = 始点, 1 = 終点。
    SegmentEnd { geom: usize, end: usize },
    PolyVertex { geom: usize, vert: usize },
    CircleCenter { geom: usize },
    CircleRadius { geom: usize },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DragValue {
    Pos([f64; 2]),
    Radius(f64),
}

/// ドラッグ 1 ステップの結果。`source` が新しいソース全文。
#[derive(Clone, Debug)]
pub struct DragOutcome {
    pub source: String,
    pub model: SketchModel,
    /// 書き込みできず固定されたままの軸の説明。空なら全軸適用。
    pub pinned: Vec<String>,
}

/// ドラッグが一切適用できなかった理由。
#[derive(Clone, Debug, PartialEq)]
pub enum DragReject {
    /// 対象軸が読み取り専用 (座標式から書き込み可能な var に到達できない)。
    ReadOnly(String),
    /// 書き込み先が曖昧 (二項演算の両辺が書き込み可能)。
    Ambiguous(String),
    /// 係数 0 で逆評価できない。
    ZeroFactor(String),
    /// 複数の書き込みが同じリテラルに異なる値で衝突した。
    Conflict(String),
    /// sketch ブロックの構造が想定外 (validation を通っていない等)。
    Invalid(String),
}

impl DragReject {
    pub fn message(&self) -> &str {
        match self {
            DragReject::ReadOnly(m)
            | DragReject::Ambiguous(m)
            | DragReject::ZeroFactor(m)
            | DragReject::Conflict(m)
            | DragReject::Invalid(m) => m,
        }
    }
}

/// ソーステキストへの置換編集。
#[derive(Clone, Debug, PartialEq)]
pub struct TextEdit {
    pub span: Span,
    pub replacement: String,
}

/// 編集群を適用した新しいソースを返す。span は重複しないこと。
pub fn apply_edits(src: &str, edits: &[TextEdit]) -> String {
    let mut sorted: Vec<&TextEdit> = edits.iter().collect();
    sorted.sort_by_key(|e| e.span.start);
    let mut out = src.to_string();
    for e in sorted.iter().rev() {
        out.replace_range(e.span.range(), &e.replacement);
    }
    out
}

// ---------------------------------------------------------------------------
// binding 探索 + forward eval
// ---------------------------------------------------------------------------

/// body が `Expr::Sketch` そのものである引数なし top-level binding の名前一覧。
pub fn sketch_binding_names(module: &Module) -> Vec<String> {
    module
        .decls
        .iter()
        .filter_map(|d| match d {
            Decl::Value(v) if v.params.is_empty() && matches!(v.body, Expr::Sketch { .. }) => {
                Some(v.name.clone())
            }
            _ => None,
        })
        .collect()
}

fn find_block<'a>(
    module: &'a Module,
    binding: &str,
) -> Result<(&'a [SketchBinding], &'a Expr, Span), String> {
    for d in &module.decls {
        if let Decl::Value(v) = d {
            if v.name == binding && v.params.is_empty() {
                if let Expr::Sketch {
                    bindings,
                    body,
                    span,
                } = &v.body
                {
                    return Ok((bindings, body, *span));
                }
            }
        }
    }
    Err(format!(
        "`{binding}` は sketch ブロックの binding ではありません"
    ))
}

/// 同一モジュールのトップレベル `var` 宣言 (名前 → RHS 式)。
fn top_var_map(module: &Module) -> HashMap<&str, &Expr> {
    module
        .decls
        .iter()
        .filter_map(|d| match d {
            Decl::Var(v) => Some((v.name.as_str(), &v.body)),
            _ => None,
        })
        .collect()
}

pub fn model(module: &Module, binding: &str) -> Result<SketchModel, String> {
    let (bindings, _body, span) = find_block(module, binding)?;
    let block = Block::build(module, bindings)?;
    let mut geoms = Vec::new();
    let mut links = Vec::new();
    for b in bindings {
        if b.kind != SketchBindKind::Bare {
            continue;
        }
        let gi = geoms.len();
        match block.geom_shape(b)? {
            GeomShape::Point(slot) => {
                links.push(HandleLink {
                    target: DragTarget::Point { geom: gi },
                    leaves: block.slot_leaves(&slot),
                });
                geoms.push(SketchGeom::Point {
                    name: b.name.clone(),
                    pos: slot.pos,
                });
            }
            GeomShape::Segment(a, bb) => {
                links.push(HandleLink {
                    target: DragTarget::SegmentEnd { geom: gi, end: 0 },
                    leaves: block.slot_leaves(&a),
                });
                links.push(HandleLink {
                    target: DragTarget::SegmentEnd { geom: gi, end: 1 },
                    leaves: block.slot_leaves(&bb),
                });
                geoms.push(SketchGeom::Segment {
                    name: b.name.clone(),
                    a: a.pos,
                    b: bb.pos,
                });
            }
            GeomShape::Polygon(slots) => {
                for (vi, s) in slots.iter().enumerate() {
                    links.push(HandleLink {
                        target: DragTarget::PolyVertex { geom: gi, vert: vi },
                        leaves: block.slot_leaves(s),
                    });
                }
                geoms.push(SketchGeom::Polygon {
                    name: b.name.clone(),
                    verts: slots.iter().map(|s| s.pos).collect(),
                });
            }
            GeomShape::Circle(c) => {
                links.push(HandleLink {
                    target: DragTarget::CircleCenter { geom: gi },
                    leaves: c
                        .dst
                        .as_ref()
                        .map(|d| block.slot_leaves(d))
                        .unwrap_or_default(),
                });
                links.push(HandleLink {
                    target: DragTarget::CircleRadius { geom: gi },
                    leaves: block.axis_leaves(&c.radius.exprs, c.radius.value),
                });
                geoms.push(SketchGeom::Circle {
                    name: b.name.clone(),
                    center: c.center_pos,
                    radius: c.radius.value,
                });
            }
        }
    }
    Ok(SketchModel {
        binding: binding.to_string(),
        span,
        geoms,
        links,
    })
}

pub fn model_from_source(src: &str, binding: &str) -> Result<SketchModel, String> {
    let module = parse_or_msg(src)?;
    model(&module, binding)
}

fn parse_or_msg(src: &str) -> Result<Module, String> {
    parse::parse(src).map_err(|e| {
        e.first()
            .map(|d| d.message())
            .unwrap_or_else(|| "parse error".to_string())
    })
}

// ---------------------------------------------------------------------------
// 内部表現: 座標スロット (値 + 書き込み対象式の集合)
// ---------------------------------------------------------------------------

/// スカラー 1 つ分。`exprs` は junction を構成する全座標式 (通常 1 つ)。
#[derive(Clone)]
struct Slot<'a> {
    value: f64,
    exprs: Vec<&'a Expr>,
}

/// 2D 位置 1 つ分。
#[derive(Clone)]
struct PosSlot<'a> {
    pos: [f64; 2],
    xs: Vec<&'a Expr>,
    ys: Vec<&'a Expr>,
}

impl<'a> PosSlot<'a> {
    /// junction の合流。同じ式 (span 一致) は重複させない。
    fn merge(&mut self, other: PosSlot<'a>) {
        for e in other.xs {
            if !self.xs.iter().any(|x| x.span() == e.span()) {
                self.xs.push(e);
            }
        }
        for e in other.ys {
            if !self.ys.iter().any(|y| y.span() == e.span()) {
                self.ys.push(e);
            }
        }
    }
}

struct CircleShape<'a> {
    center_pos: [f64; 2],
    /// `translate2d` の dst 引数。無ければ原点固定で中心は動かせない。
    dst: Option<PosSlot<'a>>,
    /// `translate2d` の src の静的値 (中心 = dst - src)。
    src: [f64; 2],
    radius: Slot<'a>,
}

enum GeomShape<'a> {
    Point(PosSlot<'a>),
    Segment(PosSlot<'a>, PosSlot<'a>),
    Polygon(Vec<PosSlot<'a>>),
    Circle(CircleShape<'a>),
}

struct Block<'a> {
    by_name: HashMap<&'a str, &'a SketchBinding>,
    scalar_values: HashMap<&'a str, f64>,
    /// トップレベル `var` (名前 → RHS 式)。ブロック内 binding が優先される。
    top_vars: HashMap<&'a str, &'a Expr>,
}

impl<'a> Block<'a> {
    fn build(module: &'a Module, bindings: &'a [SketchBinding]) -> Result<Self, String> {
        let mut block = Block {
            by_name: HashMap::new(),
            scalar_values: HashMap::new(),
            top_vars: top_var_map(module),
        };
        // RHS がリテラルでない top var は sema が拒否するのでここでは単に無視する
        // (参照時に「スカラーではありません」エラーになる)。
        for (name, e) in &block.top_vars {
            if let Some((v, _)) = signed_lit_leaf(e) {
                block.scalar_values.insert(*name, v);
            }
        }
        for b in bindings {
            if b.kind != SketchBindKind::Bare {
                let v = block.eval_scalar(&b.body)?;
                block.scalar_values.insert(b.name.as_str(), v);
            }
            block.by_name.insert(b.name.as_str(), b);
        }
        Ok(block)
    }

    fn eval_scalar(&self, e: &Expr) -> Result<f64, String> {
        match e {
            Expr::Lit(Lit::Float(v), _) => Ok(*v),
            Expr::Var {
                module: None, name, ..
            } => self
                .scalar_values
                .get(name.as_str())
                .copied()
                .ok_or_else(|| format!("`{name}` はスカラーではありません")),
            Expr::Negate(inner, _) => Ok(-self.eval_scalar(inner)?),
            Expr::BinOp {
                op, left, right, ..
            } => {
                let l = self.eval_scalar(left)?;
                let r = self.eval_scalar(right)?;
                match op {
                    BinOp::Add => Ok(l + r),
                    BinOp::Sub => Ok(l - r),
                    BinOp::Mul => Ok(l * r),
                    BinOp::Div => Ok(l / r),
                    _ => Err("スカラー式に使えない演算子".to_string()),
                }
            }
            _ => Err("スカラー式として評価できません".to_string()),
        }
    }

    /// 点参照 (点 binding 名か `p2 x y`) をスロットに解決する。
    fn point_slot(&self, e: &'a Expr) -> Result<PosSlot<'a>, String> {
        if let Expr::Var {
            module: None, name, ..
        } = e
        {
            let b = self
                .by_name
                .get(name.as_str())
                .ok_or_else(|| format!("未定義の名前 `{name}`"))?;
            return self.point_slot(&b.body);
        }
        let (head, args) = app_spine(e);
        if head == Some("p2") && args.len() == 2 {
            let x = self.eval_scalar(args[0])?;
            let y = self.eval_scalar(args[1])?;
            Ok(PosSlot {
                pos: [x, y],
                xs: vec![args[0]],
                ys: vec![args[1]],
            })
        } else {
            Err("点参照が `p2 x y` の形ではありません".to_string())
        }
    }

    /// 線分参照 (線分 binding 名か `line a b`) を両端点スロットに解決する。
    fn segment_ends(&self, e: &'a Expr) -> Result<(PosSlot<'a>, PosSlot<'a>), String> {
        if let Expr::Var {
            module: None, name, ..
        } = e
        {
            let b = self
                .by_name
                .get(name.as_str())
                .ok_or_else(|| format!("未定義の名前 `{name}`"))?;
            return self.segment_ends(&b.body);
        }
        let (head, args) = app_spine(e);
        if head == Some("line") && args.len() == 2 {
            Ok((self.point_slot(args[0])?, self.point_slot(args[1])?))
        } else {
            Err("線分参照が `line a b` の形ではありません".to_string())
        }
    }

    fn geom_shape(&self, b: &'a SketchBinding) -> Result<GeomShape<'a>, String> {
        let e = &b.body;
        let (head, args) = app_spine(e);
        match head {
            Some("p2") if args.len() == 2 => Ok(GeomShape::Point(self.point_slot(e)?)),
            Some("line") if args.len() == 2 => {
                let (a, bb) = self.segment_ends(e)?;
                Ok(GeomShape::Segment(a, bb))
            }
            Some("polygon") if args.len() == 1 => {
                Ok(GeomShape::Polygon(self.polygon_slots(args[0])?))
            }
            Some("circle") if args.len() == 1 => Ok(GeomShape::Circle(CircleShape {
                center_pos: [0.0, 0.0],
                dst: None,
                src: [0.0, 0.0],
                radius: Slot {
                    value: self.eval_scalar(args[0])?,
                    exprs: vec![args[0]],
                },
            })),
            _ => {
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
                        let src = self.point_slot(rargs[0])?;
                        let dst = self.point_slot(rargs[1])?;
                        let center_pos =
                            [dst.pos[0] - src.pos[0], dst.pos[1] - src.pos[1]];
                        return Ok(GeomShape::Circle(CircleShape {
                            center_pos,
                            src: src.pos,
                            dst: Some(dst),
                            radius: Slot {
                                value: self.eval_scalar(largs[0])?,
                                exprs: vec![largs[0]],
                            },
                        }));
                    }
                }
                Err(format!(
                    "`{}` の右辺が幾何式として解釈できません",
                    b.name
                ))
            }
        }
    }

    /// polygon の頂点スロット列。line 形式は隣接線分の端点を junction として合流する。
    fn polygon_slots(&self, arg: &'a Expr) -> Result<Vec<PosSlot<'a>>, String> {
        match arg {
            // `polygon [line .., ...]` (線分列)
            Expr::List(items, _) => {
                let mut ends: Vec<(PosSlot<'a>, PosSlot<'a>)> = Vec::new();
                for it in items {
                    ends.push(self.segment_ends(it)?);
                }
                let n = ends.len();
                let mut verts: Vec<PosSlot<'a>> = Vec::new();
                for i in 0..n {
                    // 頂点 i = seg[i] の始点 ∪ seg[i-1] の終点 (wrap)
                    let mut v = ends[i].0.clone();
                    if n > 1 {
                        v.merge(ends[(i + n - 1) % n].1.clone());
                    }
                    verts.push(v);
                }
                Ok(verts)
            }
            // `polygon (segments [p2 .., ...])` (点列)
            _ => {
                let (head, args) = app_spine(arg);
                if head == Some("segments") && args.len() == 1 {
                    if let Expr::List(items, _) = args[0] {
                        let mut verts = Vec::new();
                        for it in items {
                            verts.push(self.point_slot(it)?);
                        }
                        Ok(verts)
                    } else {
                        Err("segments の引数がリストリテラルではありません".to_string())
                    }
                } else {
                    Err("polygon の引数が解釈できません".to_string())
                }
            }
        }
    }

    /// 部分木に `var` 束縛 (ブロック内 / トップレベル) への参照が含まれるか。
    /// `let` 参照は導出値として RHS に再帰する (循環は `build` の逐次評価が先に弾く)。
    fn contains_writable(&self, e: &Expr) -> bool {
        match e {
            Expr::Var {
                module: None, name, ..
            } => match self.by_name.get(name.as_str()) {
                Some(b) if b.kind == SketchBindKind::Var => true,
                Some(b) if b.kind == SketchBindKind::Let => self.contains_writable(&b.body),
                Some(_) => false,
                None => self.top_vars.contains_key(name.as_str()),
            },
            Expr::Negate(inner, _) => self.contains_writable(inner),
            Expr::BinOp { left, right, .. } => {
                self.contains_writable(left) || self.contains_writable(right)
            }
            _ => false,
        }
    }

    /// 座標式 `e` に目標値 `target` を流し、書き換えるリテラル葉と新値を返す。
    fn invert(&self, e: &Expr, target: f64) -> Result<(Leaf, f64), SolveErr> {
        // 座標式全体が符号付きリテラル → 匿名 var として書き込み可
        if let Some((_, leaf)) = signed_lit_leaf(e) {
            return Ok((leaf, target));
        }
        self.invert_inner(e, target)
    }

    fn invert_inner(&self, e: &Expr, target: f64) -> Result<(Leaf, f64), SolveErr> {
        match e {
            Expr::Var {
                module: None, name, ..
            } => match self.by_name.get(name.as_str()) {
                Some(b) if b.kind == SketchBindKind::Var => {
                    // var の RHS は符号付きリテラル (validation 保証)
                    let (_, leaf) = signed_lit_leaf(&b.body).ok_or(SolveErr::ReadOnly)?;
                    Ok((leaf, target))
                }
                // let は導出値: RHS を辿って依存先の var へ押し込む。
                Some(b) if b.kind == SketchBindKind::Let => self.invert_inner(&b.body, target),
                Some(_) => Err(SolveErr::ReadOnly),
                None => match self.top_vars.get(name.as_str()) {
                    Some(body) => {
                        let (_, leaf) = signed_lit_leaf(body).ok_or(SolveErr::ReadOnly)?;
                        Ok((leaf, target))
                    }
                    None => Err(SolveErr::ReadOnly),
                },
            },
            Expr::Negate(inner, _) => self.invert_inner(inner, -target),
            Expr::BinOp {
                op, left, right, ..
            } => {
                let lw = self.contains_writable(left);
                let rw = self.contains_writable(right);
                match (lw, rw) {
                    (true, true) => Err(SolveErr::Ambiguous),
                    (false, false) => Err(SolveErr::ReadOnly),
                    (true, false) => {
                        let rv = self.eval_scalar(right).map_err(|_| SolveErr::ReadOnly)?;
                        match op {
                            BinOp::Add => self.invert_inner(left, target - rv),
                            BinOp::Sub => self.invert_inner(left, target + rv),
                            BinOp::Mul if rv != 0.0 => self.invert_inner(left, target / rv),
                            BinOp::Mul => Err(SolveErr::ZeroFactor),
                            BinOp::Div => self.invert_inner(left, target * rv),
                            _ => Err(SolveErr::ReadOnly),
                        }
                    }
                    (false, true) => {
                        let lv = self.eval_scalar(left).map_err(|_| SolveErr::ReadOnly)?;
                        match op {
                            BinOp::Add => self.invert_inner(right, target - lv),
                            BinOp::Sub => self.invert_inner(right, lv - target),
                            BinOp::Mul if lv != 0.0 => self.invert_inner(right, target / lv),
                            BinOp::Mul => Err(SolveErr::ZeroFactor),
                            // Div の右辺はリテラル (validation 保証) なので writable にならない
                            _ => Err(SolveErr::ReadOnly),
                        }
                    }
                }
            }
            _ => Err(SolveErr::ReadOnly),
        }
    }

    /// 軸 1 つ分のドラッグ書き込み先リテラル葉 (span start)。1 式でも書き込め
    /// ない場合 [`drag`] はその軸ごと固定するので、連動情報としても空を返す。
    fn axis_leaves(&self, exprs: &[&Expr], value: f64) -> Vec<usize> {
        let mut leaves = Vec::new();
        for e in exprs {
            match self.invert(e, value) {
                Ok((leaf, _)) => leaves.push(leaf.span.start),
                Err(_) => return Vec::new(),
            }
        }
        leaves
    }

    fn slot_leaves(&self, slot: &PosSlot) -> Vec<usize> {
        let mut leaves = self.axis_leaves(&slot.xs, slot.pos[0]);
        leaves.extend(self.axis_leaves(&slot.ys, slot.pos[1]));
        leaves
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum SolveErr {
    ReadOnly,
    Ambiguous,
    ZeroFactor,
}

impl SolveErr {
    fn reason(self) -> &'static str {
        match self {
            SolveErr::ReadOnly => "書き込める var がありません",
            SolveErr::Ambiguous => "書き込み先が曖昧です (両辺に var があります)",
            SolveErr::ZeroFactor => "係数が 0 のため動かせません",
        }
    }

    fn into_reject(self, msg: String) -> DragReject {
        match self {
            SolveErr::ReadOnly => DragReject::ReadOnly(msg),
            SolveErr::Ambiguous => DragReject::Ambiguous(msg),
            SolveErr::ZeroFactor => DragReject::ZeroFactor(msg),
        }
    }
}

/// 書き換え対象のリテラル葉。
#[derive(Clone, Copy, Debug)]
struct Leaf {
    span: Span,
    /// `Negate(Lit)` 全体 (span が `-` を含む) か。
    negated: bool,
}

fn signed_lit_leaf(e: &Expr) -> Option<(f64, Leaf)> {
    match e {
        Expr::Lit(Lit::Float(v), span) => Some((
            *v,
            Leaf {
                span: *span,
                negated: false,
            },
        )),
        Expr::Negate(inner, span) => match inner.as_ref() {
            Expr::Lit(Lit::Float(v), _) => Some((
                -v,
                Leaf {
                    span: *span,
                    negated: true,
                },
            )),
            _ => None,
        },
        _ => None,
    }
}

/// Float リテラルとして re-lex 可能な形に整形する (`.0` を保証)。
/// -0.0 は 0.0 に正規化する (`-0.0 < 0.0` が false のため括弧が付かず、
/// `-0.0` のまま埋め込むと引数位置で減算にパースされてしまう)。
fn fmt_float(v: f64) -> String {
    let v = if v == 0.0 { 0.0 } else { v };
    let s = format!("{v}");
    if s.contains('.') { s } else { format!("{s}.0") }
}

/// 葉の置換テキスト。負数は元が `-lit` 全体でない限り括弧で包む
/// (関数適用の引数位置で減算に解釈されないように)。
fn fmt_leaf(v: f64, leaf: &Leaf) -> String {
    let s = fmt_float(v);
    if !leaf.negated && v < 0.0 {
        format!("({s})")
    } else {
        s
    }
}

/// 式の引数位置に埋め込む座標リテラル。負数は括弧で包む。
fn fmt_coord(v: f64) -> String {
    let s = fmt_float(v);
    if v < 0.0 { format!("({s})") } else { s }
}

// ---------------------------------------------------------------------------
// drag (逆評価)
// ---------------------------------------------------------------------------

pub fn drag(
    src: &str,
    binding: &str,
    target: DragTarget,
    value: DragValue,
) -> Result<DragOutcome, DragReject> {
    match value {
        DragValue::Pos(p) if !(p[0].is_finite() && p[1].is_finite()) => {
            return Err(DragReject::Invalid("座標が有限値ではありません".into()));
        }
        DragValue::Radius(r) if !r.is_finite() => {
            return Err(DragReject::Invalid("半径が有限値ではありません".into()));
        }
        _ => {}
    }
    let module = parse_or_msg(src).map_err(DragReject::Invalid)?;
    let (bindings, _body, _span) = find_block(&module, binding).map_err(DragReject::Invalid)?;
    let block = Block::build(&module, bindings).map_err(DragReject::Invalid)?;
    let bare: Vec<&SketchBinding> = bindings
        .iter()
        .filter(|b| b.kind == SketchBindKind::Bare)
        .collect();

    let geom_of = |i: usize| -> Result<&SketchBinding, DragReject> {
        bare.get(i)
            .copied()
            .ok_or_else(|| DragReject::Invalid(format!("幾何 index {i} が範囲外です")))
    };

    // 軸ごとの (ラベル, 書き込み対象式, 目標値)
    let mut axes: Vec<(&'static str, Vec<&Expr>, f64)> = Vec::new();
    match (target, value) {
        (DragTarget::Point { geom }, DragValue::Pos(p)) => {
            let shape = block.geom_shape(geom_of(geom)?).map_err(DragReject::Invalid)?;
            let GeomShape::Point(slot) = shape else {
                return Err(DragReject::Invalid("対象が点ではありません".into()));
            };
            axes.push(("x", slot.xs, p[0]));
            axes.push(("y", slot.ys, p[1]));
        }
        (DragTarget::SegmentEnd { geom, end }, DragValue::Pos(p)) => {
            let shape = block.geom_shape(geom_of(geom)?).map_err(DragReject::Invalid)?;
            let GeomShape::Segment(a, b) = shape else {
                return Err(DragReject::Invalid("対象が線分ではありません".into()));
            };
            let slot = if end == 0 { a } else { b };
            axes.push(("x", slot.xs, p[0]));
            axes.push(("y", slot.ys, p[1]));
        }
        (DragTarget::PolyVertex { geom, vert }, DragValue::Pos(p)) => {
            let shape = block.geom_shape(geom_of(geom)?).map_err(DragReject::Invalid)?;
            let GeomShape::Polygon(slots) = shape else {
                return Err(DragReject::Invalid("対象が polygon ではありません".into()));
            };
            let slot = slots
                .into_iter()
                .nth(vert)
                .ok_or_else(|| DragReject::Invalid(format!("頂点 index {vert} が範囲外です")))?;
            axes.push(("x", slot.xs, p[0]));
            axes.push(("y", slot.ys, p[1]));
        }
        (DragTarget::CircleCenter { geom }, DragValue::Pos(p)) => {
            let shape = block.geom_shape(geom_of(geom)?).map_err(DragReject::Invalid)?;
            let GeomShape::Circle(c) = shape else {
                return Err(DragReject::Invalid("対象が circle ではありません".into()));
            };
            let Some(dst) = c.dst else {
                return Err(DragReject::ReadOnly(
                    "この circle は translate2d が無いため中心を動かせません".into(),
                ));
            };
            // 中心 = dst - src なので dst の目標値は 新中心 + src
            axes.push(("x", dst.xs, p[0] + c.src[0]));
            axes.push(("y", dst.ys, p[1] + c.src[1]));
        }
        (DragTarget::CircleRadius { geom }, DragValue::Radius(r)) => {
            let shape = block.geom_shape(geom_of(geom)?).map_err(DragReject::Invalid)?;
            let GeomShape::Circle(c) = shape else {
                return Err(DragReject::Invalid("対象が circle ではありません".into()));
            };
            axes.push(("半径", c.radius.exprs, r));
        }
        _ => {
            return Err(DragReject::Invalid(
                "ドラッグ対象と値の種類が一致しません".into(),
            ));
        }
    }

    // 軸ごとに独立して逆評価。軸内は全式が書けなければその軸ごと固定。
    let mut writes: Vec<(Leaf, f64)> = Vec::new();
    let mut pinned: Vec<String> = Vec::new();
    let mut first_err: Option<SolveErr> = None;
    for (label, exprs, tval) in axes {
        let mut axis_writes: Vec<(Leaf, f64)> = Vec::new();
        let mut axis_err: Option<SolveErr> = if exprs.is_empty() {
            Some(SolveErr::ReadOnly)
        } else {
            None
        };
        for e in exprs {
            match block.invert(e, tval) {
                Ok(w) => axis_writes.push(w),
                Err(err) => {
                    axis_err = Some(err);
                    break;
                }
            }
        }
        match axis_err {
            None => writes.extend(axis_writes),
            Some(err) => {
                pinned.push(format!("{label} は{}", err.reason()));
                if first_err.is_none() {
                    first_err = Some(err);
                }
            }
        }
    }
    if writes.is_empty() {
        let err = first_err.unwrap_or(SolveErr::ReadOnly);
        return Err(err.into_reject(pinned.join(" / ")));
    }

    // 同一葉への書き込みを整理: 同値は dedupe、異値は衝突。
    let mut merged: Vec<(Leaf, f64)> = Vec::new();
    for (leaf, v) in writes {
        if let Some((_, prev)) = merged.iter().find(|(l, _)| l.span == leaf.span) {
            if (prev - v).abs() > f64::EPSILON {
                return Err(DragReject::Conflict(
                    "同じ値に異なる座標を書き込もうとしています (x と y が同じ var を共有)".into(),
                ));
            }
        } else {
            merged.push((leaf, v));
        }
    }

    let edits: Vec<TextEdit> = merged
        .iter()
        .map(|(leaf, v)| TextEdit {
            span: leaf.span,
            replacement: fmt_leaf(*v, leaf),
        })
        .collect();
    let source = apply_edits(src, &edits);
    let model = model_from_source(&source, binding).map_err(DragReject::Invalid)?;
    Ok(DragOutcome {
        source,
        model,
        pinned,
    })
}

// ---------------------------------------------------------------------------
// 構造編集
// ---------------------------------------------------------------------------

/// 既存名と衝突しない `prefix{n}` を返す。
fn fresh_name(prefix: &str, taken: impl Fn(&str) -> bool) -> String {
    let mut n = 1;
    loop {
        let name = format!("{prefix}{n}");
        if !taken(&name) {
            return name;
        }
        n += 1;
    }
}

/// `span.start` を含む行のインデント文字列。
fn line_indent(src: &str, at: usize) -> String {
    let line_start = src[..at].rfind('\n').map(|i| i + 1).unwrap_or(0);
    src[line_start..at]
        .chars()
        .take_while(|c| c.is_whitespace())
        .collect()
}

/// body record に `name = name` の field を足す編集。
fn body_add_field(body: &Expr, name: &str) -> Result<TextEdit, String> {
    match body {
        Expr::Record(fields, span) => match fields.last() {
            Some(last) => Ok(TextEdit {
                span: Span::new(last.span.end, last.span.end),
                replacement: format!(", {name} = {name}"),
            }),
            None => Ok(TextEdit {
                span: *span,
                replacement: format!("{{ {name} = {name} }}"),
            }),
        },
        Expr::Var {
            name: old, span, ..
        } => Ok(TextEdit {
            span: *span,
            replacement: format!("{{ {old} = {old}, {name} = {name} }}"),
        }),
        _ => Err("body が record か幾何名ではありません".to_string()),
    }
}

/// 新しい bare binding を末尾に挿入し body record に追加する。
/// 戻り値は (新ソース, 新 binding 名)。
fn insert_geom(
    src: &str,
    binding: &str,
    prefix: &str,
    rhs: &str,
) -> Result<(String, String), String> {
    let module = parse_or_msg(src)?;
    let (bindings, body, _span) = find_block(&module, binding)?;
    let name = fresh_name(prefix, |n| bindings.iter().any(|b| b.name == n));
    let last = bindings
        .last()
        .ok_or_else(|| "sketch ブロックに束縛がありません".to_string())?;
    let indent = line_indent(src, last.span.start);
    let edits = vec![
        TextEdit {
            span: Span::new(last.span.end, last.span.end),
            replacement: format!("\n{indent}{name} = {rhs}"),
        },
        body_add_field(body, &name)?,
    ];
    Ok((apply_edits(src, &edits), name))
}

/// 戻り値は (新ソース, 追加された binding 名)。
pub fn add_point(src: &str, binding: &str, pos: [f64; 2]) -> Result<(String, String), String> {
    let rhs = format!("p2 {} {}", fmt_coord(pos[0]), fmt_coord(pos[1]));
    insert_geom(src, binding, "pt", &rhs)
}

/// 戻り値は (新ソース, 追加された binding 名)。
pub fn add_circle(
    src: &str,
    binding: &str,
    center: [f64; 2],
    radius: f64,
) -> Result<(String, String), String> {
    let rhs = if center == [0.0, 0.0] {
        format!("circle {}", fmt_coord(radius))
    } else {
        format!(
            "circle {} |> translate2d (p2 0.0 0.0) (p2 {} {})",
            fmt_coord(radius),
            fmt_coord(center[0]),
            fmt_coord(center[1])
        )
    };
    insert_geom(src, binding, "circ", &rhs)
}

/// 戻り値は (新ソース, 追加された binding 名)。
pub fn add_polygon(
    src: &str,
    binding: &str,
    verts: &[[f64; 2]],
) -> Result<(String, String), String> {
    if verts.len() < 2 {
        return Err("polygon には 2 点以上必要です".to_string());
    }
    let pts = verts
        .iter()
        .map(|p| format!("p2 {} {}", fmt_coord(p[0]), fmt_coord(p[1])))
        .collect::<Vec<_>>()
        .join(", ");
    let rhs = format!("polygon (segments [{pts}])");
    insert_geom(src, binding, "poly", &rhs)
}

/// `polygon (segments [...])` 形式の polygon の末尾に頂点を足す。
pub fn append_polygon_vertex(
    src: &str,
    binding: &str,
    geom_name: &str,
    pos: [f64; 2],
) -> Result<String, String> {
    let module = parse_or_msg(src)?;
    let (bindings, _body, _span) = find_block(&module, binding)?;
    let b = bindings
        .iter()
        .find(|b| b.kind == SketchBindKind::Bare && b.name == geom_name)
        .ok_or_else(|| format!("幾何 `{geom_name}` が見つかりません"))?;
    let (head, args) = app_spine(&b.body);
    if head != Some("polygon") || args.len() != 1 {
        return Err(format!("`{geom_name}` は polygon ではありません"));
    }
    let (h2, a2) = app_spine(args[0]);
    let list = match (h2, a2.as_slice()) {
        (Some("segments"), [Expr::List(items, lspan)]) => (items, *lspan),
        _ => {
            return Err(format!(
                "`{geom_name}` は `polygon (segments [...])` 形式ではないため頂点追加できません"
            ));
        }
    };
    let txt = format!("p2 {} {}", fmt_coord(pos[0]), fmt_coord(pos[1]));
    let edit = match list.0.last() {
        Some(last) => TextEdit {
            span: Span::new(last.span().end, last.span().end),
            replacement: format!(", {txt}"),
        },
        None => TextEdit {
            span: Span::new(list.1.start + 1, list.1.start + 1),
            replacement: txt,
        },
    };
    Ok(apply_edits(src, &[edit]))
}

/// 幾何 binding とその body field を削除する。
pub fn remove_geom(src: &str, binding: &str, geom_name: &str) -> Result<String, String> {
    let module = parse_or_msg(src)?;
    let (bindings, body, _span) = find_block(&module, binding)?;
    let b = bindings
        .iter()
        .find(|b| b.name == geom_name)
        .ok_or_else(|| format!("`{geom_name}` が見つかりません"))?;
    if bindings.len() == 1 {
        return Err("最後の要素は削除できません".to_string());
    }
    // 他の束縛から参照されていたら削除できない (点や線分が他形状の一部の場合)。
    for other in bindings.iter().filter(|o| o.name != geom_name) {
        let mut bound = HashSet::new();
        let mut refs = HashSet::new();
        free_var_names_in(&other.body, &mut bound, &mut refs);
        if refs.contains(geom_name) {
            return Err(format!(
                "`{geom_name}` は `{}` から参照されているため削除できません",
                other.name
            ));
        }
    }
    let mut edits: Vec<TextEdit> = Vec::new();
    // binding 行の削除 (行頭〜行末の改行まで)
    let line_start = src[..b.span.start].rfind('\n').map(|i| i + 1).unwrap_or(0);
    let line_end = src[b.span.end..]
        .find('\n')
        .map(|i| b.span.end + i + 1)
        .unwrap_or(src.len());
    edits.push(TextEdit {
        span: Span::new(line_start, line_end),
        replacement: String::new(),
    });
    // body field の削除
    match body {
        Expr::Record(fields, span) => {
            let idx = fields.iter().position(
                |f| matches!(&f.value, Expr::Var { name, .. } if name == geom_name),
            );
            if let Some(i) = idx {
                let edit = if fields.len() == 1 {
                    TextEdit {
                        span: *span,
                        replacement: "{}".to_string(),
                    }
                } else if i == 0 {
                    TextEdit {
                        span: Span::new(fields[0].span.start, fields[1].span.start),
                        replacement: String::new(),
                    }
                } else {
                    TextEdit {
                        span: Span::new(fields[i - 1].span.end, fields[i].span.end),
                        replacement: String::new(),
                    }
                };
                edits.push(edit);
            }
        }
        Expr::Var { name, span, .. } if name == geom_name => {
            edits.push(TextEdit {
                span: *span,
                replacement: "{}".to_string(),
            });
        }
        _ => {}
    }
    Ok(apply_edits(src, &edits))
}

/// 座標式のうちリテラル葉だけを出現順で集める。junction などで共有された
/// 同一 span は 1 回だけ数える。
fn collect_lit_leaves(
    exprs: &[&Expr],
    out: &mut Vec<(Span, f64)>,
    seen: &mut HashSet<(usize, usize)>,
) {
    for e in exprs {
        if let Some((v, leaf)) = signed_lit_leaf(e) {
            if seen.insert((leaf.span.start, leaf.span.end)) {
                out.push((leaf.span, v));
            }
        }
    }
}

/// 2 回以上現れる同値の座標リテラルを軸ごとに `var xN` / `var yN` へまとめる。
/// 対象は座標式全体がリテラルであるもののみ。既に var / 式になっている座標、
/// circle の半径と translate2d の src は対象外。まとめた座標は以後ドラッグで連動する。
/// x と y は値が同じでも別の var にする (共有すると斜めドラッグが衝突拒否されるため)。
pub fn factor_vars(src: &str, binding: &str) -> Result<String, String> {
    let module = parse_or_msg(src)?;
    let (bindings, _body, _span) = find_block(&module, binding)?;
    let block = Block::build(&module, bindings)?;

    // 軸ごと (0 = x, 1 = y) の座標リテラルを出現順で集める
    let mut leaves: [Vec<(Span, f64)>; 2] = [Vec::new(), Vec::new()];
    let mut seen: HashSet<(usize, usize)> = HashSet::new();
    for b in bindings.iter().filter(|b| b.kind == SketchBindKind::Bare) {
        let slots: Vec<PosSlot> = match block.geom_shape(b)? {
            GeomShape::Point(s) => vec![s],
            GeomShape::Segment(a, b) => vec![a, b],
            GeomShape::Polygon(v) => v,
            GeomShape::Circle(c) => c.dst.into_iter().collect(),
        };
        for s in &slots {
            collect_lit_leaves(&s.xs, &mut leaves[0], &mut seen);
            collect_lit_leaves(&s.ys, &mut leaves[1], &mut seen);
        }
    }

    // トップレベル var は shadow できないので、生成名から除外する。
    let mut taken: HashSet<String> = bindings.iter().map(|b| b.name.clone()).collect();
    taken.extend(block.top_vars.keys().map(|n| n.to_string()));
    let mut decls: Vec<(String, f64)> = Vec::new();
    let mut edits: Vec<TextEdit> = Vec::new();
    for (axis, prefix) in [(0, "x"), (1, "y")] {
        // 値ごとにグループ化 (初出順)。ビット比較なのでリテラル同士なら厳密一致。
        let mut groups: Vec<(u64, Vec<Span>)> = Vec::new();
        for (span, v) in &leaves[axis] {
            let bits = v.to_bits();
            match groups.iter_mut().find(|(b, _)| *b == bits) {
                Some((_, spans)) => spans.push(*span),
                None => groups.push((bits, vec![*span])),
            }
        }
        for (bits, spans) in groups {
            if spans.len() < 2 {
                continue;
            }
            let name = fresh_name(prefix, |n| taken.contains(n));
            taken.insert(name.clone());
            decls.push((name.clone(), f64::from_bits(bits)));
            for span in spans {
                edits.push(TextEdit {
                    span,
                    replacement: name.clone(),
                });
            }
        }
    }
    if decls.is_empty() {
        return Err("まとめられる重複座標がありません".to_string());
    }

    // var 宣言は前方参照禁止のため先頭の binding の直前に挿入する
    let first = bindings
        .first()
        .ok_or_else(|| "sketch ブロックに束縛がありません".to_string())?;
    let indent = line_indent(src, first.span.start);
    let mut header = String::new();
    for (name, v) in &decls {
        header.push_str(&format!("var {name} = {}\n{indent}", fmt_float(*v)));
    }
    edits.push(TextEdit {
        span: Span::new(first.span.start, first.span.start),
        replacement: header,
    });
    Ok(apply_edits(src, &edits))
}

// ---------------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const SRC_SEGMENTS: &str = "sk =\n    sketch\n        var x1 = 0.0\n        var x2 = 4.0\n        let y2 = 3.0\n        poly1 = polygon (segments [p2 x1 0.0, p2 x2 0.0, p2 x2 y2, p2 x1 y2])\n    in\n    { poly1 = poly1 }\n    end\n";

    const SRC_LINES: &str = "sk =\n    sketch\n        v1 = p2 0.0 0.0\n        v2 = p2 4.0 0.0\n        v3 = p2 4.0 3.0\n        l1 = line v1 v2\n        poly1 = polygon [l1, line v2 v3, line v3 v1]\n    in\n    { poly1 = poly1 }\n    end\n";

    fn model_of(src: &str) -> SketchModel {
        model_from_source(src, "sk").expect("model")
    }

    #[test]
    fn model_from_segments_polygon() {
        let m = model_of(SRC_SEGMENTS);
        assert_eq!(m.geoms.len(), 1);
        let SketchGeom::Polygon { name, verts } = &m.geoms[0] else {
            panic!("expected polygon: {:?}", m.geoms[0]);
        };
        assert_eq!(name, "poly1");
        assert_eq!(
            verts,
            &vec![[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]]
        );
    }

    #[test]
    fn model_from_line_polygon() {
        let m = model_of(SRC_LINES);
        // v1..v3 (点) + l1 (線分) + poly1
        assert_eq!(m.geoms.len(), 5);
        let SketchGeom::Polygon { verts, .. } = &m.geoms[4] else {
            panic!("expected polygon");
        };
        assert_eq!(verts, &vec![[0.0, 0.0], [4.0, 0.0], [4.0, 3.0]]);
    }

    #[test]
    fn model_circle_translated() {
        let src = "sk =\n    sketch\n        var r = 2.0\n        circ1 = circle r |> translate2d (p2 1.0 1.0) (p2 4.0 6.0)\n    in\n    { circ1 = circ1 }\n    end\n";
        let m = model_of(src);
        let SketchGeom::Circle { center, radius, .. } = &m.geoms[0] else {
            panic!("expected circle");
        };
        assert_eq!(*center, [3.0, 5.0]);
        assert_eq!(*radius, 2.0);
    }

    fn leaves_of(m: &SketchModel, t: DragTarget) -> Vec<usize> {
        m.links
            .iter()
            .find(|l| l.target == t)
            .unwrap_or_else(|| panic!("no link for {t:?}"))
            .leaves
            .clone()
    }

    fn shares_leaf(m: &SketchModel, a: DragTarget, b: DragTarget) -> bool {
        let la = leaves_of(m, a);
        leaves_of(m, b).iter().any(|l| la.contains(l))
    }

    #[test]
    fn links_shared_var_couples_vertices() {
        let m = model_of(SRC_SEGMENTS);
        // 頂点 0 と 3 は x1 を共有、頂点 0 と 1 は独立
        assert!(shares_leaf(
            &m,
            DragTarget::PolyVertex { geom: 0, vert: 0 },
            DragTarget::PolyVertex { geom: 0, vert: 3 },
        ));
        assert!(!shares_leaf(
            &m,
            DragTarget::PolyVertex { geom: 0, vert: 0 },
            DragTarget::PolyVertex { geom: 0, vert: 1 },
        ));
    }

    #[test]
    fn links_pinned_axis_has_no_leaf() {
        let m = model_of(SRC_SEGMENTS);
        // 頂点 2 は x=x2 (var), y=y2 (let → 書き込み不可) なので葉は x2 の 1 つだけ
        assert_eq!(
            leaves_of(&m, DragTarget::PolyVertex { geom: 0, vert: 2 }).len(),
            1
        );
    }

    #[test]
    fn links_named_point_couples_segment_and_polygon() {
        let m = model_of(SRC_LINES);
        // v2 (geom 1) は l1 (geom 3) の終点と poly1 (geom 4) の頂点 1 に現れる
        let v2 = DragTarget::Point { geom: 1 };
        assert!(shares_leaf(&m, v2, DragTarget::SegmentEnd { geom: 3, end: 1 }));
        assert!(shares_leaf(&m, v2, DragTarget::PolyVertex { geom: 4, vert: 1 }));
        assert!(!shares_leaf(&m, v2, DragTarget::Point { geom: 0 }));
    }

    #[test]
    fn sketch_binding_names_lists_top_level_sketches() {
        let module = parse::parse(SRC_SEGMENTS).unwrap();
        assert_eq!(sketch_binding_names(&module), vec!["sk".to_string()]);
    }

    #[test]
    fn drag_vertex_writes_shared_var() {
        // 頂点 0 (x1, 0.0) を (1.0, 0.5) へ: x1 は var → 書き換え。
        // x1 は頂点 3 でも使われているので連動する。
        let out = drag(
            SRC_SEGMENTS,
            "sk",
            DragTarget::PolyVertex { geom: 0, vert: 0 },
            DragValue::Pos([1.0, 0.5]),
        )
        .expect("drag");
        assert!(out.source.contains("var x1 = 1.0"), "{}", out.source);
        // 頂点 0 の y はインラインリテラル 0.0 → 0.5 に書き換え
        let SketchGeom::Polygon { verts, .. } = &out.model.geoms[0] else {
            panic!()
        };
        assert_eq!(verts[0], [1.0, 0.5]);
        assert_eq!(verts[3][0], 1.0, "x1 共有頂点も連動する");
        assert!(out.pinned.is_empty(), "{:?}", out.pinned);
    }

    #[test]
    fn drag_pinned_axis_is_reported() {
        // y2 は let → y 軸固定。x のみ適用される。
        let out = drag(
            SRC_SEGMENTS,
            "sk",
            DragTarget::PolyVertex { geom: 0, vert: 2 },
            DragValue::Pos([5.0, 9.0]),
        )
        .expect("drag");
        assert!(out.source.contains("var x2 = 5.0"), "{}", out.source);
        assert!(out.source.contains("let y2 = 3.0"), "{}", out.source);
        assert_eq!(out.pinned.len(), 1);
        assert!(out.pinned[0].contains("y"), "{:?}", out.pinned);
    }

    #[test]
    fn drag_all_pinned_is_rejected() {
        let src = "sk =\n    sketch\n        let x = 1.0\n        let y = 2.0\n        p = p2 x y\n    in\n    { p = p }\n    end\n";
        let err = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([3.0, 4.0]),
        )
        .expect_err("should reject");
        assert!(matches!(err, DragReject::ReadOnly(_)), "{err:?}");
    }

    #[test]
    fn drag_ambiguous_is_rejected() {
        let src = "sk =\n    sketch\n        var a = 1.0\n        var b = 2.0\n        p = p2 (a + b) (a + b)\n    in\n    { p = p }\n    end\n";
        let err = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([3.0, 4.0]),
        )
        .expect_err("should reject");
        assert!(matches!(err, DragReject::Ambiguous(_)), "{err:?}");
    }

    #[test]
    fn drag_offset_pattern_preserves_literal() {
        // `x1 + 1.0` は x1 に押し込まれ、オフセット 1.0 は保持される。
        let src = "sk =\n    sketch\n        var x1 = 2.0\n        p = p2 (x1 + 1.0) 0.0\n    in\n    { p = p }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([5.0, 0.0]),
        )
        .expect("drag");
        assert!(out.source.contains("var x1 = 4.0"), "{}", out.source);
        assert!(out.source.contains("x1 + 1.0"), "{}", out.source);
    }

    #[test]
    fn drag_through_let_pushes_into_var() {
        // let は導出値: t = b + 150.0 の t をドラッグすると b が書き換わる。
        let src = "sk =\n    sketch\n        var b = -40.0\n        let t = b + 150.0\n        p = p2 0.0 t\n    in\n    { p = p }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([0.0, 120.0]),
        )
        .expect("drag");
        assert!(out.source.contains("var b = -30.0"), "{}", out.source);
        assert!(out.source.contains("let t = b + 150.0"), "{}", out.source);
        let SketchGeom::Point { pos, .. } = &out.model.geoms[0] else {
            panic!()
        };
        assert_eq!(*pos, [0.0, 120.0]);
    }

    #[test]
    fn drag_through_let_chain() {
        // let の多段: t2 = t * 2.0, t = b + 5.0 → b まで押し込む。
        let src = "sk =\n    sketch\n        var b = 10.0\n        let t = b + 5.0\n        let t2 = t * 2.0\n        p = p2 t2 0.0\n    in\n    { p = p }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([40.0, 0.0]),
        )
        .expect("drag");
        assert!(out.source.contains("var b = 15.0"), "{}", out.source);
    }

    #[test]
    fn drag_through_let_ambiguous_when_var_on_both_sides() {
        let src = "sk =\n    sketch\n        var b = 1.0\n        let t = b + 1.0\n        p = p2 (t + b) (t + b)\n    in\n    { p = p }\n    end\n";
        let err = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([9.0, 8.0]),
        )
        .expect_err("should reject");
        assert!(matches!(err, DragReject::Ambiguous(_)), "{err:?}");
    }

    #[test]
    fn drag_through_let_conflicts_with_direct_var_axis() {
        // x = t (→ b へ押し込み)、y = b。異なる値の書き込みは衝突。
        let src = "sk =\n    sketch\n        var b = 0.0\n        let t = b + 150.0\n        p = p2 t b\n    in\n    { p = p }\n    end\n";
        let err = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([160.0, 20.0]),
        )
        .expect_err("should conflict");
        assert!(matches!(err, DragReject::Conflict(_)), "{err:?}");
    }

    #[test]
    fn drag_through_let_reaches_top_level_var() {
        let src = "var zb = 5.0\nsk =\n    sketch\n        let t = zb + 1.0\n        p = p2 0.0 t\n    in\n    { p = p }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([0.0, 10.0]),
        )
        .expect("drag");
        assert!(out.source.contains("var zb = 9.0"), "{}", out.source);
    }

    #[test]
    fn drag_division_inverts_through_numerator() {
        let src = "sk =\n    sketch\n        var w = 8.0\n        p = p2 (w / 2.0) 0.0\n    in\n    { p = p }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([3.0, 0.0]),
        )
        .expect("drag");
        assert!(out.source.contains("var w = 6.0"), "{}", out.source);
    }

    #[test]
    fn drag_negative_literal_stays_parenthesized() {
        let src = "sk =\n    sketch\n        p = p2 (-2.0) 3.0\n    in\n    { p = p }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([-5.0, -1.0]),
        )
        .expect("drag");
        assert!(out.source.contains("p2 (-5.0) (-1.0)"), "{}", out.source);
        // 再パースできる
        model_from_source(&out.source, "sk").expect("reparse");
    }

    #[test]
    fn drag_conflict_same_var_both_axes() {
        let src = "sk =\n    sketch\n        var a = 1.0\n        p = p2 a a\n    in\n    { p = p }\n    end\n";
        let err = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([3.0, 4.0]),
        )
        .expect_err("should conflict");
        assert!(matches!(err, DragReject::Conflict(_)), "{err:?}");
    }

    #[test]
    fn drag_junction_updates_named_point_once() {
        // line 形式 polygon の junction は名前付き点 1 箇所の書き込みになる。
        let out = drag(
            SRC_LINES,
            "sk",
            DragTarget::PolyVertex { geom: 4, vert: 1 },
            DragValue::Pos([5.0, 1.0]),
        )
        .expect("drag");
        assert!(out.source.contains("v2 = p2 5.0 1.0"), "{}", out.source);
        let SketchGeom::Polygon { verts, .. } = &out.model.geoms[4] else {
            panic!()
        };
        assert_eq!(verts[1], [5.0, 1.0]);
    }

    #[test]
    fn drag_junction_inline_duplicate_literals_both_updated() {
        let src = "sk =\n    sketch\n        poly1 = polygon [line (p2 0.0 0.0) (p2 4.0 0.0), line (p2 4.0 0.0) (p2 0.0 3.0), line (p2 0.0 3.0) (p2 0.0 0.0)]\n    in\n    { poly1 = poly1 }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::PolyVertex { geom: 0, vert: 1 },
            DragValue::Pos([5.0, 1.0]),
        )
        .expect("drag");
        // junction (4.0, 0.0) の 2 出現が両方 (5.0, 1.0) になる
        assert_eq!(out.source.matches("p2 5.0 1.0").count(), 2, "{}", out.source);
        let SketchGeom::Polygon { verts, .. } = &out.model.geoms[0] else {
            panic!()
        };
        assert_eq!(verts[1], [5.0, 1.0]);
    }

    #[test]
    fn drag_circle_center_and_radius() {
        let src = "sk =\n    sketch\n        circ1 = circle 2.0 |> translate2d (p2 0.0 0.0) (p2 4.0 5.0)\n    in\n    { circ1 = circ1 }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::CircleCenter { geom: 0 },
            DragValue::Pos([7.0, 8.0]),
        )
        .expect("drag center");
        assert!(out.source.contains("p2 7.0 8.0"), "{}", out.source);
        let out2 = drag(
            &out.source,
            "sk",
            DragTarget::CircleRadius { geom: 0 },
            DragValue::Radius(3.5),
        )
        .expect("drag radius");
        assert!(out2.source.contains("circle 3.5"), "{}", out2.source);
    }

    #[test]
    fn drag_origin_circle_center_rejected() {
        let src = "sk =\n    sketch\n        circ1 = circle 2.0\n    in\n    { circ1 = circ1 }\n    end\n";
        let err = drag(
            src,
            "sk",
            DragTarget::CircleCenter { geom: 0 },
            DragValue::Pos([1.0, 1.0]),
        )
        .expect_err("should reject");
        assert!(matches!(err, DragReject::ReadOnly(_)), "{err:?}");
    }

    #[test]
    fn add_point_and_remove_geom_roundtrip() {
        let (s1, _) = add_point(SRC_SEGMENTS, "sk", [3.0, -2.0]).expect("add_point");
        assert!(s1.contains("pt1 = p2 3.0 (-2.0)"), "{s1}");
        assert!(s1.contains("{ poly1 = poly1, pt1 = pt1 }"), "{s1}");
        let m = model_from_source(&s1, "sk").expect("model");
        assert_eq!(m.geoms.len(), 2);

        let s2 = remove_geom(&s1, "sk", "pt1").expect("remove");
        assert!(!s2.contains("pt1"), "{s2}");
        assert!(s2.contains("{ poly1 = poly1 }"), "{s2}");
    }

    #[test]
    fn add_circle_and_polygon() {
        let (s1, _) = add_circle(SRC_SEGMENTS, "sk", [2.0, 3.0], 1.5).expect("add_circle");
        assert!(
            s1.contains("circ1 = circle 1.5 |> translate2d (p2 0.0 0.0) (p2 2.0 3.0)"),
            "{s1}"
        );
        let (s2, poly_name) = add_polygon(&s1, "sk", &[[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]).expect("add_polygon");
        assert_eq!(poly_name, "poly2");
        assert!(
            s2.contains("poly2 = polygon (segments [p2 0.0 0.0, p2 1.0 0.0, p2 0.0 1.0])"),
            "{s2}"
        );
        let m = model_from_source(&s2, "sk").expect("model");
        assert_eq!(m.geoms.len(), 3);
    }

    #[test]
    fn append_vertex_to_segments_polygon() {
        let s = append_polygon_vertex(SRC_SEGMENTS, "sk", "poly1", [9.0, 9.0]).expect("append");
        assert!(s.contains("p2 x1 y2, p2 9.0 9.0])"), "{s}");
        let m = model_from_source(&s, "sk").expect("model");
        let SketchGeom::Polygon { verts, .. } = &m.geoms[0] else {
            panic!()
        };
        assert_eq!(verts.len(), 5);
    }

    #[test]
    fn append_vertex_to_line_polygon_rejected() {
        let err =
            append_polygon_vertex(SRC_LINES, "sk", "poly1", [9.0, 9.0]).expect_err("reject");
        assert!(err.contains("segments"), "{err}");
    }

    #[test]
    fn remove_referenced_point_rejected() {
        let err = remove_geom(SRC_LINES, "sk", "v1").expect_err("reject");
        assert!(err.contains("参照されている"), "{err}");
    }

    #[test]
    fn remove_middle_record_field() {
        let src = "sk =\n    sketch\n        pt1 = p2 0.0 0.0\n        pt2 = p2 1.0 0.0\n        pt3 = p2 2.0 0.0\n    in\n    { pt1 = pt1, pt2 = pt2, pt3 = pt3 }\n    end\n";
        let s = remove_geom(src, "sk", "pt2").expect("remove");
        assert!(s.contains("{ pt1 = pt1, pt3 = pt3 }"), "{s}");
        assert!(!s.contains("pt2"), "{s}");
    }

    #[test]
    fn remove_last_field_makes_empty_record() {
        let src = "sk =\n    sketch\n        pt1 = p2 0.0 0.0\n        pt2 = p2 1.0 0.0\n    in\n    { pt2 = pt2 }\n    end\n";
        let s = remove_geom(src, "sk", "pt2").expect("remove");
        assert!(s.contains("{}"), "{s}");
    }

    #[test]
    fn drag_after_structural_edit_roundtrip() {
        // 追加した点をドラッグ → モデルに反映される
        let (s1, _) = add_point(SRC_SEGMENTS, "sk", [1.0, 1.0]).expect("add");
        let out = drag(
            &s1,
            "sk",
            DragTarget::Point { geom: 1 },
            DragValue::Pos([6.0, -7.5]),
        )
        .expect("drag");
        let SketchGeom::Point { pos, .. } = &out.model.geoms[1] else {
            panic!()
        };
        assert_eq!(*pos, [6.0, -7.5]);
    }

    #[test]
    fn add_point_negative_zero_is_normalized() {
        // キャンバスのスナップ計算は -0.0 を作ることがある
        let (s, _) = add_point(SRC_SEGMENTS, "sk", [11.0, -0.0]).expect("add");
        assert!(s.contains("p2 11.0 0.0"), "{s}");
        model_from_source(&s, "sk").expect("parses");
    }

    #[test]
    fn drag_to_negative_zero_writes_plain_zero() {
        let src = "sk =\n    sketch\n        pt1 = p2 1.0 2.0\n    in\n    { pt1 = pt1 }\n    end\n";
        let out = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([11.0, -0.0]),
        )
        .expect("drag");
        assert!(out.source.contains("p2 11.0 0.0"), "{}", out.source);
    }

    #[test]
    fn factor_vars_groups_per_axis() {
        let src = "sk =\n    sketch\n        poly1 = polygon (segments [p2 0.0 0.0, p2 4.0 0.0, p2 4.0 3.0, p2 0.0 3.0])\n    in\n    { poly1 = poly1 }\n    end\n";
        let before = model_of(src);
        let s = factor_vars(src, "sk").expect("factor");
        assert!(s.contains("var x1 = 0.0"), "{s}");
        assert!(s.contains("var x2 = 4.0"), "{s}");
        // 0.0 は y 軸にも 2 回現れるが、x1 とは別 var になる
        assert!(s.contains("var y1 = 0.0"), "{s}");
        assert!(s.contains("var y2 = 3.0"), "{s}");
        assert!(
            s.contains("segments [p2 x1 y1, p2 x2 y1, p2 x2 y2, p2 x1 y2]"),
            "{s}"
        );
        assert_eq!(model_of(&s).geoms, before.geoms, "形状は変わらない");
    }

    #[test]
    fn factor_vars_nothing_to_merge_is_rejected() {
        // x の 2.0 は var 参照とリテラルで 1 回ずつ → リテラル同士でないのでまとめない
        let src = "sk =\n    sketch\n        var x1 = 2.0\n        pt1 = p2 x1 1.0\n        pt2 = p2 2.0 5.0\n    in\n    { pt1 = pt1, pt2 = pt2 }\n    end\n";
        let err = factor_vars(src, "sk").expect_err("reject");
        assert!(err.contains("まとめられる"), "{err}");
    }

    #[test]
    fn factor_vars_avoids_existing_names() {
        let src = "sk =\n    sketch\n        var x1 = 9.0\n        pt1 = p2 x1 0.0\n        pt2 = p2 5.0 0.0\n        pt3 = p2 5.0 1.0\n    in\n    { pt1 = pt1, pt2 = pt2, pt3 = pt3 }\n    end\n";
        let s = factor_vars(src, "sk").expect("factor");
        assert!(s.contains("var x2 = 5.0"), "{s}");
        assert!(s.contains("p2 x2 0.0") || s.contains("p2 x2 y1"), "{s}");
        model_from_source(&s, "sk").expect("parses");
    }

    #[test]
    fn factor_vars_shares_junction_literals_once() {
        // line 形式 polygon の junction (同一 span) は 1 回として数える。
        // x 軸は 4.0 のみ 2 回 (v2, v3)、y 軸は 0.0 のみ 2 回 (v1, v2)。
        let s = factor_vars(SRC_LINES, "sk").expect("factor");
        assert!(s.contains("var x1 = 4.0"), "{s}");
        assert!(s.contains("var y1 = 0.0"), "{s}");
        assert!(s.contains("v1 = p2 0.0 y1"), "{s}");
        assert!(s.contains("v2 = p2 x1 y1"), "{s}");
        assert!(s.contains("v3 = p2 x1 3.0"), "{s}");
        model_from_source(&s, "sk").expect("parses");
    }

    #[test]
    fn factor_vars_skips_circle_src_and_radius() {
        // translate2d の src (p2 0.0 0.0) と半径 3.0 は対象外
        let src = "sk =\n    sketch\n        pt1 = p2 0.0 3.0\n        circ1 = circle 3.0 |> translate2d (p2 0.0 0.0) (p2 0.0 6.0)\n    in\n    { pt1 = pt1, circ1 = circ1 }\n    end\n";
        // x: pt1 の 0.0 と dst の 0.0 の 2 回 → まとめる。src の 0.0 は数えない
        let s = factor_vars(src, "sk").expect("factor");
        assert!(s.contains("var x1 = 0.0"), "{s}");
        assert!(s.contains("pt1 = p2 x1 3.0"), "{s}");
        assert!(s.contains("circle 3.0"), "半径はまとめない: {s}");
        assert!(s.contains("translate2d (p2 0.0 0.0) (p2 x1 6.0)"), "{s}");
        model_from_source(&s, "sk").expect("parses");
    }

    #[test]
    fn factor_vars_negative_literals() {
        let src = "sk =\n    sketch\n        pt1 = p2 (-1.5) 0.0\n        pt2 = p2 (-1.5) 2.0\n    in\n    { pt1 = pt1, pt2 = pt2 }\n    end\n";
        let s = factor_vars(src, "sk").expect("factor");
        assert!(s.contains("var x1 = -1.5"), "{s}");
        let m = model_from_source(&s, "sk").expect("parses");
        assert_eq!(m.geoms.len(), 2);
    }

    const SRC_TOP_VAR: &str = "var z1 = 50.0\n\nskxz =\n    sketch\n        var x1 = 10.0\n        p = p2 x1 z1\n    in\n    { p = p }\n    end\n\nskyz =\n    sketch\n        q = p2 3.0 z1\n        r = p2 4.0 (z1 + 10.0)\n    in\n    { q = q, r = r }\n    end\n";

    #[test]
    fn model_reads_top_level_var() {
        let m = model_from_source(SRC_TOP_VAR, "skxz").expect("model");
        let SketchGeom::Point { pos, .. } = &m.geoms[0] else {
            panic!()
        };
        assert_eq!(*pos, [10.0, 50.0]);
        let m2 = model_from_source(SRC_TOP_VAR, "skyz").expect("model");
        let SketchGeom::Point { pos, .. } = &m2.geoms[1] else {
            panic!()
        };
        assert_eq!(*pos, [4.0, 60.0]);
    }

    #[test]
    fn drag_writes_top_level_var_and_other_sketch_follows() {
        let out = drag(
            SRC_TOP_VAR,
            "skxz",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([10.0, 70.0]),
        )
        .expect("drag");
        assert!(out.source.contains("var z1 = 70.0"), "{}", out.source);
        // 同じ z1 を参照する別 sketch も新ソースでは連動する
        let m2 = model_from_source(&out.source, "skyz").expect("model");
        let SketchGeom::Point { pos, .. } = &m2.geoms[0] else {
            panic!()
        };
        assert_eq!(*pos, [3.0, 70.0]);
    }

    #[test]
    fn drag_offset_expr_pushes_into_top_level_var() {
        // r の y は `z1 + 10.0` → z1 に押し込まれ、オフセットは保持される。
        let out = drag(
            SRC_TOP_VAR,
            "skyz",
            DragTarget::Point { geom: 1 },
            DragValue::Pos([4.0, 90.0]),
        )
        .expect("drag");
        assert!(out.source.contains("var z1 = 80.0"), "{}", out.source);
        assert!(out.source.contains("z1 + 10.0"), "{}", out.source);
    }

    #[test]
    fn local_binding_shadows_top_var_in_block() {
        // sema は shadow を拒否するが、Block の名前解決はブロック内優先で一貫させる。
        let src = "var a = 1.0\nsk =\n    sketch\n        let a = 5.0\n        p = p2 a a\n    in\n    { p = p }\n    end\n";
        let m = model_from_source(src, "sk").expect("model");
        let SketchGeom::Point { pos, .. } = &m.geoms[0] else {
            panic!()
        };
        assert_eq!(*pos, [5.0, 5.0]);
        // ブロック内 let 優先なので両軸とも書き込み不可
        let err = drag(
            src,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([9.0, 9.0]),
        )
        .expect_err("reject");
        assert!(matches!(err, DragReject::ReadOnly(_)), "{err:?}");
    }

    #[test]
    fn factor_vars_avoids_top_level_var_names() {
        let src = "var x1 = 9.0\nsk =\n    sketch\n        pt1 = p2 5.0 0.0\n        pt2 = p2 5.0 1.0\n    in\n    { pt1 = pt1, pt2 = pt2 }\n    end\n";
        let s = factor_vars(src, "sk").expect("factor");
        // トップレベル x1 と衝突しない名前が生成される
        assert!(s.contains("var x2 = 5.0"), "{s}");
        model_from_source(&s, "sk").expect("parses");
    }

    #[test]
    fn factor_vars_then_drag_moves_shared() {
        let src = "sk =\n    sketch\n        pt1 = p2 4.0 0.0\n        pt2 = p2 4.0 2.0\n    in\n    { pt1 = pt1, pt2 = pt2 }\n    end\n";
        let s = factor_vars(src, "sk").expect("factor");
        let out = drag(
            &s,
            "sk",
            DragTarget::Point { geom: 0 },
            DragValue::Pos([7.0, 0.0]),
        )
        .expect("drag");
        let SketchGeom::Point { pos, .. } = &out.model.geoms[1] else {
            panic!()
        };
        assert_eq!(pos[0], 7.0, "共有 var 経由で pt2 も連動する");
    }
}
