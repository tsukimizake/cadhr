//! `case` 式の網羅性 (exhaustiveness) 検査。
//!
//! ネストパターンも追える Maranget 風の簡易マトリックスアルゴリズム。
//! 列ごとに「irrefutable な行があれば網羅」「そうでなければ ADT/Bool/List の
//! 全 ctor が揃っていて、各 ctor の arg 位置でも再帰的に網羅」と判定する。
//!
//! 型情報の取り方:
//! - ADT の `type Foo a = C1 X | C2 Y Z` を集めて `ctor → (Foo, [X, Y, ...])` で持つ
//! - `Bool` / `List` は組み込みの仮想 ADT として扱う
//! - 数値・文字列・未知の型は「無限の値集合」と見て、wildcard/var が無ければ非網羅
//!
//! `compile()` が warning として吐く。

use crate::diagnostic::{Diagnostic, Span};
use crate::syntax::ast::*;
use std::collections::HashMap;

pub fn check_module(m: &Module) -> Vec<Diagnostic> {
    let mut env = AdtEnv::default();
    env.collect(m);
    let mut diag = Vec::new();
    for d in &m.decls {
        if let Decl::Value(v) = d {
            walk_expr(&v.body, &env, &mut diag);
        }
    }
    diag
}

#[derive(Default)]
struct AdtEnv {
    /// ctor 名 → 属する ADT 名。
    ctor_to_type: HashMap<String, String>,
    /// ADT 名 → 全 ctor 名 (宣言順)。
    type_to_ctors: HashMap<String, Vec<String>>,
    /// ctor 名 → arg の型式 (位置順)。再帰時に各 arg の型ヒントを得るのに使う。
    ctor_arg_types: HashMap<String, Vec<TypeExpr>>,
}

impl AdtEnv {
    fn collect(&mut self, m: &Module) {
        for d in &m.decls {
            if let Decl::Type(t) = d {
                let ctor_names: Vec<String> =
                    t.constructors.iter().map(|c| c.name.clone()).collect();
                for c in &t.constructors {
                    self.ctor_to_type.insert(c.name.clone(), t.name.clone());
                    self.ctor_arg_types.insert(c.name.clone(), c.args.clone());
                }
                self.type_to_ctors.insert(t.name.clone(), ctor_names);
            }
        }
    }
}

fn walk_expr(e: &Expr, env: &AdtEnv, diag: &mut Vec<Diagnostic>) {
    match e {
        Expr::Case {
            scrutinee,
            arms,
            span,
        } => {
            walk_expr(scrutinee, env, diag);
            for a in arms {
                if let Some(g) = &a.guard {
                    walk_expr(g, env, diag);
                }
                walk_expr(&a.body, env, diag);
            }
            check_case(arms, *span, env, diag);
        }
        Expr::App { func, arg, .. } => {
            walk_expr(func, env, diag);
            walk_expr(arg, env, diag);
        }
        Expr::Lambda { body, .. } => walk_expr(body, env, diag),
        Expr::Let { bindings, body, .. } => {
            for b in bindings {
                walk_expr(&b.body, env, diag);
            }
            walk_expr(body, env, diag);
        }
        Expr::If {
            cond,
            then_branch,
            else_branch,
            ..
        } => {
            walk_expr(cond, env, diag);
            walk_expr(then_branch, env, diag);
            walk_expr(else_branch, env, diag);
        }
        Expr::List(items, _) => {
            for i in items {
                walk_expr(i, env, diag);
            }
        }
        Expr::Record(fields, _) => {
            for f in fields {
                walk_expr(&f.value, env, diag);
            }
        }
        Expr::RecordUpdate { base, updates, .. } => {
            walk_expr(base, env, diag);
            for u in updates {
                walk_expr(&u.value, env, diag);
            }
        }
        Expr::Field { receiver, .. } => walk_expr(receiver, env, diag),
        Expr::BinOp { left, right, .. } => {
            walk_expr(left, env, diag);
            walk_expr(right, env, diag);
        }
        Expr::Negate(inner, _) => walk_expr(inner, env, diag),
        Expr::Range { lo, hi, .. } => {
            walk_expr(lo, env, diag);
            walk_expr(hi, env, diag);
        }
        Expr::Var { .. } | Expr::Ctor { .. } | Expr::Lit(..) | Expr::Error(_) => {}
    }
}

// --------- 型ヒント ----------

/// pattern 列がカバーすべき値集合の見当。`TypeExpr` から作る。
#[derive(Clone, Debug)]
enum TypeHint {
    Bool,
    /// 既知の ADT 名。`AdtEnv` に decl がある前提。
    Adt(String),
    /// `List elem_type_hint`。
    List(Box<TypeHint>),
    /// それ以外: 無限集合 (Int / Float / String / 未知の Con / 型変数 / 関数型 / record) →
    /// wildcard/var が無ければ非網羅判定。
    Other,
}

fn type_hint_from(t: &TypeExpr, env: &AdtEnv) -> TypeHint {
    match t {
        TypeExpr::Con { name, args, .. } => match name.as_str() {
            "Bool" => TypeHint::Bool,
            "List" => {
                let inner = if let Some(a) = args.first() {
                    type_hint_from(a, env)
                } else {
                    TypeHint::Other
                };
                TypeHint::List(Box::new(inner))
            }
            other => {
                if env.type_to_ctors.contains_key(other) {
                    TypeHint::Adt(other.to_string())
                } else {
                    TypeHint::Other
                }
            }
        },
        _ => TypeHint::Other,
    }
}

// --------- 網羅性判定 ----------

fn unwrap_as(p: &Pattern) -> &Pattern {
    match p {
        Pattern::As { inner, .. } => unwrap_as(inner),
        other => other,
    }
}

fn is_irrefutable(p: &Pattern) -> bool {
    match unwrap_as(p) {
        Pattern::Var(_, _) | Pattern::Wildcard(_) | Pattern::Record(_, _) => true,
        _ => false,
    }
}

/// 「これら patterns でこの型 hint の全値を網羅しているか」を判定。
/// 不足があれば missing 文字列を返す (網羅なら None)。
fn missing_for(patterns: &[&Pattern], env: &AdtEnv, hint: &TypeHint) -> Option<String> {
    // 1 つでも irrefutable があれば網羅
    if patterns.iter().any(|p| is_irrefutable(p)) {
        return None;
    }

    match hint {
        TypeHint::Bool => {
            let t = patterns
                .iter()
                .any(|p| matches!(unwrap_as(p), Pattern::Lit(Lit::Bool(true), _)));
            let f = patterns
                .iter()
                .any(|p| matches!(unwrap_as(p), Pattern::Lit(Lit::Bool(false), _)));
            match (t, f) {
                (true, true) => None,
                (true, false) => Some("False".into()),
                (false, true) => Some("True".into()),
                (false, false) => Some("True | False".into()),
            }
        }
        TypeHint::Adt(name) => {
            let ctors = env.type_to_ctors.get(name).cloned().unwrap_or_default();
            let mut missing_ctors: Vec<String> = Vec::new();
            let mut missing_inner: Vec<String> = Vec::new();
            for ctor in &ctors {
                let groups: Vec<&Vec<Pattern>> = patterns
                    .iter()
                    .filter_map(|p| match unwrap_as(p) {
                        Pattern::Ctor { name: n, args, .. } if n == ctor => Some(args),
                        _ => None,
                    })
                    .collect();
                if groups.is_empty() {
                    missing_ctors.push(ctor.clone());
                    continue;
                }
                // ctor が現れているので、各 arg 位置で再帰的に網羅性を確認
                let arg_types = env.ctor_arg_types.get(ctor).cloned().unwrap_or_default();
                let n_args = arg_types.len();
                for i in 0..n_args {
                    let col: Vec<&Pattern> = groups.iter().filter_map(|g| g.get(i)).collect();
                    let sub_hint = type_hint_from(&arg_types[i], env);
                    if let Some(m) = missing_for(&col, env, &sub_hint) {
                        missing_inner.push(format!("{ctor} 引数 {} に {m}", i + 1));
                    }
                }
            }
            let mut parts = Vec::new();
            if !missing_ctors.is_empty() {
                parts.push(format!(
                    "{} の ctor {{ {} }}",
                    name,
                    missing_ctors.join(", ")
                ));
            }
            parts.extend(missing_inner);
            if parts.is_empty() {
                None
            } else {
                Some(parts.join(" / "))
            }
        }
        TypeHint::List(elem_hint) => {
            // [] と Cons の両方が必要。Cons は head/tail を再帰検査。
            let empties: Vec<()> = patterns
                .iter()
                .filter_map(|p| match unwrap_as(p) {
                    Pattern::List(items, _) if items.is_empty() => Some(()),
                    _ => None,
                })
                .collect();
            let has_empty = !empties.is_empty();

            // Cons の head / tail を集める。`x :: xs`、`[a, b]`、`[a, b | tail]` を共通形に分解。
            let mut cons_heads: Vec<Pattern> = Vec::new();
            let mut cons_tails: Vec<Pattern> = Vec::new();
            let mut has_cons = false;
            for p in patterns {
                if let Some((h, t)) = decompose_to_cons(unwrap_as(p)) {
                    has_cons = true;
                    cons_heads.push(h);
                    cons_tails.push(t);
                }
            }

            if !has_empty && !has_cons {
                return Some("[]、(_ :: _)".into());
            }
            if !has_empty {
                return Some("[]".into());
            }
            if !has_cons {
                return Some("(_ :: _)".into());
            }

            // head / tail の再帰
            let head_refs: Vec<&Pattern> = cons_heads.iter().collect();
            let tail_refs: Vec<&Pattern> = cons_tails.iter().collect();
            let mut inner = Vec::new();
            if let Some(m) = missing_for(&head_refs, env, elem_hint) {
                inner.push(format!("(_ :: _) の head に {m}"));
            }
            // tail は同じ List 型
            if let Some(m) = missing_for(&tail_refs, env, hint) {
                inner.push(format!("(_ :: _) の tail に {m}"));
            }
            if inner.is_empty() {
                None
            } else {
                Some(inner.join(" / "))
            }
        }
        TypeHint::Other => {
            // 数値・文字列・未知型: 1 つでも値リテラルが書かれていて wildcard
            // (irrefutable) が無いなら非網羅。
            // 上の早期 return で wildcard 無しは判明。何かパターンがある時点で非網羅。
            if patterns.is_empty() {
                Some("値が一切無い (空 case)".into())
            } else {
                Some("`_ ->` (catch-all)".into())
            }
        }
    }
}

/// `x :: xs` / `[a, b, c]` / `[a, b | tail]` を `(head, tail)` のペアに分解する。
/// 分解できないパターン (irrefutable は別ルートで判定済み) は `None`。
fn decompose_to_cons(p: &Pattern) -> Option<(Pattern, Pattern)> {
    match p {
        Pattern::Cons { head, tail, .. } => Some(((**head).clone(), (**tail).clone())),
        Pattern::List(items, span) if !items.is_empty() => {
            let head = items[0].clone();
            let tail_items = items[1..].to_vec();
            let tail = Pattern::List(tail_items, *span);
            Some((head, tail))
        }
        Pattern::As { inner, .. } => decompose_to_cons(inner),
        _ => None,
    }
}

fn check_case(arms: &[CaseArm], case_span: Span, env: &AdtEnv, diag: &mut Vec<Diagnostic>) {
    // guard 付き arm は「catch-all」にならないので irrefutable 判定から除外。
    // case の引数型は AST だけからは取れないので、最初の arm のパターン形状から
    // 推測する: ctor パターンならその ADT、Bool リテラルなら Bool、List なら List、
    // それ以外は Other。
    let usable_arms: Vec<&CaseArm> = arms.iter().filter(|a| a.guard.is_none()).collect();
    if usable_arms.is_empty() {
        // 全 arm に guard がある → 必ず非網羅。
        diag.push(Diagnostic::NonExhaustiveAllGuarded { span: case_span });
        return;
    }

    let hint = infer_hint_from_arms(&usable_arms, env);
    let pats: Vec<&Pattern> = usable_arms.iter().map(|a| &a.pattern).collect();
    if let Some(missing) = missing_for(&pats, env, &hint) {
        diag.push(Diagnostic::NonExhaustiveMissing {
            span: case_span,
            missing: format!("{missing}"),
        });
    }
}

fn infer_hint_from_arms(arms: &[&CaseArm], env: &AdtEnv) -> TypeHint {
    // 全 arm から hint を集めて、最も情報量の多い (Other より具体的な) ものを選ぶ。
    let hints: Vec<TypeHint> = arms
        .iter()
        .filter_map(|a| hint_from_pattern(&a.pattern, env))
        .collect();
    if hints.is_empty() {
        return TypeHint::Other;
    }
    // List 同士なら要素 hint を合成、それ以外は最初に見つけた具体型を採用。
    let mut acc = TypeHint::Other;
    for h in hints {
        acc = merge_hint(acc, h);
    }
    acc
}

fn merge_hint(a: TypeHint, b: TypeHint) -> TypeHint {
    match (a, b) {
        (TypeHint::Other, x) | (x, TypeHint::Other) => x,
        (TypeHint::List(ae), TypeHint::List(be)) => TypeHint::List(Box::new(merge_hint(*ae, *be))),
        (x, _) => x,
    }
}

fn hint_from_pattern(p: &Pattern, env: &AdtEnv) -> Option<TypeHint> {
    match unwrap_as(p) {
        Pattern::Ctor { name, .. } => env.ctor_to_type.get(name).cloned().map(TypeHint::Adt),
        Pattern::Lit(Lit::Bool(_), _) => Some(TypeHint::Bool),
        Pattern::Cons { head, .. } => {
            let elem = hint_from_pattern(head, env).unwrap_or(TypeHint::Other);
            Some(TypeHint::List(Box::new(elem)))
        }
        Pattern::List(items, _) => {
            let elem = items
                .first()
                .and_then(|h| hint_from_pattern(h, env))
                .unwrap_or(TypeHint::Other);
            Some(TypeHint::List(Box::new(elem)))
        }
        Pattern::Lit(_, _) => Some(TypeHint::Other),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diagnostic::Severity;
    use crate::syntax::parse::parse;

    fn run(src: &str) -> Vec<Diagnostic> {
        let m = parse(src).unwrap();
        check_module(&m)
    }

    #[test]
    fn wildcard_makes_exhaustive() {
        let src = "f x = case x of | 1 -> 1 | _ -> 0";
        assert!(run(src).is_empty());
    }

    #[test]
    fn var_makes_exhaustive() {
        let src = "f x = case x of | 1 -> 1 | n -> n";
        assert!(run(src).is_empty());
    }

    #[test]
    fn missing_bool_branch_is_warning() {
        let src = "f x = case x of | True -> 1";
        let d = run(src);
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].severity(), Severity::Warning);
        assert!(d[0].message().contains("False"));
    }

    #[test]
    fn both_bool_branches_ok() {
        let src = "f x = case x of | True -> 1 | False -> 0";
        assert!(run(src).is_empty());
    }

    #[test]
    fn missing_adt_ctor_is_warning() {
        let src = "type Shape = Cube Float Float Float | Sphere Float | Empty\n\
                   f s = case s of | Cube _ _ _ -> 1 | Sphere _ -> 2";
        let d = run(src);
        assert_eq!(d.len(), 1);
        assert!(d[0].message().contains("Empty"));
    }

    #[test]
    fn all_adt_ctors_ok() {
        let src = "type Shape = Cube Float Float Float | Sphere Float\n\
                   f s = case s of | Cube _ _ _ -> 1 | Sphere _ -> 2";
        assert!(run(src).is_empty());
    }

    #[test]
    fn missing_empty_list_is_warning() {
        let src = "f xs = case xs of | x :: _ -> x";
        let d = run(src);
        assert_eq!(d.len(), 1);
        assert!(d[0].message().contains("[]"));
    }

    #[test]
    fn missing_cons_list_is_warning() {
        let src = "f xs = case xs of | [] -> 0";
        let d = run(src);
        assert_eq!(d.len(), 1);
        assert!(d[0].message().contains("::"));
    }

    #[test]
    fn both_list_cases_ok() {
        let src = "f xs = case xs of | [] -> 0 | x :: _ -> x";
        assert!(run(src).is_empty());
    }

    #[test]
    fn int_lit_without_wildcard_is_warning() {
        let src = "f x = case x of | 0 -> 0 | 1 -> 1";
        let d = run(src);
        assert_eq!(d.len(), 1);
        assert!(d[0].message().contains("`_ ->`"));
    }

    #[test]
    fn nested_adt_missing_inner_ctor() {
        // Maybe a = Just a | Nothing
        // pair shape: ペア (Maybe x, Maybe y) のうち
        // `Just _ Just _` は書いてるが `Just _ Nothing` が漏れている
        let src = "type Maybe a = Just a | Nothing\n\
                   type Pair = MkPair (Maybe Float) (Maybe Float)\n\
                   f p = case p of | MkPair (Just _) (Just _) -> 1 | MkPair Nothing _ -> 2";
        let d = run(src);
        assert_eq!(d.len(), 1, "got {d:?}");
        // Maybe の Nothing が「MkPair の 2 番目の引数」位置で漏れているはず
        assert!(
            d[0].message().contains("Nothing") || d[0].message().contains("引数 2"),
            "{}",
            d[0].message()
        );
    }

    #[test]
    fn nested_adt_fully_covered() {
        let src = "type Maybe a = Just a | Nothing\n\
                   type Pair = MkPair (Maybe Float) (Maybe Float)\n\
                   f p = case p of | MkPair (Just _) (Just _) -> 1 | MkPair (Just _) Nothing -> 2 | MkPair Nothing _ -> 3";
        let d = run(src);
        assert!(d.is_empty(), "expected exhaustive, got {d:?}");
    }

    #[test]
    fn nested_bool_in_adt() {
        // ctor の引数が Bool だが True しか書いてない → 漏れ検出
        let src = "type Wrap = MkWrap Bool\n\
                   f w = case w of | MkWrap True -> 1";
        let d = run(src);
        assert_eq!(d.len(), 1, "{d:?}");
        assert!(d[0].message().contains("False"), "{}", d[0].message());
    }

    #[test]
    fn cons_head_bool_missing() {
        let src = "f xs = case xs of | [] -> 0 | True :: _ -> 1";
        let d = run(src);
        assert_eq!(d.len(), 1, "{d:?}");
        assert!(d[0].message().contains("False"), "{}", d[0].message());
    }

    #[test]
    fn nested_walks_into_lambda_and_let() {
        let src = "f x = \\y -> case y of | True -> 1";
        let d = run(src);
        assert_eq!(d.len(), 1);
    }

    #[test]
    fn fixed_length_list_pattern_missing_empty() {
        // [a, b] は cons cons nil に展開される。空リスト [] と 1 要素 [_] が漏れる
        let src = "f xs = case xs of | [a, b] -> a";
        let d = run(src);
        assert_eq!(d.len(), 1, "{d:?}");
    }
}
