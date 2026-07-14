//! AST → ソースコード文字列の pretty printer。
//!
//! parse → pretty の round-trip テストと、cadhr-lsp の formatting で使う。
//! 設計指針:
//! - 出力は deterministic (2 回 pretty しても結果が同じ)
//! - 演算子の優先順位を尊重して括弧を付ける
//! - インデントは 4 スペース固定
//! - 改行は decl の境界、`let` 内の body、関数定義の body などで挿入

use crate::syntax::ast::*;

const INDENT: &str = "    ";

/// 指定数の半角スペースを push する (layout 出力のインデント用)。
fn push_indent(n: usize, buf: &mut String) {
    for _ in 0..n {
        buf.push(' ');
    }
}

pub fn pretty_module(m: &Module) -> String {
    let mut buf = String::new();
    if let Some(h) = &m.header {
        buf.push_str("module ");
        pretty_module_name(&h.name, &mut buf);
        buf.push_str(" exposing ");
        pretty_exposing(&h.exposing, &mut buf);
        buf.push_str("\n\n");
    }
    for imp in &m.imports {
        pretty_import(imp, &mut buf);
        buf.push('\n');
    }
    if !m.imports.is_empty() {
        buf.push('\n');
    }
    for (i, decl) in m.decls.iter().enumerate() {
        if i > 0 {
            buf.push('\n');
        }
        pretty_decl(decl, &mut buf);
        buf.push('\n');
    }
    buf
}

fn pretty_module_name(n: &ModuleName, buf: &mut String) {
    for (i, seg) in n.segments.iter().enumerate() {
        if i > 0 {
            buf.push('.');
        }
        buf.push_str(seg);
    }
}

fn pretty_exposing(e: &Exposing, buf: &mut String) {
    buf.push('(');
    match e {
        Exposing::All(_) => buf.push_str(".."),
        Exposing::Some(items, _) => {
            for (i, item) in items.iter().enumerate() {
                if i > 0 {
                    buf.push_str(", ");
                }
                buf.push_str(&item.name);
                match &item.variant {
                    ExposingVariant::Bare => {}
                    ExposingVariant::AllCtors => buf.push_str("(..)"),
                    ExposingVariant::SomeCtors(ctors) => {
                        buf.push('(');
                        for (j, c) in ctors.iter().enumerate() {
                            if j > 0 {
                                buf.push_str(", ");
                            }
                            buf.push_str(c);
                        }
                        buf.push(')');
                    }
                }
            }
        }
    }
    buf.push(')');
}

fn pretty_import(imp: &Import, buf: &mut String) {
    buf.push_str("import ");
    pretty_module_name(&imp.module, buf);
    if let Some(alias) = &imp.alias {
        buf.push_str(" as ");
        buf.push_str(alias);
    }
    if let Some(exposing) = &imp.exposing {
        buf.push_str(" exposing ");
        pretty_exposing(exposing, buf);
    }
}

fn pretty_decl(d: &Decl, buf: &mut String) {
    match d {
        Decl::Signature(s) => {
            buf.push_str(&s.name);
            buf.push_str(" : ");
            pretty_type(&s.ty, buf);
        }
        Decl::Value(v) => {
            buf.push_str(&v.name);
            for p in &v.params {
                buf.push(' ');
                pretty_pattern_atom(p, buf);
            }
            buf.push_str(" =\n");
            buf.push_str(INDENT);
            pretty_expr(&v.body, INDENT.len(), buf);
        }
        Decl::Type(t) => {
            buf.push_str("type ");
            buf.push_str(&t.name);
            for p in &t.params {
                buf.push(' ');
                buf.push_str(p);
            }
            buf.push_str(" =");
            for (i, c) in t.constructors.iter().enumerate() {
                buf.push_str(if i == 0 { "\n    " } else { "\n    | " });
                buf.push_str(&c.name);
                for arg in &c.args {
                    buf.push(' ');
                    pretty_type_atom(arg, buf);
                }
            }
        }
        Decl::TypeAlias(a) => {
            buf.push_str("type alias ");
            buf.push_str(&a.name);
            for p in &a.params {
                buf.push(' ');
                buf.push_str(p);
            }
            buf.push_str(" =\n");
            buf.push_str(INDENT);
            pretty_type(&a.body, buf);
        }
        Decl::Slider(s) => {
            buf.push_str("slider ");
            if let Some(binding) = &s.binding {
                buf.push_str(binding);
                buf.push('.');
            }
            buf.push_str(&s.param);
            buf.push_str(" = ");
            pretty_expr(&s.body, 0, buf);
        }
    }
}

fn pretty_type(t: &TypeExpr, buf: &mut String) {
    match t {
        TypeExpr::Arrow { from, to, .. } => {
            pretty_type_app(from, buf);
            buf.push_str(" -> ");
            pretty_type(to, buf);
        }
        _ => pretty_type_app(t, buf),
    }
}

fn pretty_type_app(t: &TypeExpr, buf: &mut String) {
    match t {
        TypeExpr::Con {
            module, name, args, ..
        } if !args.is_empty() => {
            if let Some(m) = module {
                pretty_module_name(m, buf);
                buf.push('.');
            }
            buf.push_str(name);
            for a in args {
                buf.push(' ');
                pretty_type_atom(a, buf);
            }
        }
        _ => pretty_type_atom(t, buf),
    }
}

fn pretty_type_atom(t: &TypeExpr, buf: &mut String) {
    match t {
        TypeExpr::Var(s, _) => buf.push_str(s),
        TypeExpr::Con {
            module, name, args, ..
        } => {
            if args.is_empty() {
                if let Some(m) = module {
                    pretty_module_name(m, buf);
                    buf.push('.');
                }
                buf.push_str(name);
            } else {
                buf.push('(');
                pretty_type_app(t, buf);
                buf.push(')');
            }
        }
        TypeExpr::Arrow { .. } => {
            buf.push('(');
            pretty_type(t, buf);
            buf.push(')');
        }
        TypeExpr::Record(fields, _) => {
            buf.push_str("{ ");
            for (i, f) in fields.iter().enumerate() {
                if i > 0 {
                    buf.push_str(", ");
                }
                buf.push_str(&f.name);
                buf.push_str(" : ");
                pretty_type(&f.ty, buf);
            }
            buf.push_str(" }");
        }
    }
}

/// 式の pretty。優先度 0 から始める (括弧なし)。
/// `indent` は現在の式が属する行の字下げ列。case / let の継続行はこれを基準に深くする。
pub fn pretty_expr(e: &Expr, indent: usize, buf: &mut String) {
    pretty_expr_prec(e, 0, indent, buf);
}

fn pretty_expr_prec(e: &Expr, prec: u8, indent: usize, buf: &mut String) {
    match e {
        Expr::Var { module, name, .. } => {
            if let Some(m) = module {
                pretty_module_name(m, buf);
                buf.push('.');
            }
            buf.push_str(name);
        }
        Expr::Ctor { module, name, .. } => {
            if let Some(m) = module {
                pretty_module_name(m, buf);
                buf.push('.');
            }
            buf.push_str(name);
        }
        Expr::Lit(l, _) => pretty_lit(l, buf),
        Expr::List(items, _) => {
            buf.push('[');
            for (i, item) in items.iter().enumerate() {
                if i > 0 {
                    buf.push_str(", ");
                }
                pretty_expr(item, indent, buf);
            }
            buf.push(']');
        }
        Expr::Record(fields, _) => {
            buf.push_str("{ ");
            for (i, f) in fields.iter().enumerate() {
                if i > 0 {
                    buf.push_str(", ");
                }
                buf.push_str(&f.name);
                buf.push_str(" = ");
                pretty_expr(&f.value, indent, buf);
            }
            buf.push_str(" }");
        }
        Expr::RecordUpdate { base, updates, .. } => {
            buf.push_str("{ ");
            pretty_expr(base, indent, buf);
            buf.push_str(" | ");
            for (i, f) in updates.iter().enumerate() {
                if i > 0 {
                    buf.push_str(", ");
                }
                buf.push_str(&f.name);
                buf.push_str(" = ");
                pretty_expr(&f.value, indent, buf);
            }
            buf.push_str(" }");
        }
        Expr::Field { receiver, name, .. } => {
            pretty_expr_prec(receiver, 10, indent, buf);
            buf.push('.');
            buf.push_str(name);
        }
        Expr::App { func, arg, .. } => {
            if prec > 8 {
                buf.push('(');
            }
            pretty_expr_prec(func, 8, indent, buf);
            buf.push(' ');
            pretty_expr_prec(arg, 9, indent, buf);
            if prec > 8 {
                buf.push(')');
            }
        }
        Expr::Lambda { params, body, .. } => {
            if prec > 0 {
                buf.push('(');
            }
            buf.push('\\');
            for (i, p) in params.iter().enumerate() {
                if i > 0 {
                    buf.push(' ');
                }
                pretty_pattern_atom(p, buf);
            }
            buf.push_str(" -> ");
            pretty_expr(body, indent, buf);
            if prec > 0 {
                buf.push(')');
            }
        }
        Expr::Let { bindings, body, .. } => {
            if prec > 0 {
                buf.push('(');
            }
            // Elm 流の複数行 let。binding は indent+4、`in` と body は indent に揃える。
            buf.push_str("let");
            for b in bindings {
                buf.push('\n');
                push_indent(indent + 4, buf);
                buf.push_str(&b.name);
                for p in &b.params {
                    buf.push(' ');
                    pretty_pattern_atom(p, buf);
                }
                buf.push_str(" = ");
                pretty_expr(&b.body, indent + 4, buf);
            }
            buf.push('\n');
            push_indent(indent, buf);
            buf.push_str("in\n");
            push_indent(indent, buf);
            pretty_expr(body, indent, buf);
            if prec > 0 {
                buf.push(')');
            }
        }
        Expr::Sketch { bindings, body, .. } => {
            if prec > 0 {
                buf.push('(');
            }
            // binding 先頭に kind キーワードが付き、末尾に `end`。
            // `in` / body / `end` は binding と同列 (indent+4) に置く。let 束縛の
            // RHS に埋まっても外側レイアウトの列と衝突せず re-parse できる形。
            buf.push_str("sketch");
            for b in bindings {
                buf.push('\n');
                push_indent(indent + 4, buf);
                match b.kind {
                    SketchBindKind::Var => buf.push_str("var "),
                    SketchBindKind::Let => buf.push_str("let "),
                    SketchBindKind::Bare => {}
                }
                buf.push_str(&b.name);
                buf.push_str(" = ");
                pretty_expr(&b.body, indent + 4, buf);
            }
            buf.push('\n');
            push_indent(indent + 4, buf);
            buf.push_str("in\n");
            push_indent(indent + 4, buf);
            pretty_expr(body, indent + 4, buf);
            buf.push('\n');
            push_indent(indent + 4, buf);
            buf.push_str("end");
            if prec > 0 {
                buf.push(')');
            }
        }
        Expr::If {
            cond,
            then_branch,
            else_branch,
            ..
        } => {
            if prec > 0 {
                buf.push('(');
            }
            buf.push_str("if ");
            pretty_expr(cond, indent, buf);
            buf.push_str(" then ");
            pretty_expr(then_branch, indent, buf);
            buf.push_str(" else ");
            pretty_expr(else_branch, indent, buf);
            if prec > 0 {
                buf.push(')');
            }
        }
        Expr::Case {
            scrutinee, arms, ..
        } => {
            if prec > 0 {
                buf.push('(');
            }
            // Elm 流の複数行 case。arm の pattern は indent+4、body は次行 indent+8。
            buf.push_str("case ");
            pretty_expr(scrutinee, indent, buf);
            buf.push_str(" of");
            for (i, arm) in arms.iter().enumerate() {
                buf.push('\n');
                if i > 0 {
                    buf.push('\n'); // arm 間に空行 (Elm スタイル)
                }
                push_indent(indent + 4, buf);
                pretty_pattern(&arm.pattern, buf);
                buf.push_str(" ->\n");
                push_indent(indent + 8, buf);
                pretty_expr(&arm.body, indent + 8, buf);
            }
            if prec > 0 {
                buf.push(')');
            }
        }
        Expr::BinOp {
            op, left, right, ..
        } => {
            let (op_prec, op_str) = binop_info(*op);
            let needs_paren = prec > op_prec;
            if needs_paren {
                buf.push('(');
            }
            pretty_expr_prec(left, op_prec, indent, buf);
            buf.push(' ');
            buf.push_str(op_str);
            buf.push(' ');
            pretty_expr_prec(right, op_prec + 1, indent, buf);
            if needs_paren {
                buf.push(')');
            }
        }
        Expr::Negate(inner, _) => {
            buf.push('-');
            pretty_expr_prec(inner, 10, indent, buf);
        }
        Expr::Range { lo, hi, .. } => {
            pretty_expr_prec(lo, 5, indent, buf);
            buf.push_str(" .. ");
            pretty_expr_prec(hi, 5, indent, buf);
        }
        Expr::Error(_) => buf.push_str("<error>"),
    }
}

fn pretty_lit(l: &Lit, buf: &mut String) {
    match l {
        Lit::Int(n) => buf.push_str(&n.to_string()),
        Lit::Float(f) => {
            let s = format!("{f}");
            if s.contains('.') || s.contains('e') || s.contains('E') {
                buf.push_str(&s);
            } else {
                buf.push_str(&s);
                buf.push_str(".0");
            }
        }
        Lit::String(s) => {
            buf.push('"');
            for c in s.chars() {
                match c {
                    '"' => buf.push_str("\\\""),
                    '\\' => buf.push_str("\\\\"),
                    '\n' => buf.push_str("\\n"),
                    '\t' => buf.push_str("\\t"),
                    c => buf.push(c),
                }
            }
            buf.push('"');
        }
        Lit::Bool(true) => buf.push_str("True"),
        Lit::Bool(false) => buf.push_str("False"),
    }
}

fn pretty_pattern(p: &Pattern, buf: &mut String) {
    match p {
        Pattern::Var(s, _) => buf.push_str(s),
        Pattern::Wildcard(_) => buf.push('_'),
        Pattern::Lit(l, _) => pretty_lit(l, buf),
        Pattern::Ctor {
            module, name, args, ..
        } => {
            if let Some(m) = module {
                pretty_module_name(m, buf);
                buf.push('.');
            }
            buf.push_str(name);
            for a in args {
                buf.push(' ');
                pretty_pattern_atom(a, buf);
            }
        }
        Pattern::List(items, _) => {
            buf.push('[');
            for (i, p) in items.iter().enumerate() {
                if i > 0 {
                    buf.push_str(", ");
                }
                pretty_pattern(p, buf);
            }
            buf.push(']');
        }
        Pattern::Cons { head, tail, .. } => {
            pretty_pattern_atom(head, buf);
            buf.push_str(" :: ");
            pretty_pattern(tail, buf);
        }
        Pattern::Record(fields, _) => {
            buf.push_str("{ ");
            for (i, f) in fields.iter().enumerate() {
                if i > 0 {
                    buf.push_str(", ");
                }
                buf.push_str(f);
            }
            buf.push_str(" }");
        }
        Pattern::As { inner, name, .. } => {
            pretty_pattern_atom(inner, buf);
            buf.push_str(" as ");
            buf.push_str(name);
        }
    }
}

/// atomic context (関数引数・コンストラクタ引数) で使う pattern pretty。
fn pretty_pattern_atom(p: &Pattern, buf: &mut String) {
    match p {
        Pattern::Ctor { args, .. } if !args.is_empty() => {
            buf.push('(');
            pretty_pattern(p, buf);
            buf.push(')');
        }
        Pattern::Cons { .. } | Pattern::As { .. } => {
            buf.push('(');
            pretty_pattern(p, buf);
            buf.push(')');
        }
        _ => pretty_pattern(p, buf),
    }
}

fn binop_info(op: BinOp) -> (u8, &'static str) {
    match op {
        BinOp::ApplyR => (1, "|>"),
        BinOp::ApplyL => (1, "<|"),
        BinOp::Or => (2, "||"),
        BinOp::And => (3, "&&"),
        BinOp::Eq => (4, "=="),
        BinOp::NotEq => (4, "/="),
        BinOp::Lt => (4, "<"),
        BinOp::Le => (4, "<="),
        BinOp::Gt => (4, ">"),
        BinOp::Ge => (4, ">="),
        BinOp::Cons => (5, "::"),
        BinOp::Append => (5, "++"),
        BinOp::Add => (6, "+"),
        BinOp::Sub => (6, "-"),
        BinOp::Mul => (7, "*"),
        BinOp::Div => (7, "/"),
        BinOp::Compose => (9, ">>"),
        BinOp::ComposeR => (9, "<<"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::parse::parse;

    /// 1 回目の pretty 結果と、それを再 parse して 2 回目 pretty した結果が一致するか
    /// (= pretty が deterministic な fixed-point)。
    fn round_trip(src: &str) -> String {
        let m1 = parse(src).expect("parse 1 failed");
        let s1 = pretty_module(&m1);
        let m2 = parse(&s1).unwrap_or_else(|errs| {
            panic!("parse 2 failed on pretty output:\n--- pretty ---\n{s1}\n--- errs ---\n{errs:?}")
        });
        let s2 = pretty_module(&m2);
        assert_eq!(s1, s2, "pretty 出力が安定していません");
        s1
    }

    #[test]
    fn rt_empty() {
        round_trip("");
    }

    #[test]
    fn rt_simple_value() {
        round_trip("answer = 42");
    }

    #[test]
    fn rt_signature_and_value() {
        round_trip("f : Int -> Int\nf x = x + 1");
    }

    #[test]
    fn rt_module_with_imports() {
        round_trip(
            "module Foo exposing (a, b)\n\
             import Std exposing (cube)\n\
             import Std.Gears as G\n\
             a = 1\n\
             b = 2",
        );
    }

    #[test]
    fn rt_type_decl() {
        round_trip("type Shape = Cube Float Float Float | Sphere Float");
    }

    #[test]
    fn rt_type_alias() {
        round_trip("type alias Output = { models : List Shape3D, bom : List BomEntry }");
    }

    #[test]
    fn rt_slider() {
        round_trip("slider main.length = 6.0 .. 80.0");
    }

    #[test]
    fn rt_pipe_chain() {
        round_trip("f = cube 10 10 length |> translate3d origin dest");
    }

    #[test]
    fn rt_let_in() {
        round_trip("f = let x = 1 in x + 2");
    }

    #[test]
    fn rt_sketch_block() {
        let s = round_trip(
            "sk = sketch\n    var x1 = 0.0\n    let y1 = 3.0\n    poly1 = polygon (segments [p2 x1 y1, p2 4.0 y1, p2 x1 7.0])\n    in { poly1 = poly1 }\nend",
        );
        assert!(s.contains("sketch"), "{s}");
        assert!(s.contains("var x1 = 0.0"), "{s}");
        assert!(s.contains("let y1 = 3.0"), "{s}");
        assert!(s.contains("end"), "{s}");
    }

    #[test]
    fn rt_sketch_nested_in_let() {
        round_trip(
            "f =\n    let\n        sk = sketch\n                var x = 1.0\n                p = p2 x x\n            in p\n            end\n    in\n    sk",
        );
    }

    #[test]
    fn rt_if() {
        round_trip("f = if x then a else b");
    }

    #[test]
    fn rt_case() {
        round_trip("f s = case s of\n    Cube x y z -> x\n    Sphere r -> r");
    }

    #[test]
    fn rt_lambda() {
        round_trip("f = \\x y -> x + y");
    }

    #[test]
    fn rt_record_literal() {
        round_trip("f = { models = [1, 2], bom = [] }");
    }

    #[test]
    fn rt_record_update() {
        round_trip("f = { r | x = 1, y = 2 }");
    }

    #[test]
    fn rt_field_access() {
        round_trip("f = r.field.nested");
    }

    #[test]
    fn rt_cons_pattern() {
        round_trip("f xs = case xs of\n    x :: rest -> x\n    [] -> 0");
    }

    #[test]
    fn rt_precedence_keeps_meaning() {
        // `1 + 2 * 3` の精度が保たれる (出力でも `1 + 2 * 3` となる)
        let s = round_trip("f = 1 + 2 * 3");
        assert!(s.contains("1 + 2 * 3"));

        // `(1 + 2) * 3` は括弧が必要
        let s = round_trip("f = (1 + 2) * 3");
        assert!(s.contains("(1 + 2) * 3"));
    }

    #[test]
    fn rt_bolt_example() {
        let src = "\
module Bolt exposing (main)

import Std exposing (Output)

main : Float -> Output
main length =
    let bolt = cube 10 10 length in
    { models = [bolt], bom = [], controls = [] }

slider main.length = 6.0 .. 80.0
";
        round_trip(src);
    }

    #[test]
    fn rt_string_escape() {
        round_trip(r#"f = "hello\n\tworld""#);
    }

    #[test]
    fn rt_negate() {
        round_trip("f = -x + 1");
    }

    #[test]
    fn rt_qualified_var() {
        round_trip("f = Std.cube 10 10 10");
    }
}
