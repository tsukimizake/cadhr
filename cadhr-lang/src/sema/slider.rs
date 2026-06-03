//! `slider name = expr` から GUI スライダー仕様 (`SliderDecl`) を抽出する。
//!
//! 右辺は **コンパイル時定数式** である必要がある (Plan §3.5):
//! - リテラル `1..10` / `6.0..80.0`
//! - 別 `Range` 定数の参照
//! - `intersect` などの builtin 集合演算
//!
//! `main` の引数値や他の `slider` の値に依存する右辺はエラー。

use crate::diagnostic::Diagnostic;
use crate::runtime::builtin::registry as runtime_registry;
use crate::runtime::eval::Evaluator;
use crate::runtime::value::{Env, Value};
use crate::syntax::ast::*;
use std::rc::Rc;

/// 解決済み slider 宣言。GUI 側はこの形で受け取る。
#[derive(Clone, Debug, PartialEq)]
pub struct SliderDecl {
    pub name: String,
    pub lo: f64,
    pub hi: f64,
    pub elem_ty: ElemTy,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ElemTy {
    Int,
    Float,
}

/// モジュール全体から slider 宣言を抽出する。
/// 評価時に main の引数 / 他の slider 値に依存していた場合はエラー。
pub fn extract_sliders(module: &Module) -> (Vec<SliderDecl>, Vec<Diagnostic>) {
    let mut diag = Vec::new();
    let mut sliders = Vec::new();

    // top-level の Range 定数を集める (slider rhs で参照可能)
    let mut const_env = Env::new();
    let reg = runtime_registry();
    let ev = Evaluator::new(&reg);

    // builtin の Value を環境に追加
    for (name, b) in &reg.by_name {
        const_env = const_env.extend(
            name,
            Value::Builtin {
                name: name.to_string(),
                arity: b.arity,
                args: Vec::new(),
            },
        );
    }
    // 引数なしの値 decl のみ評価して環境に追加 (引数を持つ関数定義は除外)。
    for decl in &module.decls {
        if let Decl::Value(v) = decl {
            if v.params.is_empty() {
                match ev.eval_expr(&v.body, &Rc::new(const_env.clone())) {
                    Ok(value) => {
                        const_env = const_env.extend(&v.name, value);
                    }
                    Err(_) => {
                        // 動的なものは無視 (slider rhs では使えない)
                    }
                }
            }
        }
    }

    // 各 slider decl を評価
    for decl in &module.decls {
        if let Decl::Slider(s) = decl {
            match ev.eval_expr(&s.body, &Rc::new(const_env.clone())) {
                Ok(Value::Range { lo, hi, is_int }) => sliders.push(SliderDecl {
                    name: s.name.clone(),
                    lo,
                    hi,
                    elem_ty: if is_int { ElemTy::Int } else { ElemTy::Float },
                }),
                Ok(other) => diag.push(Diagnostic::SliderNotRange {
                    span: s.span,
                    name: s.name.clone(),
                    got: format!("{other}"),
                }),
                Err(e) => {
                    diag.push(Diagnostic::SliderNotConst {
                        span: s.span,
                        name: s.name.clone(),
                        related: vec![crate::diagnostic::RelatedInfo {
                            span: e.span(),
                            message: e.message(),
                        }],
                    });
                }
            }
        }
    }

    (sliders, diag)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::parse::parse;

    fn extract(src: &str) -> (Vec<SliderDecl>, Vec<Diagnostic>) {
        let m = parse(src).unwrap();
        extract_sliders(&m)
    }

    #[test]
    fn slider_literal_range() {
        let (s, diag) = extract(
            "main length = length\n\
             slider length = 6.0 .. 80.0",
        );
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].name, "length");
        assert_eq!(s[0].lo, 6.0);
        assert_eq!(s[0].hi, 80.0);
        assert_eq!(s[0].elem_ty, ElemTy::Float);
    }

    #[test]
    fn slider_int_range() {
        let (s, diag) = extract(
            "main n = n\n\
             slider n = 1 .. 5",
        );
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert_eq!(s[0].elem_ty, ElemTy::Int);
        assert_eq!(s[0].lo, 1.0);
        assert_eq!(s[0].hi, 5.0);
    }

    #[test]
    fn slider_via_constant() {
        let (s, diag) = extract(
            "boltLength = 1.0 .. 100.0\n\
             main length = length\n\
             slider length = boltLength",
        );
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert_eq!(s[0].lo, 1.0);
        assert_eq!(s[0].hi, 100.0);
    }

    #[test]
    fn slider_via_intersect() {
        let (s, diag) = extract(
            "main length = length\n\
             slider length = intersect (1.0 .. 100.0) (5.0 .. 50.0)",
        );
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert_eq!(s[0].lo, 5.0);
        assert_eq!(s[0].hi, 50.0);
    }

    #[test]
    fn slider_depends_on_main_arg_is_error() {
        let (_s, diag) = extract(
            "main length = length\n\
             slider length = 0.0 .. length",
        );
        assert!(!diag.is_empty(), "動的依存はエラーであるべき");
    }
}
