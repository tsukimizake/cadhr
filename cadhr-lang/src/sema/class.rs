//! 型クラスの登録と判定。Haskell の type class の極小版で、
//! single-parameter constraint (`Class a`) のみを扱う。
//!
//! - インスタンスは「ヘッド型コンストラクタ名 + 引数の制約」のリストで持つ。
//!   例: `Eq Int`、`Eq (List a) where Eq a`。
//! - 未解決のまま残った制約はエラーにする (Haskell の `default` のような暗黙
//!   フォールバックは行わない)。曖昧な場合は呼び出し側で型注釈を強制する。
//!
//! 将来 multi-parameter / fundeps が要るときは `Constraint` に複数の type を
//! 持たせる方向で拡張する。

use crate::sema::ty::Type;
use std::collections::HashMap;

/// 制約 `ClassName ty`。`ty` は単一化対象なので `Type` を素直に保持する。
#[derive(Clone, Debug, PartialEq)]
pub struct Constraint {
    pub class_name: String,
    pub ty: Type,
}

/// 1 つの class のメタ情報。
#[derive(Clone, Debug)]
pub struct ClassInfo {
    /// インスタンスのヘッド。`Type::Con("Int", _)` のような具体型コンストラクタを
    /// パターンに使う。`args_constraints` は head の引数に課す制約 (例: `Eq (List a)`
    /// は head=List で args_constraints=[Eq a])。
    pub instances: Vec<Instance>,
}

#[derive(Clone, Debug)]
pub struct Instance {
    /// head の型コンストラクタ名 ("Int", "Float", "List", "Range", …)。
    /// 関数型・record・型変数はマッチしない。
    pub head: String,
    /// head の引数 arity (List a なら 1、Dict k v なら 2)。-1 は arity 不問。
    pub arity: Option<usize>,
    /// head の各引数に課す制約 (それぞれ class 名)。
    /// 例: `Eq (List a)` なら `vec![vec!["Eq".into()]]`。
    pub arg_constraints: Vec<Vec<String>>,
}

impl Instance {
    pub fn simple(head: &str) -> Self {
        Self {
            head: head.to_string(),
            arity: Some(0),
            arg_constraints: Vec::new(),
        }
    }

    pub fn parametric(head: &str, arg_constraints: Vec<Vec<String>>) -> Self {
        Self {
            head: head.to_string(),
            arity: Some(arg_constraints.len()),
            arg_constraints,
        }
    }
}

/// インスタンス判定の結果。
#[derive(Clone, Debug)]
pub enum ClassCheck {
    /// マッチした (派生先の制約も合わせて)。
    Yes { derived: Vec<Constraint> },
    /// マッチしない。
    No,
    /// 型が未解決 (型変数のまま) で判定不能。
    Unknown,
}

/// class の登録簿。`standard()` で言語組み込み class を作る。
#[derive(Clone, Debug, Default)]
pub struct ClassRegistry {
    classes: HashMap<String, ClassInfo>,
}

impl ClassRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register(&mut self, name: &str, info: ClassInfo) {
        self.classes.insert(name.to_string(), info);
    }

    pub fn get(&self, name: &str) -> Option<&ClassInfo> {
        self.classes.get(name)
    }

    pub fn contains(&self, name: &str) -> bool {
        self.classes.contains_key(name)
    }

    /// 制約 `class_name ty` をインスタンス集合と突き合わせる。
    pub fn check(&self, class_name: &str, ty: &Type) -> ClassCheck {
        let Some(info) = self.classes.get(class_name) else {
            return ClassCheck::No;
        };
        match ty {
            Type::Var(_) => ClassCheck::Unknown,
            Type::Con(head, args) => {
                for inst in &info.instances {
                    if inst.head != *head {
                        continue;
                    }
                    if let Some(a) = inst.arity {
                        if a != args.len() {
                            continue;
                        }
                    }
                    let mut derived = Vec::new();
                    for (i, arg_ty) in args.iter().enumerate() {
                        if let Some(constraints) = inst.arg_constraints.get(i) {
                            for cn in constraints {
                                derived.push(Constraint {
                                    class_name: cn.clone(),
                                    ty: arg_ty.clone(),
                                });
                            }
                        }
                    }
                    return ClassCheck::Yes { derived };
                }
                ClassCheck::No
            }
            Type::Arrow(_, _) | Type::Record(_) => ClassCheck::No,
        }
    }

    /// 標準の class セットを登録した registry を返す。
    /// - `Num`: Int, Float
    /// - `Ord`: Int, Float, String, Bool
    /// - `Eq`:  Int, Float, String, Bool, List a (Eq a 派生), Range a (Eq a 派生)
    pub fn standard() -> Self {
        let mut r = Self::new();
        r.register(
            "Num",
            ClassInfo {
                instances: vec![Instance::simple("Int"), Instance::simple("Float")],
            },
        );
        r.register(
            "Ord",
            ClassInfo {
                instances: vec![
                    Instance::simple("Int"),
                    Instance::simple("Float"),
                    Instance::simple("String"),
                    Instance::simple("Bool"),
                ],
            },
        );
        r.register(
            "Eq",
            ClassInfo {
                instances: vec![
                    Instance::simple("Int"),
                    Instance::simple("Float"),
                    Instance::simple("String"),
                    Instance::simple("Bool"),
                    Instance::parametric("List", vec![vec!["Eq".into()]]),
                    Instance::parametric("Range", vec![vec!["Eq".into()]]),
                ],
            },
        );
        r
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn num_int_is_instance() {
        let r = ClassRegistry::standard();
        let res = r.check("Num", &Type::con("Int"));
        assert!(matches!(res, ClassCheck::Yes { .. }));
    }

    #[test]
    fn num_string_is_not_instance() {
        let r = ClassRegistry::standard();
        let res = r.check("Num", &Type::con("String"));
        assert!(matches!(res, ClassCheck::No));
    }

    #[test]
    fn num_var_is_unknown() {
        let r = ClassRegistry::standard();
        let res = r.check("Num", &Type::Var(crate::sema::ty::TyVar(0)));
        assert!(matches!(res, ClassCheck::Unknown));
    }

    #[test]
    fn eq_list_int_derives_eq_int() {
        let r = ClassRegistry::standard();
        let ty = Type::app("List", vec![Type::con("Int")]);
        match r.check("Eq", &ty) {
            ClassCheck::Yes { derived } => {
                assert_eq!(derived.len(), 1);
                assert_eq!(derived[0].class_name, "Eq");
                assert_eq!(derived[0].ty, Type::con("Int"));
            }
            other => panic!("expected Yes, got {other:?}"),
        }
    }
}
