//! 型置換と単一化 (Robinson unification + occurs check)。
//!
//! `Subst` は `TyVar → Type` の有限写像。`apply` で型に置換を適用し、
//! `unify` で 2 つの型を一致させる置換を求める。
//!
//! 単一化失敗は `UnifyError` として返す。span 付きの診断は呼び出し側 (infer.rs)
//! が `Diagnostic` に変換する。

use crate::sema::class::Constraint;
use crate::sema::ty::{TyVar, Type};
use std::collections::HashMap;

#[derive(Clone, Debug, Default)]
pub struct Subst {
    map: HashMap<TyVar, Type>,
}

impl Subst {
    pub fn empty() -> Self {
        Self {
            map: HashMap::new(),
        }
    }

    pub fn singleton(v: TyVar, t: Type) -> Self {
        let mut s = Self::empty();
        s.map.insert(v, t);
        s
    }

    pub fn insert(&mut self, v: TyVar, t: Type) {
        self.map.insert(v, t);
    }

    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }

    /// 別の置換を合成する。`s2 ∘ self`: 先に self、次に s2 を適用するのと等価。
    pub fn compose(&self, s2: &Subst) -> Subst {
        let mut out = Subst::empty();
        // self の各エントリに s2 を適用
        for (v, t) in &self.map {
            out.map.insert(*v, s2.apply(t));
        }
        // s2 のエントリは self に無いものをそのまま追加
        for (v, t) in &s2.map {
            out.map.entry(*v).or_insert_with(|| t.clone());
        }
        out
    }

    /// 型に置換を適用する。
    pub fn apply(&self, t: &Type) -> Type {
        match t {
            Type::Var(v) => match self.map.get(v) {
                Some(t2) => self.apply(t2),
                None => Type::Var(*v),
            },
            Type::Con(name, args) => {
                Type::Con(name.clone(), args.iter().map(|a| self.apply(a)).collect())
            }
            Type::Arrow(from, to) => {
                Type::Arrow(Box::new(self.apply(from)), Box::new(self.apply(to)))
            }
            Type::Record(fields) => Type::Record(
                fields
                    .iter()
                    .map(|(n, t)| (n.clone(), self.apply(t)))
                    .collect(),
            ),
        }
    }

    /// 制約 (`Class ty`) に置換を適用する。
    pub fn apply_constraint(&self, c: &Constraint) -> Constraint {
        Constraint {
            class_name: c.class_name.clone(),
            ty: self.apply(&c.ty),
        }
    }
}

/// 単一化エラー。呼び出し側が span 付き `Diagnostic` に変換する。
#[derive(Clone, Debug)]
pub enum UnifyError {
    /// 構造的不一致 (例: `Int` と `Float`、`a -> b` と `Int`)
    Mismatch { left: Type, right: Type },
    /// occurs check 失敗 (循環型)
    InfiniteType { var: TyVar, ty: Type },
    /// record field 不一致
    RecordFieldMismatch {
        left_fields: Vec<String>,
        right_fields: Vec<String>,
    },
}

/// 2 つの型を単一化する。成功すれば結合置換を返す。
pub fn unify(a: &Type, b: &Type) -> Result<Subst, UnifyError> {
    match (a, b) {
        (Type::Var(v), t) | (t, Type::Var(v)) => bind(*v, t),
        (Type::Con(n1, a1), Type::Con(n2, a2)) if n1 == n2 && a1.len() == a2.len() => {
            unify_seq(a1, a2)
        }
        (Type::Arrow(f1, t1), Type::Arrow(f2, t2)) => {
            let s1 = unify(f1, f2)?;
            let s2 = unify(&s1.apply(t1), &s1.apply(t2))?;
            Ok(s1.compose(&s2))
        }
        (Type::Record(r1), Type::Record(r2)) => {
            // field 名集合が一致することを要求 (順序は無関係)
            let names1: Vec<_> = r1.iter().map(|(n, _)| n.clone()).collect();
            let names2: Vec<_> = r2.iter().map(|(n, _)| n.clone()).collect();
            let mut sorted1 = names1.clone();
            sorted1.sort();
            let mut sorted2 = names2.clone();
            sorted2.sort();
            if sorted1 != sorted2 {
                return Err(UnifyError::RecordFieldMismatch {
                    left_fields: names1,
                    right_fields: names2,
                });
            }
            // 同名フィールド同士を unify
            let mut s = Subst::empty();
            for (n1, t1) in r1 {
                let (_, t2) = r2.iter().find(|(n2, _)| n2 == n1).unwrap();
                let s_field = unify(&s.apply(t1), &s.apply(t2))?;
                s = s.compose(&s_field);
            }
            Ok(s)
        }
        _ => Err(UnifyError::Mismatch {
            left: a.clone(),
            right: b.clone(),
        }),
    }
}

fn bind(v: TyVar, t: &Type) -> Result<Subst, UnifyError> {
    match t {
        Type::Var(v2) if *v2 == v => Ok(Subst::empty()),
        _ if t.free_vars().contains(&v) => Err(UnifyError::InfiniteType {
            var: v,
            ty: t.clone(),
        }),
        _ => Ok(Subst::singleton(v, t.clone())),
    }
}

fn unify_seq(xs: &[Type], ys: &[Type]) -> Result<Subst, UnifyError> {
    let mut s = Subst::empty();
    for (x, y) in xs.iter().zip(ys.iter()) {
        let s2 = unify(&s.apply(x), &s.apply(y))?;
        s = s.compose(&s2);
    }
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sema::ty::TyVarGen;

    #[test]
    fn unify_concrete_equal() {
        let s = unify(&Type::con("Int"), &Type::con("Int")).unwrap();
        assert!(s.map.is_empty());
    }

    #[test]
    fn unify_concrete_mismatch() {
        let err = unify(&Type::con("Int"), &Type::con("Float")).unwrap_err();
        assert!(matches!(err, UnifyError::Mismatch { .. }));
    }

    #[test]
    fn unify_var_with_concrete() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        let s = unify(&Type::Var(a), &Type::con("Int")).unwrap();
        assert_eq!(s.apply(&Type::Var(a)), Type::con("Int"));
    }

    #[test]
    fn unify_arrow() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        let b = g.fresh();
        // (a -> b) と (Int -> Float)
        let lhs = Type::arrow(Type::Var(a), Type::Var(b));
        let rhs = Type::arrow(Type::con("Int"), Type::con("Float"));
        let s = unify(&lhs, &rhs).unwrap();
        assert_eq!(s.apply(&Type::Var(a)), Type::con("Int"));
        assert_eq!(s.apply(&Type::Var(b)), Type::con("Float"));
    }

    #[test]
    fn unify_occurs_check() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        // a = a -> Int は循環型
        let err = unify(&Type::Var(a), &Type::arrow(Type::Var(a), Type::con("Int"))).unwrap_err();
        assert!(matches!(err, UnifyError::InfiniteType { .. }));
    }

    #[test]
    fn unify_app() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        // List a と List Int
        let s = unify(
            &Type::app("List", vec![Type::Var(a)]),
            &Type::app("List", vec![Type::con("Int")]),
        )
        .unwrap();
        assert_eq!(s.apply(&Type::Var(a)), Type::con("Int"));
    }

    #[test]
    fn unify_record_same_fields() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        let lhs = Type::Record(vec![
            ("x".into(), Type::Var(a)),
            ("y".into(), Type::con("Float")),
        ]);
        let rhs = Type::Record(vec![
            ("y".into(), Type::con("Float")),
            ("x".into(), Type::con("Int")),
        ]);
        let s = unify(&lhs, &rhs).unwrap();
        assert_eq!(s.apply(&Type::Var(a)), Type::con("Int"));
    }

    #[test]
    fn unify_record_field_mismatch() {
        let lhs = Type::Record(vec![("x".into(), Type::con("Int"))]);
        let rhs = Type::Record(vec![
            ("x".into(), Type::con("Int")),
            ("y".into(), Type::con("Int")),
        ]);
        let err = unify(&lhs, &rhs).unwrap_err();
        assert!(matches!(err, UnifyError::RecordFieldMismatch { .. }));
    }

    #[test]
    fn compose_preserves_apply() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        let b = g.fresh();
        // s1: a -> Int
        let s1 = Subst::singleton(a, Type::con("Int"));
        // s2: b -> a
        let s2 = Subst::singleton(b, Type::Var(a));
        let composed = s1.compose(&s2);
        // 合成後、b は a 経由で Int になる
        assert_eq!(composed.apply(&Type::Var(b)), Type::con("Int"));
    }
}
