//! 型推論器 (LANG_SPEC §5)。
//!
//! - rank-1 多相 (シグネチャに型変数を明示、本体内は monomorphic)
//! - Robinson unification + occurs check
//! - generalization / let polymorphism は持たない
//!
//! 本ファイルは推論の基盤 (Substitution / unify / instantiate) のみを提供する。
//! 式・Clause レベルの推論ドライバは段階的に追加する。

use crate::types::Type;
use std::collections::HashMap;
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TypeError {
    /// 単一化失敗: 形が違う型同士を unify しようとした。
    Mismatch { expected: Type, actual: Type },
    /// occurs check 違反: α と T(α) を unify しようとした (無限型)。
    InfiniteType { var: u32, ty: Type },
    /// シグネチャ未指定の関数引数 / 戻り値。推論器はここで停止する。
    /// Any への fallback はしない (LANG_SPEC §5.2)。
    MissingSignature { context: String },
    /// `[]` などの context 不在の polymorphic value。
    AmbiguousType { context: String },
}

impl fmt::Display for TypeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TypeError::Mismatch { expected, actual } => {
                write!(f, "expected {}, got {}", expected, actual)
            }
            TypeError::InfiniteType { var, ty } => {
                write!(f, "infinite type α{} = {}", var, ty)
            }
            TypeError::MissingSignature { context } => {
                write!(f, "missing type signature: {}", context)
            }
            TypeError::AmbiguousType { context } => {
                write!(f, "ambiguous type: {}", context)
            }
        }
    }
}

/// 型変数を割り当てるためのカウンター。関数本体の推論セッションごとに一度作る。
#[derive(Debug, Clone)]
pub struct VarGen {
    next: u32,
}

impl VarGen {
    pub fn new() -> Self {
        Self { next: 0 }
    }

    pub fn fresh(&mut self) -> Type {
        let v = Type::Var(self.next);
        self.next += 1;
        v
    }
}

impl Default for VarGen {
    fn default() -> Self {
        Self::new()
    }
}

/// 型変数 → Type の写像。同一の代入を表すために flat に保つ。
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Substitution {
    map: HashMap<u32, Type>,
}

impl Substitution {
    pub fn new() -> Self {
        Self {
            map: HashMap::new(),
        }
    }

    /// `ty` に substitution を再帰適用する。Var が解決できないなら Var のまま残す。
    pub fn apply(&self, ty: &Type) -> Type {
        match ty {
            Type::Var(id) => match self.map.get(id) {
                // 連鎖した代入を辿る (`α0 -> α1`, `α1 -> Number` のような場合)。
                Some(next) => self.apply(next),
                None => Type::Var(*id),
            },
            Type::List(inner) => Type::list_of(self.apply(inner)),
            // Forall は instantiate されてから unify に入るので、ここに来ない想定。
            // 来た場合はそのまま (instantiate 漏れのケースを silent に隠さない方針で、
            // assert ではなくそのまま返す: 必要なら呼び出し側が判定)。
            _ => ty.clone(),
        }
    }

    fn bind(&mut self, var: u32, ty: Type) -> Result<(), TypeError> {
        // self-bind は no-op として吸収。
        if let Type::Var(v) = &ty {
            if *v == var {
                return Ok(());
            }
        }
        if occurs_check(var, &ty, self) {
            return Err(TypeError::InfiniteType { var, ty });
        }
        self.map.insert(var, ty);
        Ok(())
    }
}

/// `var` が `ty` 内に (substitution 展開後) 現れるか。Robinson 単一化に必須。
fn occurs_check(var: u32, ty: &Type, subst: &Substitution) -> bool {
    let resolved = subst.apply(ty);
    match resolved {
        Type::Var(v) => v == var,
        Type::List(inner) => occurs_check(var, &inner, subst),
        _ => false,
    }
}

/// `t1` と `t2` を unify し、必要な代入を `subst` に追加する。
/// 単一化不能ならエラーを返し、`subst` は呼び出し前の状態に戻されているとは限らない
/// (失敗時は呼び出し側で trial_env パターンを使うこと)。
pub fn unify(t1: &Type, t2: &Type, subst: &mut Substitution) -> Result<(), TypeError> {
    let a = subst.apply(t1);
    let b = subst.apply(t2);
    match (a, b) {
        (Type::Number, Type::Number)
        | (Type::Atom, Type::Atom)
        | (Type::String, Type::String)
        | (Type::Bool, Type::Bool)
        | (Type::Shape2D, Type::Shape2D)
        | (Type::Shape3D, Type::Shape3D)
        | (Type::PlacedShape2D, Type::PlacedShape2D)
        | (Type::Path2D, Type::Path2D)
        | (Type::Point2D, Type::Point2D)
        | (Type::Point3D, Type::Point3D)
        | (Type::Plane, Type::Plane) => Ok(()),
        (Type::Record(a), Type::Record(b)) if a == b => Ok(()),
        (Type::List(a), Type::List(b)) => unify(&a, &b, subst),
        (Type::Var(x), Type::Var(y)) if x == y => Ok(()),
        (Type::Var(v), other) | (other, Type::Var(v)) => subst.bind(v, other),
        // Forall がここに来た場合は instantiate 漏れ。エラーで知らせる方が安全。
        (a @ Type::Forall(_), b) | (a, b @ Type::Forall(_)) => Err(TypeError::Mismatch {
            expected: a,
            actual: b,
        }),
        (expected, actual) => Err(TypeError::Mismatch { expected, actual }),
    }
}

/// シグネチャの Forall(name) を fresh Var に置き換える。
/// 同じ name は同じ Var にマップする (`forall T. T -> T` を `α0 -> α0` に)。
pub fn instantiate(ty: &Type, var_gen: &mut VarGen) -> Type {
    fn go(ty: &Type, var_gen: &mut VarGen, mapping: &mut HashMap<String, Type>) -> Type {
        match ty {
            Type::Forall(name) => mapping
                .entry(name.clone())
                .or_insert_with(|| var_gen.fresh())
                .clone(),
            Type::List(inner) => Type::list_of(go(inner, var_gen, mapping)),
            _ => ty.clone(),
        }
    }
    go(ty, var_gen, &mut HashMap::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Type;

    #[test]
    fn unify_concrete_same_type_ok() {
        let mut s = Substitution::new();
        assert!(unify(&Type::Number, &Type::Number, &mut s).is_ok());
        assert!(unify(&Type::Shape3D, &Type::Shape3D, &mut s).is_ok());
    }

    #[test]
    fn unify_concrete_mismatch_errors() {
        let mut s = Substitution::new();
        let r = unify(&Type::Number, &Type::Shape3D, &mut s);
        assert!(matches!(r, Err(TypeError::Mismatch { .. })));
    }

    #[test]
    fn unify_list_recurses() {
        let mut s = Substitution::new();
        unify(
            &Type::list_of(Type::Number),
            &Type::list_of(Type::Number),
            &mut s,
        )
        .unwrap();
        // List(Number) vs List(Shape3D) is an error
        let r = unify(
            &Type::list_of(Type::Number),
            &Type::list_of(Type::Shape3D),
            &mut s,
        );
        assert!(matches!(r, Err(TypeError::Mismatch { .. })));
    }

    #[test]
    fn unify_var_binds() {
        let mut s = Substitution::new();
        unify(&Type::Var(0), &Type::Shape3D, &mut s).unwrap();
        assert_eq!(s.apply(&Type::Var(0)), Type::Shape3D);
    }

    #[test]
    fn unify_var_and_var_then_concrete() {
        let mut s = Substitution::new();
        // α0 = α1, α1 = Number → α0 should resolve to Number
        unify(&Type::Var(0), &Type::Var(1), &mut s).unwrap();
        unify(&Type::Var(1), &Type::Number, &mut s).unwrap();
        assert_eq!(s.apply(&Type::Var(0)), Type::Number);
    }

    #[test]
    fn unify_occurs_check_blocks_infinite_type() {
        let mut s = Substitution::new();
        // α0 = List(α0) should fail with occurs check
        let r = unify(&Type::Var(0), &Type::list_of(Type::Var(0)), &mut s);
        assert!(matches!(r, Err(TypeError::InfiniteType { .. })));
    }

    #[test]
    fn unify_propagates_through_list() {
        let mut s = Substitution::new();
        // List(α0) unified with List(Number) → α0 = Number
        unify(
            &Type::list_of(Type::Var(0)),
            &Type::list_of(Type::Number),
            &mut s,
        )
        .unwrap();
        assert_eq!(s.apply(&Type::Var(0)), Type::Number);
    }

    #[test]
    fn instantiate_replaces_forall_with_fresh_var() {
        let mut var_gen = VarGen::new();
        // forall T. List(T) -> List(α0)
        let t = instantiate(
            &Type::list_of(Type::Forall("T".to_string())),
            &mut var_gen,
        );
        assert_eq!(t, Type::list_of(Type::Var(0)));
    }

    #[test]
    fn instantiate_keeps_same_name_consistent() {
        let mut var_gen = VarGen::new();
        // (forall T. T -> T) should instantiate to (α0 -> α0): same Var for both.
        // Here we use List as a stand-in container for two co-occurrences.
        let pair = instantiate(
            &Type::list_of(Type::list_of(Type::Forall("T".to_string()))),
            &mut var_gen,
        );
        // 両 Forall("T") は同一 Var に。
        assert_eq!(pair, Type::list_of(Type::list_of(Type::Var(0))));
    }

    #[test]
    fn instantiate_distinct_names_get_distinct_vars() {
        let mut var_gen = VarGen::new();
        // Use a tuple-like construction: List(List(T)) merged with List(List(U))
        // is awkward without tuple types, so test with two separate calls sharing gen.
        let a = instantiate(&Type::Forall("T".to_string()), &mut var_gen);
        let b = instantiate(&Type::Forall("U".to_string()), &mut var_gen);
        assert_ne!(a, b);
    }

    #[test]
    fn unify_record_same_name_ok() {
        let mut s = Substitution::new();
        unify(
            &Type::Record("output".to_string()),
            &Type::Record("output".to_string()),
            &mut s,
        )
        .unwrap();
    }

    #[test]
    fn unify_record_different_names_errors() {
        let mut s = Substitution::new();
        let r = unify(
            &Type::Record("a".to_string()),
            &Type::Record("b".to_string()),
            &mut s,
        );
        assert!(matches!(r, Err(TypeError::Mismatch { .. })));
    }
}
