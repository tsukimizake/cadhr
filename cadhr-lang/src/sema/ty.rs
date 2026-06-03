//! Type の内部表現。
//!
//! `Type` は構造体ベースの「具体型 + 型変数」を表す。`Scheme` は多相型
//! (`forall α β. T`) で、シグネチャ宣言や let-poly で生成される。

use std::collections::HashSet;
use std::fmt;

/// 型変数 ID。`fresh_ty_var()` で一意に生成される。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TyVar(pub u32);

/// 推論用の型 (具体型 + 型変数 + 関数型 + record + ADT)。
#[derive(Clone, Debug, PartialEq)]
pub enum Type {
    /// 型変数。単一化対象。
    Var(TyVar),
    /// 名前付き型コンストラクタ。`Int`, `Float`, `Shape3D` (引数なし)、
    /// `List a`, `Range a`, `Dict k v` (引数つき)。
    Con(String, Vec<Type>),
    /// 関数型 `from -> to`。
    Arrow(Box<Type>, Box<Type>),
    /// レコード型 `{ field : Type, ... }`。順序は宣言順で保持する。
    Record(Vec<(String, Type)>),
}

impl Type {
    pub fn con(name: &str) -> Self {
        Type::Con(name.to_string(), Vec::new())
    }

    pub fn app(name: &str, args: Vec<Type>) -> Self {
        Type::Con(name.to_string(), args)
    }

    pub fn arrow(from: Type, to: Type) -> Self {
        Type::Arrow(Box::new(from), Box::new(to))
    }

    /// 多引数の curried 関数 `a -> b -> c -> r` を一度に組む。
    pub fn arrows(args: Vec<Type>, ret: Type) -> Self {
        let mut acc = ret;
        for a in args.into_iter().rev() {
            acc = Type::arrow(a, acc);
        }
        acc
    }

    /// 出現する型変数を集める (occurs check / generalize で使う)。
    pub fn free_vars(&self) -> HashSet<TyVar> {
        let mut out = HashSet::new();
        self.collect_free_vars(&mut out);
        out
    }

    fn collect_free_vars(&self, out: &mut HashSet<TyVar>) {
        match self {
            Type::Var(v) => {
                out.insert(*v);
            }
            Type::Con(_, args) => {
                for a in args {
                    a.collect_free_vars(out);
                }
            }
            Type::Arrow(from, to) => {
                from.collect_free_vars(out);
                to.collect_free_vars(out);
            }
            Type::Record(fields) => {
                for (_, t) in fields {
                    t.collect_free_vars(out);
                }
            }
        }
    }
}

impl fmt::Display for Type {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt_ty(self, f, 0)
    }
}

fn fmt_ty(t: &Type, f: &mut fmt::Formatter<'_>, prec: u8) -> fmt::Result {
    match t {
        Type::Var(v) => write!(f, "t{}", v.0),
        Type::Con(name, args) if args.is_empty() => f.write_str(name),
        Type::Con(name, args) => {
            if prec > 1 {
                f.write_str("(")?;
            }
            f.write_str(name)?;
            for a in args {
                f.write_str(" ")?;
                fmt_ty(a, f, 2)?;
            }
            if prec > 1 {
                f.write_str(")")?;
            }
            Ok(())
        }
        Type::Arrow(from, to) => {
            if prec > 0 {
                f.write_str("(")?;
            }
            fmt_ty(from, f, 1)?;
            f.write_str(" -> ")?;
            fmt_ty(to, f, 0)?;
            if prec > 0 {
                f.write_str(")")?;
            }
            Ok(())
        }
        Type::Record(fields) => {
            f.write_str("{ ")?;
            for (i, (n, ty)) in fields.iter().enumerate() {
                if i > 0 {
                    f.write_str(", ")?;
                }
                write!(f, "{n} : ")?;
                fmt_ty(ty, f, 0)?;
            }
            f.write_str(" }")?;
            Ok(())
        }
    }
}

/// 多相型。`forall vars. ty` を表す。ユーザーシグネチャと let-poly の
/// generalization で生成される。
#[derive(Clone, Debug, PartialEq)]
pub struct Scheme {
    pub vars: Vec<TyVar>,
    pub ty: Type,
}

impl Scheme {
    /// 全称量化なしの単型を Scheme として包む。
    pub fn mono(ty: Type) -> Self {
        Self {
            vars: Vec::new(),
            ty,
        }
    }

    /// 自由変数 (= 全称量化されていない型変数)。
    pub fn free_vars(&self) -> HashSet<TyVar> {
        let bound: HashSet<TyVar> = self.vars.iter().copied().collect();
        self.ty
            .free_vars()
            .into_iter()
            .filter(|v| !bound.contains(v))
            .collect()
    }
}

impl fmt::Display for Scheme {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if !self.vars.is_empty() {
            f.write_str("forall")?;
            for v in &self.vars {
                write!(f, " t{}", v.0)?;
            }
            f.write_str(". ")?;
        }
        write!(f, "{}", self.ty)
    }
}

/// 型変数のフレッシュ生成器 (推論器ごとに 1 つ持つ)。
#[derive(Debug, Default)]
pub struct TyVarGen {
    next: u32,
}

impl TyVarGen {
    pub fn new() -> Self {
        Self { next: 0 }
    }

    pub fn fresh(&mut self) -> TyVar {
        let v = TyVar(self.next);
        self.next += 1;
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_vars_simple() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        let b = g.fresh();
        let ty = Type::arrow(Type::Var(a), Type::Var(b));
        let fv: Vec<_> = ty.free_vars().into_iter().collect();
        assert_eq!(fv.len(), 2);
        assert!(fv.contains(&a));
        assert!(fv.contains(&b));
    }

    #[test]
    fn scheme_free_vars_skips_bound() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        let b = g.fresh();
        let scheme = Scheme {
            vars: vec![a],
            ty: Type::arrow(Type::Var(a), Type::Var(b)),
        };
        let fv = scheme.free_vars();
        assert_eq!(fv.len(), 1);
        assert!(fv.contains(&b));
    }

    #[test]
    fn display_arrow() {
        let mut g = TyVarGen::new();
        let a = g.fresh();
        // Int -> a -> Shape3D
        let ty = Type::arrows(vec![Type::con("Int"), Type::Var(a)], Type::con("Shape3D"));
        assert_eq!(format!("{ty}"), "Int -> t0 -> Shape3D");
    }

    #[test]
    fn display_app() {
        let ty = Type::app("List", vec![Type::con("Int")]);
        assert_eq!(format!("{ty}"), "List Int");

        let ty = Type::arrow(Type::app("List", vec![Type::con("Int")]), Type::con("Int"));
        assert_eq!(format!("{ty}"), "List Int -> Int");
    }
}
