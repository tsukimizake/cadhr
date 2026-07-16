//! 推論専用の可変型表現 (Algorithm J)。
//!
//! 型変数を `Rc<RefCell<VarCell>>` で共有し、単一化は破壊的にセルを書き換える。
//! これにより置換 (`Subst`) を引き回す必要が無くなり、「env / pending に subst を
//! 適用し忘れる」系統のバグが構造的に発生しなくなる。
//!
//! 保存・公開される `Type` / `Scheme` ([`crate::sema::ty`]) とは別物。推論の境界
//! (`instantiate` / `generalize`) で相互変換する。`Type` は `Send` な immutable 型
//! として残し、`CompiledProgram` に載せてスレッド境界を越える。`InferTy` は推論パス
//! 内でしか生存しない。
//!
//! ## level (Rémy のランク付き型変数)
//! 各未束縛変数は生成時の let ネスト深さ (`level`) を持つ。単一化で外側の変数と
//! 結びつくと level が下がり、generalize 時には「level > 現在深さ」の変数だけを
//! 全称化する。これで env 全走査なしに正しい let 多相が得られる。

use crate::sema::ty::{TyVar, Type};
use std::cell::RefCell;
use std::rc::Rc;

/// 共有可変な型変数セルへの参照。
pub type TyRef = Rc<RefCell<VarCell>>;

/// 型変数セルの状態。
#[derive(Debug)]
pub enum VarCell {
    /// 未束縛。`level` で generalize 可否を、`id` で表示・dedup を判定する。
    Unbound { id: u32, level: u32 },
    /// 単一化で別の型に束縛された (union-find のリンク)。
    Link(InferTy),
    /// scheme 内の量化変数プレースホルダ。`instantiate` で fresh な `Unbound` に置換。
    /// 単一化中に出会ったら instantiate 漏れ (バグ)。
    Generic(u32),
}

/// 推論中の型。`Var` だけが共有可変で、それ以外は構造そのまま。
#[derive(Clone, Debug)]
pub enum InferTy {
    Var(TyRef),
    Con(String, Vec<InferTy>),
    Arrow(Box<InferTy>, Box<InferTy>),
    Record(Vec<(String, InferTy)>),
}

impl InferTy {
    pub fn con(name: &str) -> Self {
        InferTy::Con(name.to_string(), Vec::new())
    }

    pub fn app(name: &str, args: Vec<InferTy>) -> Self {
        InferTy::Con(name.to_string(), args)
    }

    pub fn arrow(from: InferTy, to: InferTy) -> Self {
        InferTy::Arrow(Box::new(from), Box::new(to))
    }

    pub fn arrows(args: Vec<InferTy>, ret: InferTy) -> Self {
        let mut acc = ret;
        for a in args.into_iter().rev() {
            acc = InferTy::arrow(a, acc);
        }
        acc
    }
}

/// 推論中の型クラス制約。`ty` 内の変数が共有セルなので、後続の単一化を
/// 自動で追従する (置換の再適用が不要)。
#[derive(Clone, Debug)]
pub struct InferConstraint {
    pub class_name: String,
    pub ty: InferTy,
}

/// 推論中の多相スキーム。`vars` は量化された Generic セルの id。`ty` / `constraints`
/// 内の対応する `Var(Generic(id))` セルが量化点で、`instantiate` で fresh 化される。
/// 量化されていない `Var(Unbound ...)` セルは自由変数として共有されたまま残る。
#[derive(Clone, Debug)]
pub struct InferScheme {
    pub vars: Vec<u32>,
    pub constraints: Vec<InferConstraint>,
    pub ty: InferTy,
}

impl InferScheme {
    /// 全称量化なしの単型。
    pub fn mono(ty: InferTy) -> Self {
        Self {
            vars: Vec::new(),
            constraints: Vec::new(),
            ty,
        }
    }
}

/// 新しい未束縛セルを作る。
pub fn new_unbound(id: u32, level: u32) -> TyRef {
    Rc::new(RefCell::new(VarCell::Unbound { id, level }))
}

/// 新しい量化プレースホルダセルを作る。
pub fn new_generic(id: u32) -> TyRef {
    Rc::new(RefCell::new(VarCell::Generic(id)))
}

/// リンクを 1 段辿って代表元を返す (path compression つき)。
/// 返るのは `Var`(未束縛/Generic) か非変数の型。
pub fn resolve(t: &InferTy) -> InferTy {
    if let InferTy::Var(cell) = t {
        let linked = match &*cell.borrow() {
            VarCell::Link(inner) => Some(inner.clone()),
            _ => None,
        };
        if let Some(inner) = linked {
            let rep = resolve(&inner);
            *cell.borrow_mut() = VarCell::Link(rep.clone());
            return rep;
        }
    }
    t.clone()
}

/// リンクを全て解決し、構造を完全に展開した型を返す。
pub fn zonk(t: &InferTy) -> InferTy {
    match resolve(t) {
        InferTy::Con(n, args) => InferTy::Con(n, args.iter().map(zonk).collect()),
        InferTy::Arrow(f, to) => InferTy::arrow(zonk(&f), zonk(&to)),
        InferTy::Record(fs) => {
            InferTy::Record(fs.iter().map(|(n, t)| (n.clone(), zonk(t))).collect())
        }
        v => v, // Var (Unbound / Generic)
    }
}

/// `InferTy` を表示用 `Type` に焼き付ける (zonk しつつ変数を `TyVar(id)` へ)。
/// 未束縛変数も Generic も `id` をそのまま使う。
pub fn to_type_raw(t: &InferTy) -> Type {
    match resolve(t) {
        InferTy::Var(cell) => match &*cell.borrow() {
            VarCell::Unbound { id, .. } | VarCell::Generic(id) => Type::Var(TyVar(*id)),
            VarCell::Link(_) => unreachable!("resolve 済みなので Link は来ない"),
        },
        InferTy::Con(n, args) => Type::Con(n, args.iter().map(to_type_raw).collect()),
        InferTy::Arrow(f, to) => Type::arrow(to_type_raw(&f), to_type_raw(&to)),
        InferTy::Record(fs) => {
            Type::Record(fs.iter().map(|(n, t)| (n.clone(), to_type_raw(t))).collect())
        }
    }
}

/// 診断メッセージ用に整形する。
pub fn show(t: &InferTy) -> String {
    format!("{}", to_type_raw(t))
}

/// 単一化エラー。呼び出し側 (infer.rs) が span 付き `Diagnostic` に変換する。
#[derive(Clone, Debug)]
pub enum UnifyError {
    Mismatch { left: String, right: String },
    InfiniteType { ty: String },
    RecordFieldMismatch {
        left_fields: Vec<String>,
        right_fields: Vec<String>,
    },
}

/// 2 つの型を単一化する。成功すると変数セルが破壊的に書き換わる。
pub fn unify(a: &InferTy, b: &InferTy) -> Result<(), UnifyError> {
    let a = resolve(a);
    let b = resolve(b);
    match (&a, &b) {
        (InferTy::Var(ra), InferTy::Var(rb)) if Rc::ptr_eq(ra, rb) => Ok(()),
        (InferTy::Var(ra), _) => bind(ra, &b),
        (_, InferTy::Var(rb)) => bind(rb, &a),
        (InferTy::Con(n1, a1), InferTy::Con(n2, a2)) if n1 == n2 && a1.len() == a2.len() => {
            for (x, y) in a1.iter().zip(a2.iter()) {
                unify(x, y)?;
            }
            Ok(())
        }
        (InferTy::Arrow(f1, t1), InferTy::Arrow(f2, t2)) => {
            unify(f1, f2)?;
            unify(t1, t2)
        }
        (InferTy::Record(r1), InferTy::Record(r2)) => unify_records(r1, r2),
        _ => Err(UnifyError::Mismatch {
            left: show(&a),
            right: show(&b),
        }),
    }
}

fn unify_records(
    r1: &[(String, InferTy)],
    r2: &[(String, InferTy)],
) -> Result<(), UnifyError> {
    let names1: Vec<String> = r1.iter().map(|(n, _)| n.clone()).collect();
    let names2: Vec<String> = r2.iter().map(|(n, _)| n.clone()).collect();
    let mut s1 = names1.clone();
    let mut s2 = names2.clone();
    s1.sort();
    s2.sort();
    if s1 != s2 {
        return Err(UnifyError::RecordFieldMismatch {
            left_fields: names1,
            right_fields: names2,
        });
    }
    for (n1, t1) in r1 {
        let (_, t2) = r2.iter().find(|(n2, _)| n2 == n1).unwrap();
        unify(t1, t2)?;
    }
    Ok(())
}

/// 未束縛セル `cell` を型 `t` に束縛する。occurs check と level 調整を伴う。
fn bind(cell: &TyRef, t: &InferTy) -> Result<(), UnifyError> {
    let level = match &*cell.borrow() {
        VarCell::Unbound { level, .. } => *level,
        VarCell::Generic(_) => {
            panic!("単一化中に Generic 変数に出会った: instantiate 漏れの可能性");
        }
        VarCell::Link(_) => unreachable!("resolve 済み"),
    };
    occurs_adjust(cell, t, level)?;
    *cell.borrow_mut() = VarCell::Link(t.clone());
    Ok(())
}

/// `t` 内に `cell` が出現したら循環型エラー。併せて出現する未束縛変数の level を
/// `level` まで下げる (外側スコープへのエスケープを正しく扱う)。
fn occurs_adjust(cell: &TyRef, t: &InferTy, level: u32) -> Result<(), UnifyError> {
    match resolve(t) {
        InferTy::Var(r) => {
            if Rc::ptr_eq(&r, cell) {
                return Err(UnifyError::InfiniteType {
                    ty: show(&InferTy::Var(r)),
                });
            }
            let lowered = match &*r.borrow() {
                VarCell::Unbound { id, level: l } => Some(VarCell::Unbound {
                    id: *id,
                    level: (*l).min(level),
                }),
                _ => None,
            };
            if let Some(nc) = lowered {
                *r.borrow_mut() = nc;
            }
            Ok(())
        }
        InferTy::Con(_, args) => {
            for a in &args {
                occurs_adjust(cell, a, level)?;
            }
            Ok(())
        }
        InferTy::Arrow(f, to) => {
            occurs_adjust(cell, &f, level)?;
            occurs_adjust(cell, &to, level)
        }
        InferTy::Record(fs) => {
            for (_, t) in &fs {
                occurs_adjust(cell, t, level)?;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unbound(id: u32) -> InferTy {
        InferTy::Var(new_unbound(id, 1))
    }

    #[test]
    fn unify_var_with_concrete() {
        let a = unbound(0);
        unify(&a, &InferTy::con("Int")).unwrap();
        assert_eq!(show(&a), "Int");
    }

    #[test]
    fn unify_arrow() {
        let a = unbound(0);
        let b = unbound(1);
        let lhs = InferTy::arrow(a.clone(), b.clone());
        let rhs = InferTy::arrow(InferTy::con("Int"), InferTy::con("Float"));
        unify(&lhs, &rhs).unwrap();
        assert_eq!(show(&a), "Int");
        assert_eq!(show(&b), "Float");
    }

    #[test]
    fn unify_mismatch() {
        let err = unify(&InferTy::con("Int"), &InferTy::con("Float")).unwrap_err();
        assert!(matches!(err, UnifyError::Mismatch { .. }));
    }

    #[test]
    fn occurs_check() {
        let a = unbound(0);
        // a = a -> Int は循環
        let err = unify(&a, &InferTy::arrow(a.clone(), InferTy::con("Int"))).unwrap_err();
        assert!(matches!(err, UnifyError::InfiniteType { .. }));
    }

    #[test]
    fn shared_var_updates_everywhere() {
        // 同じセルを 2 箇所で共有し、片方を unify するともう片方も変わる
        let a = unbound(0);
        let pair = InferTy::Record(vec![("x".into(), a.clone()), ("y".into(), a.clone())]);
        unify(&a, &InferTy::con("Int")).unwrap();
        assert_eq!(show(&pair), "{ x : Int, y : Int }");
    }

    #[test]
    fn level_lowered_on_unify() {
        // level 5 の変数と level 2 の変数を unify すると、survivor の level は 2 に下がる
        let hi = InferTy::Var(new_unbound(0, 5));
        let lo = InferTy::Var(new_unbound(1, 2));
        unify(&hi, &lo).unwrap();
        // hi が lo にリンクし、lo の level は min(2,5)=2 のまま
        if let InferTy::Var(cell) = resolve(&lo) {
            match &*cell.borrow() {
                VarCell::Unbound { level, .. } => assert_eq!(*level, 2),
                _ => panic!("unbound のはず"),
            }
        } else {
            panic!("var のはず");
        }
    }
}
