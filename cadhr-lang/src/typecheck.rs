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

/// 同じシグネチャ内の複数の型 (params + return) を共有 mapping で instantiate する。
/// `length(Xs: List(T)) -> Number` のような signature で、両側の T が同一 Var に
/// 束縛される必要があるため必要。
pub fn instantiate_signature(
    params: &[Type],
    return_ty: &Type,
    var_gen: &mut VarGen,
) -> (Vec<Type>, Type) {
    let mut mapping = HashMap::new();
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
    let inst_params = params
        .iter()
        .map(|p| go(p, var_gen, &mut mapping))
        .collect();
    let inst_return = go(return_ty, var_gen, &mut mapping);
    (inst_params, inst_return)
}

// ============================================================
// Builtin signature lookup (registry 経由)
// ============================================================

/// `BuiltinRegistry` から推論器が期待する形式へ展開する。
/// (Task #6 完了後はこの関数を経由せず Registry を直接渡してもよい。)
pub fn signatures_from_registry(
    registry: &crate::builtins::BuiltinRegistry,
) -> HashMap<(String, usize), (Vec<Type>, Type)> {
    registry.signatures()
}

// ============================================================
// 式レベル推論 (TypeEnv + infer_term)
// ============================================================

/// 関数本体内のローカル変数 → 型 (まだ未確定なら Var(α))。
/// 関数の入口で signature の引数型を入れ、本体評価中に新規 Var が登場したら fresh α を追加する。
#[derive(Debug, Clone, Default)]
pub struct TypeEnv {
    bindings: HashMap<String, Type>,
}

impl TypeEnv {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, name: impl Into<String>, ty: Type) {
        self.bindings.insert(name.into(), ty);
    }

    pub fn get_or_fresh(&mut self, name: &str, var_gen: &mut VarGen) -> Type {
        if let Some(t) = self.bindings.get(name) {
            return t.clone();
        }
        let fresh = var_gen.fresh();
        self.bindings.insert(name.to_string(), fresh.clone());
        fresh
    }
}

/// ユーザ定義述語のシグネチャテーブル。`(functor name, arity) → (params, return_ty)`。
/// 旧 cadhr-lang はシグネチャを書く構文がなかったため、新仕様で `head -> Type` が
/// 付いた Clause だけがここに登録される。signature 未指定の predicate は本テーブルに
/// 入らず、その呼び出しは `infer_term` で `MissingSignature` エラーになる。
#[derive(Debug, Clone, Default)]
pub struct UserSignatures {
    sigs: HashMap<(String, usize), (Vec<Type>, Type)>,
}

impl UserSignatures {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, name: String, params: Vec<Type>, return_ty: Type) {
        self.sigs.insert((name, params.len()), (params, return_ty));
    }

    pub fn lookup(&self, name: &str, arity: usize) -> Option<&(Vec<Type>, Type)> {
        self.sigs.get(&(name.to_string(), arity))
    }
}

/// database から `head -> Type` の付いた Clause のシグネチャを抽出する。
/// 各 head 引数の型を以下の優先順で決める:
///   1. `Var: Type` 注釈 → その Type
///   2. range Var (`0<X<100`) → Number
///   3. Struct パターン (`p3(X, Y, Z)` 等) で functor が builtin として登録されていれば
///      その return_ty を採用 (パターンマッチによる destructure を許す)
///   4. それ以外 → シグネチャ不完全とみなしスキップ
pub fn collect_user_signatures<S>(
    clauses: &[crate::parse::Clause<S>],
    builtins: &crate::builtins::BuiltinRegistry,
) -> UserSignatures {
    use crate::parse::{Clause, Term};
    let mut us = UserSignatures::new();
    for clause in clauses {
        let (head, return_type) = match clause {
            Clause::Fact { head, return_type } => (head, return_type),
            Clause::Rule { head, return_type, .. } => (head, return_type),
            _ => continue,
        };
        let Some(return_ty) = return_type else { continue };
        let Term::Struct { functor, args, .. } = head else { continue };
        let mut params = Vec::with_capacity(args.len());
        let mut complete = true;
        for arg in args {
            match arg {
                Term::Var { type_annotation: Some(t), .. } => params.push(t.clone()),
                Term::Var { min, max, .. } if min.is_some() || max.is_some() => {
                    params.push(Type::Number);
                }
                Term::Struct { functor: f, args: pat_args, .. } => {
                    // Struct パターンの型は builtin の return_ty で取れる。
                    // 同名 arity overload があっても return_ty は通常同一なので
                    // 最初の一致を採用。
                    if let Some(b) = builtins.lookup(f, pat_args.len()) {
                        params.push(b.return_ty.clone());
                    } else {
                        complete = false;
                        break;
                    }
                }
                _ => {
                    complete = false;
                    break;
                }
            }
        }
        if complete {
            us.insert(functor.clone(), params, return_ty.clone());
        }
    }
    us
}

/// record 宣言の typecheck 用 view。`#record name { f: T = default }` の `(field, T)` だけを
/// 抽出して保持する。実体は `crate::parse::Clause::RecordDecl` から `RecordDeclTable::from_clauses`
/// で構築する。
#[derive(Debug, Clone, Default)]
pub struct RecordDeclTable {
    decls: HashMap<String, Vec<(String, Type)>>,
}

impl RecordDeclTable {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, name: String, fields: Vec<(String, Type)>) {
        self.decls.insert(name, fields);
    }

    pub fn lookup(&self, name: &str) -> Option<&[(String, Type)]> {
        self.decls.get(name).map(|v| v.as_slice())
    }

    /// `RecordField.ty` が `None` の field はそのフィールドの型注釈が省略されていた
    /// (default 値から推論される予定の) ケース。新仕様 §4.4 では推論は型推論器の役割。
    /// 現状の typecheck は型注釈付き field のみ扱うため、注釈なしフィールドは
    /// `Forall("rec_field")` として登録し、lookup 時に呼び出し側 var_gen で fresh
    /// Var に instantiate する (clause-local var_gen と ID 衝突しないよう Forall 名で
    /// 名前空間を区切る)。型を pin したい場合はソース側で `field: T = default` と書く。
    pub fn from_clauses<S>(clauses: &[crate::parse::Clause<S>]) -> Self {
        use crate::parse::Clause;
        let mut t = Self::new();
        for c in clauses {
            if let Clause::RecordDecl { name, fields, .. } = c {
                let fs = fields
                    .iter()
                    .map(|f| {
                        let ty = f.ty.clone().unwrap_or_else(|| {
                            Type::Forall(format!("{}_{}", name, f.name))
                        });
                        (f.name.clone(), ty)
                    })
                    .collect();
                t.insert(name.clone(), fs);
            }
        }
        t
    }
}

pub struct InferCtx<'a> {
    pub env: TypeEnv,
    pub subst: &'a mut Substitution,
    pub var_gen: &'a mut VarGen,
    pub builtins: &'a crate::builtins::BuiltinRegistry,
    pub user_sigs: &'a UserSignatures,
    pub records: &'a RecordDeclTable,
}

/// Term を推論し、その型を返す。Substitution は副作用で更新される。
pub fn infer_term<S>(
    term: &crate::parse::Term<S>,
    ctx: &mut InferCtx<'_>,
) -> Result<Type, TypeError> {
    use crate::parse::{ArithOp, Term};
    match term {
        Term::Number { .. } => Ok(Type::Number),
        Term::StringLit { .. } => Ok(Type::String),
        Term::Var { name, type_annotation, .. } => {
            let t = ctx.env.get_or_fresh(name, ctx.var_gen);
            if let Some(annot) = type_annotation {
                // X: T 注釈があれば unify でローカル型と整合させる。
                unify(&t, annot, ctx.subst)?;
            }
            Ok(t)
        }
        Term::List { items, tail } => {
            // 全要素同じ T、結果は List(T)
            let elem_ty = ctx.var_gen.fresh();
            for item in items {
                let item_ty = infer_term(item, ctx)?;
                unify(&elem_ty, &item_ty, ctx.subst)?;
            }
            if let Some(t) = tail {
                let tail_ty = infer_term(t, ctx)?;
                unify(&Type::list_of(elem_ty.clone()), &tail_ty, ctx.subst)?;
            }
            Ok(Type::list_of(elem_ty))
        }
        Term::InfixExpr { op, left, right } => {
            // 算術: Number * Number = Number
            // CSG: Shape + Shape = Shape (Add=union, Sub=difference, Mul=intersection)
            // `/` は Number 専用。
            let l = infer_term(left, ctx)?;
            let r = infer_term(right, ctx)?;
            if matches!(op, ArithOp::Div) {
                unify(&l, &Type::Number, ctx.subst)?;
                unify(&r, &Type::Number, ctx.subst)?;
                Ok(Type::Number)
            } else {
                // 両辺を一致させる: Number-Number / Shape3D-Shape3D / Shape2D-Shape2D の
                // いずれでも通る。返り値は左辺と同じ型 (= resolved 両辺)。
                unify(&l, &r, ctx.subst)?;
                Ok(ctx.subst.apply(&l))
            }
        }
        Term::Struct { functor, args, .. } => {
            // builtin → user sig の順で探す。両方に無ければ MissingSignature。
            // user predicate は `head(...) -> Type :- body.` 形式でシグネチャを
            // 明示することが求められる (LANG_SPEC §5.2)。
            let (params_template, ret_template) =
                if let Some(b) = ctx.builtins.lookup(functor, args.len()) {
                    (b.params.clone(), b.return_ty.clone())
                } else if let Some((p, r)) = ctx.user_sigs.lookup(functor, args.len()) {
                    (p.clone(), r.clone())
                } else {
                    return Err(TypeError::MissingSignature {
                        context: format!("{}/{}", functor, args.len()),
                    });
                };
            let (params, ret_ty) =
                instantiate_signature(&params_template, &ret_template, ctx.var_gen);
            for (arg, expected) in args.iter().zip(params.iter()) {
                let actual = infer_term(arg, ctx)?;
                unify(expected, &actual, ctx.subst)?;
            }
            Ok(ret_ty)
        }
        Term::Eq { left, right } => {
            let l = infer_term(left, ctx)?;
            let r = infer_term(right, ctx)?;
            unify(&l, &r, ctx.subst)?;
            // Eq は goal なので戻り値型は意味的に不要。型システムでは Bool を返す。
            Ok(Type::Bool)
        }
        Term::FieldAccess { record, field, .. } => {
            // record の型を推論 → substitute で具体化 → Type::Record(name) を期待。
            // Var (未束縛 α) なら型が不明なため AmbiguousType。
            let rec_ty = infer_term(record, ctx)?;
            let resolved = ctx.subst.apply(&rec_ty);
            let record_name = match &resolved {
                Type::Record(n) => n.clone(),
                _ => {
                    return Err(TypeError::AmbiguousType {
                        context: format!(
                            "field access .{} on non-record value (resolved to {})",
                            field, resolved
                        ),
                    });
                }
            };
            let decl =
                ctx.records
                    .lookup(&record_name)
                    .ok_or_else(|| TypeError::MissingSignature {
                        context: format!("record decl for '{}'", record_name),
                    })?
                    .to_vec();
            let field_ty_raw = decl
                .iter()
                .find(|(n, _)| n == field)
                .map(|(_, t)| t.clone())
                .ok_or_else(|| TypeError::MissingSignature {
                    context: format!("field '{}' on record '{}'", field, record_name),
                })?;
            // decl に Forall (注釈なし field) が含まれる場合は呼び出し側 var_gen で
            // fresh Var に instantiate する。これにより RecordDeclTable の α が
            // clause-local var_gen の α と ID 衝突するのを防ぐ。
            Ok(instantiate(&field_ty_raw, ctx.var_gen))
        }
        Term::Record { name, fields, .. } => {
            let decl =
                ctx.records
                    .lookup(name)
                    .ok_or_else(|| TypeError::MissingSignature {
                        context: format!("record decl for '{}'", name),
                    })?
                    .to_vec();
            // 全 field 型を 1 つの mapping で instantiate (同一 record 内で同じ
            // Forall 名は同じ Var を共有)。decl 順に展開してから provided fields の
            // 値型と unify する。
            let decl_types: Vec<Type> = decl.iter().map(|(_, t)| t.clone()).collect();
            let (instantiated, _ret) = instantiate_signature(
                &decl_types,
                &Type::Record(name.clone()),
                ctx.var_gen,
            );
            for (provided_name, value) in fields {
                let idx = decl
                    .iter()
                    .position(|(n, _)| n == provided_name)
                    .ok_or_else(|| TypeError::MissingSignature {
                        context: format!(
                            "field '{}' is not declared on record '{}'",
                            provided_name, name
                        ),
                    })?;
                let expected = &instantiated[idx];
                let actual = infer_term(value, ctx)?;
                unify(expected, &actual, ctx.subst)?;
            }
            Ok(Type::Record(name.clone()))
        }
    }
}

// ============================================================
// Clause レベル: 関数定義をシグネチャと照合する
// ============================================================

/// `Clause::Rule` / `Clause::Fact` を型検査する。
///
/// 1. head 引数の `type_annotation` を TypeEnv に追加。range のみで型注釈なしの
///    引数は `Number` 型として扱う (range が型を兼ねる)。
/// 2. body の各 goal を `infer_term` で推論。Eq は両辺 unify、それ以外は構造の整合のみ。
/// 3. 推論できなかった α が残っていてもこの関数では OK (型変数は Var で良い)。
///    本体内で「推論不能」が確定したエラー (`MissingSignature` 等) は伝播する。
///
/// 返り値の型: 現状は `()`。`return_type` フィールドの活用は呼び出し規約が
/// 確定してから (Task #8 で interpreter と統合する際) 実装する。LANG_SPEC §7 の
/// 「返り値は head 引数として明示的に渡す」規約に合わせて、最後の head 引数を
/// return_type と unify する方向で詰める予定。
pub fn infer_clause<S: Clone>(
    clause: &crate::parse::Clause<S>,
    builtins: &crate::builtins::BuiltinRegistry,
    user_sigs: &UserSignatures,
    records: &RecordDeclTable,
) -> Result<(), TypeError> {
    use crate::parse::{Clause, Term};
    let (head, body) = match clause {
        Clause::Fact { head, .. } => (head, [].as_slice()),
        Clause::Rule { head, body, .. } => (head, body.as_slice()),
        Clause::Use { .. } | Clause::RecordDecl { .. } => return Ok(()),
    };

    let mut subst = Substitution::new();
    let mut var_gen = VarGen::new();
    let mut env = TypeEnv::new();

    // head の Var 引数を TypeEnv に登録。
    if let Term::Struct { args, .. } = head {
        for arg in args {
            if let Term::Var {
                name,
                type_annotation,
                min,
                max,
                ..
            } = arg
            {
                let ty = match type_annotation {
                    Some(t) => t.clone(),
                    None if min.is_some() || max.is_some() => Type::Number,
                    None => var_gen.fresh(),
                };
                env.insert(name.clone(), ty);
            }
        }
    }

    let mut ctx = InferCtx {
        env,
        subst: &mut subst,
        var_gen: &mut var_gen,
        builtins,
        user_sigs,
        records,
    };

    for goal in body {
        // goal の型は Bool 想定 (Eq, predicate call)。型エラーがあれば伝播。
        let _ = infer_term(goal, &mut ctx)?;
    }
    Ok(())
}

/// 診断結果のまとめ。1 つの clause での型エラーを集めて返す。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClauseDiagnostic {
    pub clause_index: usize,
    pub functor: Option<String>,
    pub error: TypeError,
}

/// database 全体に対して型推論を走らせる。
/// clauses を 2 パス処理: まず user signatures を抽出、次に各 clause を `infer_clause`。
/// 失敗した clause だけ `ClauseDiagnostic` として返す (途中で止まらない)。
pub fn infer_database<S: Clone>(
    clauses: &[crate::parse::Clause<S>],
    builtins: &crate::builtins::BuiltinRegistry,
) -> Vec<ClauseDiagnostic> {
    use crate::parse::{Clause, Term};
    let user_sigs = collect_user_signatures(clauses, builtins);
    // 注釈なし field は `Forall("rec_field")` で記録される (clause-local var_gen
    // との ID 衝突回避)。lookup 時に instantiate される。
    let records = RecordDeclTable::from_clauses(clauses);
    let mut diags = Vec::new();
    for (idx, clause) in clauses.iter().enumerate() {
        if let Err(err) = infer_clause(clause, builtins, &user_sigs, &records) {
            let functor = match clause {
                Clause::Fact { head: Term::Struct { functor, .. }, .. }
                | Clause::Rule { head: Term::Struct { functor, .. }, .. } => {
                    Some(functor.clone())
                }
                _ => None,
            };
            diags.push(ClauseDiagnostic {
                clause_index: idx,
                functor,
                error: err,
            });
        }
    }
    diags
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

    // ============================================================
    // infer_term テスト
    // ============================================================

    use crate::parse::{ArithOp, Term, number, struc, var};
    use crate::rational::Rational;

    fn fresh_ctx<'a>(
        subst: &'a mut Substitution,
        var_gen: &'a mut VarGen,
        builtins: &'a crate::builtins::BuiltinRegistry,
        user_sigs: &'a UserSignatures,
        records: &'a RecordDeclTable,
    ) -> InferCtx<'a> {
        InferCtx {
            env: TypeEnv::new(),
            subst,
            var_gen,
            builtins,
            user_sigs,
            records,
        }
    }

    #[test]
    fn infer_number_literal() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let t = infer_term::<()>(&number(Rational::from_integer(42)), &mut ctx).unwrap();
        assert_eq!(t, Type::Number);
    }

    #[test]
    fn infer_cube_returns_shape3d() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let t = struc::<()>(
            "cube".to_string(),
            vec![
                number(Rational::from_integer(10)),
                number(Rational::from_integer(20)),
                number(Rational::from_integer(30)),
            ],
        );
        let ty = infer_term(&t, &mut ctx).unwrap();
        assert_eq!(ty, Type::Shape3D);
    }

    #[test]
    fn infer_union_two_cubes_polymorphic() {
        // union は forall T. T -> T -> T。両引数が Shape3D の cube なので
        // 推論結果は Shape3D に確定する。
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let cube = |sz: i64| {
            struc::<()>(
                "cube".to_string(),
                vec![
                    number(Rational::from_integer(sz)),
                    number(Rational::from_integer(sz)),
                    number(Rational::from_integer(sz)),
                ],
            )
        };
        let t = struc::<()>("union".to_string(), vec![cube(1), cube(2)]);
        let ty = infer_term(&t, &mut ctx).unwrap();
        assert_eq!(ctx.subst.apply(&ty), Type::Shape3D);
    }

    #[test]
    fn infer_var_unannotated_is_fresh_alpha() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let ty = infer_term::<()>(&var("X".to_string()), &mut ctx).unwrap();
        assert!(matches!(ty, Type::Var(_)));
    }

    #[test]
    fn infer_var_annotation_pins_type() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let v = Term::<()>::Var {
            name: "X".to_string(),
            scope: (),
            default_value: None,
            min: None,
            max: None,
            span: None,
            type_annotation: Some(Type::Shape3D),
        };
        let ty = infer_term(&v, &mut ctx).unwrap();
        // ty は env のα、ただし unify でα = Shape3D に bind されている
        let resolved = ctx.subst.apply(&ty);
        assert_eq!(resolved, Type::Shape3D);
    }

    #[test]
    fn infer_list_with_homogeneous_numbers() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let lst = Term::<()>::List {
            items: vec![
                number(Rational::from_integer(1)),
                number(Rational::from_integer(2)),
            ],
            tail: None,
        };
        let ty = infer_term(&lst, &mut ctx).unwrap();
        let resolved = ctx.subst.apply(&ty);
        assert_eq!(resolved, Type::list_of(Type::Number));
    }

    #[test]
    fn infer_list_mismatch_errors() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let lst = Term::<()>::List {
            items: vec![
                number(Rational::from_integer(1)),
                struc::<()>("circle".to_string(), vec![number(Rational::from_integer(2))]),
            ],
            tail: None,
        };
        let r = infer_term(&lst, &mut ctx);
        assert!(matches!(r, Err(TypeError::Mismatch { .. })));
    }

    #[test]
    fn infer_eq_unifies_two_sides() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        // X = cube(10, 10, 10): X should be Shape3D
        let eq = Term::<()>::Eq {
            left: Box::new(var("X".to_string())),
            right: Box::new(struc::<()>(
                "cube".to_string(),
                vec![
                    number(Rational::from_integer(10)),
                    number(Rational::from_integer(10)),
                    number(Rational::from_integer(10)),
                ],
            )),
        };
        let _ = infer_term(&eq, &mut ctx).unwrap();
        let x_ty = ctx.env.get_or_fresh("X", ctx.var_gen);
        assert_eq!(ctx.subst.apply(&x_ty), Type::Shape3D);
    }

    #[test]
    fn infer_arith_requires_numbers() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let expr = Term::<()>::InfixExpr {
            op: ArithOp::Add,
            left: Box::new(number(Rational::from_integer(1))),
            right: Box::new(number(Rational::from_integer(2))),
        };
        let ty = infer_term(&expr, &mut ctx).unwrap();
        assert_eq!(ty, Type::Number);
    }

    #[test]
    fn infer_clause_typed_rule_ok() {
        use crate::parse::{Clause, struc};
        let b = crate::builtins::registry();
        // my_box(Size: Number) :- B = cube(Size, Size, Size).
        let head = struc::<()>(
            "my_box".to_string(),
            vec![Term::Var {
                name: "Size".to_string(),
                scope: (),
                default_value: None,
                min: None,
                max: None,
                span: None,
                type_annotation: Some(Type::Number),
            }],
        );
        let body = vec![Term::Eq {
            left: Box::new(var("B".to_string())),
            right: Box::new(struc::<()>(
                "cube".to_string(),
                vec![
                    var("Size".to_string()),
                    var("Size".to_string()),
                    var("Size".to_string()),
                ],
            )),
        }];
        let clause = Clause::<()>::Rule {
            head,
            body,
            return_type: Some(Type::Shape3D),
        };
        infer_clause(
            &clause,
            &b,
            &UserSignatures::new(),
            &RecordDeclTable::new(),
        )
        .unwrap();
    }

    #[test]
    fn infer_clause_type_mismatch_in_body() {
        use crate::parse::{Clause, struc};
        let b = crate::builtins::registry();
        // bad(X: Number) :- B = cube(X, X, circle(1)).  ← circle returns Shape2D, cube wants Number
        let head = struc::<()>(
            "bad".to_string(),
            vec![Term::Var {
                name: "X".to_string(),
                scope: (),
                default_value: None,
                min: None,
                max: None,
                span: None,
                type_annotation: Some(Type::Number),
            }],
        );
        let body = vec![Term::Eq {
            left: Box::new(var("B".to_string())),
            right: Box::new(struc::<()>(
                "cube".to_string(),
                vec![
                    var("X".to_string()),
                    var("X".to_string()),
                    struc::<()>(
                        "circle".to_string(),
                        vec![number(Rational::from_integer(1))],
                    ),
                ],
            )),
        }];
        let clause = Clause::<()>::Rule {
            head,
            body,
            return_type: None,
        };
        let r = infer_clause(&clause, &b, &UserSignatures::new(), &RecordDeclTable::new());
        assert!(matches!(r, Err(TypeError::Mismatch { .. })));
    }

    #[test]
    fn collect_user_signatures_picks_typed_clauses() {
        use crate::parse::{Clause, struc};
        // foo(X: Number) -> Shape3D :- ... と bar(Y) :- ... の二つ
        let typed_head = struc::<()>(
            "foo".to_string(),
            vec![Term::Var {
                name: "X".to_string(),
                scope: (),
                default_value: None,
                min: None,
                max: None,
                span: None,
                type_annotation: Some(Type::Number),
            }],
        );
        let bare_head = struc::<()>("bar".to_string(), vec![var("Y".to_string())]);
        let clauses = vec![
            Clause::<()>::Rule {
                head: typed_head,
                body: vec![],
                return_type: Some(Type::Shape3D),
            },
            Clause::<()>::Rule {
                head: bare_head,
                body: vec![],
                return_type: None,
            },
        ];
        let us = collect_user_signatures(&clauses, &crate::builtins::registry());
        assert!(us.lookup("foo", 1).is_some());
        assert!(us.lookup("bar", 1).is_none(), "untyped clause should be skipped");
    }

    #[test]
    fn infer_calls_user_predicate_after_sig_collection() {
        use crate::parse::{Clause, struc};
        // foo(X: Number) -> Shape3D :- B = cube(X, X, X).
        // main(M) -> Shape3D :- M = foo(10).
        let foo = Clause::<()>::Rule {
            head: struc::<()>(
                "foo".to_string(),
                vec![Term::Var {
                    name: "X".to_string(),
                    scope: (),
                    default_value: None,
                    min: None,
                    max: None,
                    span: None,
                    type_annotation: Some(Type::Number),
                }],
            ),
            body: vec![Term::Eq {
                left: Box::new(var("B".to_string())),
                right: Box::new(struc::<()>(
                    "cube".to_string(),
                    vec![
                        var("X".to_string()),
                        var("X".to_string()),
                        var("X".to_string()),
                    ],
                )),
            }],
            return_type: Some(Type::Shape3D),
        };
        let main = Clause::<()>::Rule {
            head: struc::<()>("main".to_string(), vec![var("M".to_string())]),
            body: vec![Term::Eq {
                left: Box::new(var("M".to_string())),
                right: Box::new(struc::<()>(
                    "foo".to_string(),
                    vec![number(Rational::from_integer(10))],
                )),
            }],
            return_type: None,
        };
        let clauses = vec![foo, main];
        let b = crate::builtins::registry();
        let diags = infer_database(&clauses, &b);
        assert!(diags.is_empty(), "expected no diagnostics, got {:?}", diags);
    }

    #[test]
    fn infer_database_reports_mismatch() {
        use crate::parse::{Clause, struc};
        // bad(X: Number) -> Shape3D :- B = cube(X, X, circle(1)).
        let clauses = vec![Clause::<()>::Rule {
            head: struc::<()>(
                "bad".to_string(),
                vec![Term::Var {
                    name: "X".to_string(),
                    scope: (),
                    default_value: None,
                    min: None,
                    max: None,
                    span: None,
                    type_annotation: Some(Type::Number),
                }],
            ),
            body: vec![Term::Eq {
                left: Box::new(var("B".to_string())),
                right: Box::new(struc::<()>(
                    "cube".to_string(),
                    vec![
                        var("X".to_string()),
                        var("X".to_string()),
                        struc::<()>(
                            "circle".to_string(),
                            vec![number(Rational::from_integer(1))],
                        ),
                    ],
                )),
            }],
            return_type: Some(Type::Shape3D),
        }];
        let b = crate::builtins::registry();
        let diags = infer_database(&clauses, &b);
        assert_eq!(diags.len(), 1);
        assert_eq!(diags[0].functor.as_deref(), Some("bad"));
        assert!(matches!(diags[0].error, TypeError::Mismatch { .. }));
    }

    fn point_record_decl<S>() -> crate::parse::Clause<S> {
        crate::parse::Clause::RecordDecl {
            name: "p".to_string(),
            fields: vec![
                crate::parse::RecordField {
                    name: "x".to_string(),
                    default: None,
                    ty: Some(Type::Number),
                },
                crate::parse::RecordField {
                    name: "y".to_string(),
                    default: None,
                    ty: Some(Type::Number),
                },
            ],
            span: None,
        }
    }

    #[test]
    fn infer_record_literal_ok() {
        use crate::parse::{Clause, struc};
        let decl = point_record_decl::<()>();
        // make_p(O) -> p :- O = p { x: 1, y: 2 }.
        let rule = Clause::<()>::Rule {
            head: struc::<()>("make_p".to_string(), vec![var("O".to_string())]),
            body: vec![Term::Eq {
                left: Box::new(var("O".to_string())),
                right: Box::new(Term::Record {
                    name: "p".to_string(),
                    fields: vec![
                        (
                            "x".to_string(),
                            number(Rational::from_integer(1)),
                        ),
                        (
                            "y".to_string(),
                            number(Rational::from_integer(2)),
                        ),
                    ],
                    span: None,
                }),
            }],
            return_type: Some(Type::Record("p".to_string())),
        };
        let clauses = vec![decl, rule];
        let b = crate::builtins::registry();
        let diags = infer_database(&clauses, &b);
        assert!(diags.is_empty(), "expected no diagnostics, got {:?}", diags);
    }

    #[test]
    fn infer_record_literal_wrong_field_type_errors() {
        use crate::parse::{Clause, struc};
        let decl = point_record_decl::<()>();
        // bad(O) -> p :- O = p { x: cube(1,1,1), y: 2 }.   ← x should be Number
        let rule = Clause::<()>::Rule {
            head: struc::<()>("bad".to_string(), vec![var("O".to_string())]),
            body: vec![Term::Eq {
                left: Box::new(var("O".to_string())),
                right: Box::new(Term::Record {
                    name: "p".to_string(),
                    fields: vec![
                        (
                            "x".to_string(),
                            struc::<()>(
                                "cube".to_string(),
                                vec![
                                    number(Rational::from_integer(1)),
                                    number(Rational::from_integer(1)),
                                    number(Rational::from_integer(1)),
                                ],
                            ),
                        ),
                        (
                            "y".to_string(),
                            number(Rational::from_integer(2)),
                        ),
                    ],
                    span: None,
                }),
            }],
            return_type: Some(Type::Record("p".to_string())),
        };
        let clauses = vec![decl, rule];
        let b = crate::builtins::registry();
        let diags = infer_database(&clauses, &b);
        assert_eq!(diags.len(), 1);
        assert!(matches!(diags[0].error, TypeError::Mismatch { .. }));
    }

    #[test]
    fn infer_field_access_returns_declared_field_type() {
        use crate::parse::{Clause, struc};
        let decl = point_record_decl::<()>();
        // use_field(R: p) :- N = R.x.   ← N should resolve to Number
        let rule = Clause::<()>::Rule {
            head: struc::<()>(
                "use_field".to_string(),
                vec![Term::Var {
                    name: "R".to_string(),
                    scope: (),
                    default_value: None,
                    min: None,
                    max: None,
                    span: None,
                    type_annotation: Some(Type::Record("p".to_string())),
                }],
            ),
            body: vec![Term::Eq {
                left: Box::new(var("N".to_string())),
                right: Box::new(Term::FieldAccess {
                    record: Box::new(var("R".to_string())),
                    field: "x".to_string(),
                    span: None,
                }),
            }],
            return_type: None,
        };
        let clauses = vec![decl, rule];
        let b = crate::builtins::registry();
        let diags = infer_database(&clauses, &b);
        assert!(diags.is_empty(), "expected no diagnostics, got {:?}", diags);
    }

    #[test]
    fn infer_field_access_unknown_field_errors() {
        use crate::parse::{Clause, struc};
        let decl = point_record_decl::<()>();
        let rule = Clause::<()>::Rule {
            head: struc::<()>(
                "bad".to_string(),
                vec![Term::Var {
                    name: "R".to_string(),
                    scope: (),
                    default_value: None,
                    min: None,
                    max: None,
                    span: None,
                    type_annotation: Some(Type::Record("p".to_string())),
                }],
            ),
            body: vec![Term::Eq {
                left: Box::new(var("N".to_string())),
                right: Box::new(Term::FieldAccess {
                    record: Box::new(var("R".to_string())),
                    field: "z".to_string(),
                    span: None,
                }),
            }],
            return_type: None,
        };
        let clauses = vec![decl, rule];
        let b = crate::builtins::registry();
        let diags = infer_database(&clauses, &b);
        assert_eq!(diags.len(), 1);
        assert!(matches!(diags[0].error, TypeError::MissingSignature { .. }));
    }

    #[test]
    fn infer_unknown_functor_errors_with_missing_sig() {
        let mut s = Substitution::new();
        let mut g = VarGen::new();
        let b = crate::builtins::registry();
        let us = UserSignatures::new();
        let rd = RecordDeclTable::new();
        let mut ctx = fresh_ctx(&mut s, &mut g, &b, &us, &rd);
        let t = struc::<()>("unknown_op".to_string(), vec![]);
        let r = infer_term(&t, &mut ctx);
        assert!(matches!(r, Err(TypeError::MissingSignature { .. })));
    }
}
