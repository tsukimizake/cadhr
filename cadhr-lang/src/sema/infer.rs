//! Algorithm J ベースの HM 型推論。
//!
//! 型変数は [`crate::sema::infer_ty`] の `Rc<RefCell>` 共有セルで表し、単一化は
//! セルを破壊的に書き換える。よって `infer_expr` は型 (`InferTy`) のみを返し、
//! 置換 (`Subst`) を引き回さない。これにより「env / pending に置換を適用し忘れる」
//! 系統のバグが構造的に発生しない。
//!
//! 多相は rank-1 (let-polymorphism + シグネチャ全称量化)。generalize は Rémy の
//! level 方式: 各未束縛変数の生成深さ (`level`) を持ち、`level > 現在深さ` の変数を
//! 全称化する。
//!
//! 公開 API (`infer_unit` / `infer_module`) は immutable な `Scheme` を返す
//! (`CompiledProgram` に載せてスレッド境界を越えるため)。`InferTy` ↔ `Type` の変換は
//! `instantiate` / `generalize` / 結果構築の境界でのみ行う。

use crate::diagnostic::{Diagnostic, Span};
use crate::sema::builtin::BuiltinRegistry;
use crate::sema::class::{ClassCheck, ClassRegistry, Constraint};
use crate::sema::env::TypeEnv;
use crate::sema::infer_ty::{
    InferConstraint, InferScheme, InferTy, TyRef, UnifyError, VarCell, new_generic, new_unbound,
    resolve, to_type_raw, unify, zonk,
};
use crate::sema::ty::{Scheme, TyVar, TyVarGen, Type};
use crate::syntax::ast::*;
use crate::syntax::free_vars::value_decl_free_names;
use std::collections::{HashMap, HashSet};

#[derive(Debug, Default)]
pub struct Infer {
    pub var_gen: TyVarGen,
    /// 現在の let ネスト深さ。fresh 変数の level / generalize 判定に使う。
    pub current_level: u32,
    /// シグネチャ宣言 (`f : T`) の immutable scheme。結果スキームとしても使う。
    pub signatures: HashMap<String, Scheme>,
    /// type alias 宣言。typecheck 時に展開する。
    pub aliases: HashMap<String, TypeAliasDecl>,
    /// ADT 型宣言。コンストラクタ → 関数型を引く (immutable scheme)。
    pub ctor_types: HashMap<String, Scheme>,
    /// record type alias の field マップ (record literal / update / field access で参照)。
    pub record_aliases: HashMap<String, Vec<(String, Type)>>,
    /// type class registry。標準クラス (Num/Ord/Eq) を持つ。
    pub class_registry: ClassRegistry,
    /// 推論中に蓄積した未解決制約。SCC / decl 単位で `solve_constraints` を
    /// 呼んで解消する。span は診断用。
    pub pending: Vec<PendingConstraint>,
    /// 受け手の型がまだ型変数だったフィールドアクセス。SCC 境界で型が確定した後に
    /// 実フィールド型と result を unify する。
    pub pending_fields: Vec<FieldObligation>,
}

#[derive(Clone, Debug)]
pub struct PendingConstraint {
    pub constraint: InferConstraint,
    pub span: Span,
    pub context: String,
}

/// 「receiver の field を取った結果が result 型」という遅延フィールドアクセス。
#[derive(Clone, Debug)]
pub struct FieldObligation {
    pub receiver: InferTy,
    pub field: String,
    pub result: InferTy,
    pub span: Span,
}

/// `lookup_field` の結果。
enum FieldLookup {
    Found(InferTy),
    NoField,
    NoFieldAlias(String),
    Unresolved,
    NotRecord,
}

/// 型変数セルの種別 (制約の generalize 判定に使う)。
enum CellKind {
    Unbound,
    Generic(u32),
}

impl Infer {
    pub fn new() -> Self {
        Self {
            class_registry: ClassRegistry::standard(),
            ..Self::default()
        }
    }

    /// 現在 level の fresh 単型変数を生成。
    pub fn fresh(&mut self) -> InferTy {
        InferTy::Var(new_unbound(self.var_gen.fresh().0, self.current_level))
    }

    pub fn enter_level(&mut self) {
        self.current_level += 1;
    }

    pub fn exit_level(&mut self) {
        self.current_level -= 1;
    }

    /// `InferScheme` を fresh 変数で instantiate する。Generic セルだけを fresh な
    /// 未束縛セルへ置き換え、自由変数 (Unbound) は共有のまま残す。制約は rename して
    /// pending に積む。
    pub fn instantiate_with(&mut self, sc: &InferScheme, span: Span, context: &str) -> InferTy {
        let mut map: HashMap<u32, TyRef> = HashMap::new();
        let ty = self.inst_walk(&sc.ty, &mut map);
        for c in &sc.constraints {
            let cty = self.inst_walk(&c.ty, &mut map);
            self.pending.push(PendingConstraint {
                constraint: InferConstraint {
                    class_name: c.class_name.clone(),
                    ty: cty,
                },
                span,
                context: context.to_string(),
            });
        }
        ty
    }

    fn inst_walk(&mut self, t: &InferTy, map: &mut HashMap<u32, TyRef>) -> InferTy {
        match resolve(t) {
            InferTy::Var(cell) => {
                let gid = match &*cell.borrow() {
                    VarCell::Generic(id) => Some(*id),
                    _ => None,
                };
                match gid {
                    Some(id) => {
                        if let Some(c) = map.get(&id) {
                            return InferTy::Var(c.clone());
                        }
                        let nc = new_unbound(self.var_gen.fresh().0, self.current_level);
                        map.insert(id, nc.clone());
                        InferTy::Var(nc)
                    }
                    None => InferTy::Var(cell),
                }
            }
            InferTy::Con(n, args) => {
                InferTy::Con(n, args.iter().map(|a| self.inst_walk(a, map)).collect())
            }
            InferTy::Arrow(f, to) => {
                InferTy::arrow(self.inst_walk(&f, map), self.inst_walk(&to, map))
            }
            InferTy::Record(fs) => InferTy::Record(
                fs.iter()
                    .map(|(n, t)| (n.clone(), self.inst_walk(t, map)))
                    .collect(),
            ),
        }
    }

    /// 型 ty の generalize: `level > current_level` の未束縛変数を Generic 化して
    /// 全称化する。pending のうち、generalize 対象の変数だけで構成される制約を
    /// scheme.constraints へ「持ち上げ」、それ以外は pending に残す。
    pub fn generalize(&mut self, ty: &InferTy) -> InferScheme {
        let ty = zonk(ty);
        let mut vars: Vec<u32> = Vec::new();
        self.gen_walk(&ty, &mut vars);
        let var_set: HashSet<u32> = vars.iter().copied().collect();

        let mut lifted: Vec<InferConstraint> = Vec::new();
        let mut keep: Vec<PendingConstraint> = Vec::new();
        for p in std::mem::take(&mut self.pending) {
            match self.try_lift(&p, &var_set) {
                Some(c) => lifted.push(c),
                None => keep.push(p),
            }
        }
        self.pending = keep;
        InferScheme {
            vars,
            constraints: lifted,
            ty,
        }
    }

    /// 全称化対象の未束縛変数を Generic セルへ書き換えつつ id を集める。
    fn gen_walk(&mut self, t: &InferTy, vars: &mut Vec<u32>) {
        let rt = resolve(t);
        match &rt {
            InferTy::Var(cell) => {
                enum Act {
                    Generalize(u32),
                    Already(u32),
                    No,
                }
                let act = match &*cell.borrow() {
                    VarCell::Unbound { id, level } => {
                        if *level > self.current_level {
                            Act::Generalize(*id)
                        } else {
                            Act::No
                        }
                    }
                    VarCell::Generic(id) => Act::Already(*id),
                    VarCell::Link(_) => Act::No,
                };
                match act {
                    Act::Generalize(id) => {
                        *cell.borrow_mut() = VarCell::Generic(id);
                        if !vars.contains(&id) {
                            vars.push(id);
                        }
                    }
                    Act::Already(id) => {
                        if !vars.contains(&id) {
                            vars.push(id);
                        }
                    }
                    Act::No => {}
                }
            }
            InferTy::Con(_, args) => {
                for a in args {
                    self.gen_walk(a, vars);
                }
            }
            InferTy::Arrow(f, to) => {
                self.gen_walk(f, vars);
                self.gen_walk(to, vars);
            }
            InferTy::Record(fs) => {
                for (_, t) in fs {
                    self.gen_walk(t, vars);
                }
            }
        }
    }

    /// 制約が「全称化対象の変数だけ」で構成されるなら scheme に持ち上げる。
    /// 自由変数 (Unbound) を含む / 対象外の Generic を含む / 変数を全く含まない場合は
    /// 持ち上げず pending に残す (具体型制約は finalize_pending で判定)。
    fn try_lift(&self, p: &PendingConstraint, var_set: &HashSet<u32>) -> Option<InferConstraint> {
        let zc = zonk(&p.constraint.ty);
        let mut kinds = Vec::new();
        cell_kinds(&zc, &mut kinds);
        if kinds.is_empty() {
            return None;
        }
        let liftable = kinds.iter().all(|k| match k {
            CellKind::Unbound => false,
            CellKind::Generic(id) => var_set.contains(id),
        });
        if liftable {
            Some(InferConstraint {
                class_name: p.constraint.class_name.clone(),
                ty: zc,
            })
        } else {
            None
        }
    }

    /// immutable `Scheme` を推論用 `InferScheme` に変換する (量化変数を Generic セルに)。
    pub fn scheme_to_infer(&mut self, sc: &Scheme) -> InferScheme {
        let mut map: HashMap<TyVar, TyRef> = HashMap::new();
        let mut vars: Vec<u32> = Vec::new();
        for tv in &sc.vars {
            let id = self.var_gen.fresh().0;
            map.insert(*tv, new_generic(id));
            vars.push(id);
        }
        let ty = self.type_to_generic(&sc.ty, &mut map, &mut vars);
        let constraints = sc
            .constraints
            .iter()
            .map(|c| InferConstraint {
                class_name: c.class_name.clone(),
                ty: self.type_to_generic(&c.ty, &mut map, &mut vars),
            })
            .collect();
        InferScheme {
            vars,
            constraints,
            ty,
        }
    }

    fn type_to_generic(
        &mut self,
        t: &Type,
        map: &mut HashMap<TyVar, TyRef>,
        vars: &mut Vec<u32>,
    ) -> InferTy {
        match t {
            Type::Var(tv) => {
                if let Some(cell) = map.get(tv) {
                    return InferTy::Var(cell.clone());
                }
                let id = self.var_gen.fresh().0;
                let cell = new_generic(id);
                map.insert(*tv, cell.clone());
                vars.push(id);
                InferTy::Var(cell)
            }
            Type::Con(n, args) => InferTy::Con(
                n.clone(),
                args.iter()
                    .map(|a| self.type_to_generic(a, map, vars))
                    .collect(),
            ),
            Type::Arrow(f, to) => InferTy::arrow(
                self.type_to_generic(f, map, vars),
                self.type_to_generic(to, map, vars),
            ),
            Type::Record(fs) => InferTy::Record(
                fs.iter()
                    .map(|(n, t)| (n.clone(), self.type_to_generic(t, map, vars)))
                    .collect(),
            ),
        }
    }

    /// immutable `Type` を fresh な未束縛変数つきの `InferTy` に変換する
    /// (record alias の field 型を使う場面など、その場で instantiate したいとき)。
    fn type_to_infer_fresh(&mut self, t: &Type) -> InferTy {
        let mut map: HashMap<TyVar, TyRef> = HashMap::new();
        self.type_to_infer_fresh_walk(t, &mut map)
    }

    fn type_to_infer_fresh_walk(&mut self, t: &Type, map: &mut HashMap<TyVar, TyRef>) -> InferTy {
        match t {
            Type::Var(tv) => {
                if let Some(cell) = map.get(tv) {
                    return InferTy::Var(cell.clone());
                }
                let cell = new_unbound(self.var_gen.fresh().0, self.current_level);
                map.insert(*tv, cell.clone());
                InferTy::Var(cell)
            }
            Type::Con(n, args) => InferTy::Con(
                n.clone(),
                args.iter()
                    .map(|a| self.type_to_infer_fresh_walk(a, map))
                    .collect(),
            ),
            Type::Arrow(f, to) => InferTy::arrow(
                self.type_to_infer_fresh_walk(f, map),
                self.type_to_infer_fresh_walk(to, map),
            ),
            Type::Record(fs) => InferTy::Record(
                fs.iter()
                    .map(|(n, t)| (n.clone(), self.type_to_infer_fresh_walk(t, map)))
                    .collect(),
            ),
        }
    }

    /// `Type` を `InferTy` に戻す。`map` にある変数 id は元のセルへ復元し
    /// (`infer_to_type_tracked` と対で使う)、無い id は fresh な未束縛セルにする。
    /// derived 制約を元の変数と結びつけ直すために使う。
    fn type_to_infer_via_map(&mut self, t: &Type, map: &HashMap<u32, TyRef>) -> InferTy {
        match t {
            Type::Var(tv) => match map.get(&tv.0) {
                Some(cell) => InferTy::Var(cell.clone()),
                None => InferTy::Var(new_unbound(self.var_gen.fresh().0, self.current_level)),
            },
            Type::Con(n, args) => InferTy::Con(
                n.clone(),
                args.iter()
                    .map(|a| self.type_to_infer_via_map(a, map))
                    .collect(),
            ),
            Type::Arrow(f, to) => InferTy::arrow(
                self.type_to_infer_via_map(f, map),
                self.type_to_infer_via_map(to, map),
            ),
            Type::Record(fs) => InferTy::Record(
                fs.iter()
                    .map(|(n, t)| (n.clone(), self.type_to_infer_via_map(t, map)))
                    .collect(),
            ),
        }
    }

    /// 型 `recv` から field を引いた結果。
    fn lookup_field(&mut self, recv: &InferTy, field: &str) -> FieldLookup {
        match resolve(recv) {
            InferTy::Record(fs) => match fs.iter().find(|(n, _)| n == field) {
                Some((_, t)) => FieldLookup::Found(t.clone()),
                None => FieldLookup::NoField,
            },
            InferTy::Con(alias, _) => match self.record_aliases.get(&alias).cloned() {
                Some(fields) => match fields.iter().find(|(n, _)| n == field) {
                    Some((_, t)) => FieldLookup::Found(self.type_to_infer_fresh(t)),
                    None => FieldLookup::NoFieldAlias(alias),
                },
                None => FieldLookup::NotRecord,
            },
            InferTy::Var(_) => FieldLookup::Unresolved,
            _ => FieldLookup::NotRecord,
        }
    }

    /// 全 decl 推論を終えた時点で残った制約を処理する。
    /// - 具体型に解決された制約 → ClassCheck で Yes/No 判定 (Yes なら derived を再投入)。
    /// - 型変数のまま残った制約 → `AmbiguousConstraint` エラー (暗黙 default は無し)。
    pub fn finalize_pending(&mut self, diag: &mut Vec<Diagnostic>) {
        loop {
            let pending = std::mem::take(&mut self.pending);
            if pending.is_empty() {
                return;
            }
            for p in pending {
                let mut map: HashMap<u32, TyRef> = HashMap::new();
                let zt = infer_to_type_tracked(&p.constraint.ty, &mut map);
                match self.class_registry.check(&p.constraint.class_name, &zt) {
                    ClassCheck::Yes { derived } => {
                        for d in derived {
                            let ity = self.type_to_infer_via_map(&d.ty, &map);
                            self.pending.push(PendingConstraint {
                                constraint: InferConstraint {
                                    class_name: d.class_name,
                                    ty: ity,
                                },
                                span: p.span,
                                context: p.context.clone(),
                            });
                        }
                    }
                    ClassCheck::No => {
                        diag.push(Diagnostic::NoInstance {
                            span: p.span,
                            context: p.context.clone(),
                            class_name: p.constraint.class_name.clone(),
                            ty: format!("{zt}"),
                        });
                    }
                    ClassCheck::Unknown => {
                        diag.push(Diagnostic::AmbiguousConstraint {
                            span: p.span,
                            context: p.context.clone(),
                            class_name: p.constraint.class_name.clone(),
                        });
                    }
                }
            }
        }
    }

    /// TypeExpr (AST) を Type (semantic) に変換する。type alias / ADT 名・type
    /// variables を分かる範囲で解決する。
    pub fn ty_of_type_expr(
        &self,
        t: &TypeExpr,
        var_map: &mut HashMap<String, TyVar>,
        diag: &mut Vec<Diagnostic>,
    ) -> Type {
        match t {
            TypeExpr::Var(name, _span) => {
                if let Some(&v) = var_map.get(name) {
                    return Type::Var(v);
                }
                // signature scope 内では、初出の type var にダミー ID を割当てる。
                // 後で `remap_named_vars` で fresh ID に置き換える。
                let dummy_id = TyVar(var_map.len() as u32 + 100_000);
                var_map.insert(name.clone(), dummy_id);
                Type::Var(dummy_id)
            }
            TypeExpr::Con { name, args, .. } => {
                let args_ty: Vec<Type> = args
                    .iter()
                    .map(|a| self.ty_of_type_expr(a, var_map, diag))
                    .collect();
                // 既知の type alias なら展開する (shallow alias のみ)
                if let Some(alias) = self.aliases.get(name) {
                    if alias.params.len() != args_ty.len() {
                        diag.push(Diagnostic::TypeAliasArity {
                            span: span_of_type_expr(t),
                            name: name.clone(),
                            expected: alias.params.len(),
                            got: args_ty.len(),
                        });
                        return Type::con("<error>");
                    }
                    let mut subst_map: HashMap<String, Type> = HashMap::new();
                    for (p, a) in alias.params.iter().zip(args_ty.iter()) {
                        subst_map.insert(p.clone(), a.clone());
                    }
                    let mut alias_var_map: HashMap<String, TyVar> = HashMap::new();
                    let raw = self.ty_of_type_expr(&alias.body, &mut alias_var_map, diag);
                    return substitute_named(&raw, &subst_map, &alias_var_map);
                }
                // Point2D / Point3D は組み込みの構造的 record 型に解決する。
                match name.as_str() {
                    "Point2D" => {
                        return Type::Record(vec![
                            ("x".to_string(), Type::con("Float")),
                            ("y".to_string(), Type::con("Float")),
                        ]);
                    }
                    "Point3D" => {
                        return Type::Record(vec![
                            ("x".to_string(), Type::con("Float")),
                            ("y".to_string(), Type::con("Float")),
                            ("z".to_string(), Type::con("Float")),
                        ]);
                    }
                    _ => {}
                }
                Type::Con(name.clone(), args_ty)
            }
            TypeExpr::Arrow { from, to, .. } => Type::arrow(
                self.ty_of_type_expr(from, var_map, diag),
                self.ty_of_type_expr(to, var_map, diag),
            ),
            TypeExpr::Record(fields, _) => {
                let fs: Vec<(String, Type)> = fields
                    .iter()
                    .map(|f| (f.name.clone(), self.ty_of_type_expr(&f.ty, var_map, diag)))
                    .collect();
                Type::Record(fs)
            }
        }
    }
}

/// `InferTy` を `Type` に焼きつつ、各変数の `id → セル` を記録する。
/// `ClassRegistry::check` (Type ベース) に渡しつつ、derived 制約を元のセルへ
/// 戻せるようにするため (セル同一性を保たないと偽の AmbiguousConstraint が出る)。
fn infer_to_type_tracked(t: &InferTy, map: &mut HashMap<u32, TyRef>) -> Type {
    match resolve(t) {
        InferTy::Var(cell) => {
            let id = match &*cell.borrow() {
                VarCell::Unbound { id, .. } | VarCell::Generic(id) => *id,
                VarCell::Link(_) => unreachable!("resolve 済み"),
            };
            map.insert(id, cell.clone());
            Type::Var(TyVar(id))
        }
        InferTy::Con(n, args) => Type::Con(
            n,
            args.iter().map(|a| infer_to_type_tracked(a, map)).collect(),
        ),
        InferTy::Arrow(f, to) => Type::arrow(
            infer_to_type_tracked(&f, map),
            infer_to_type_tracked(&to, map),
        ),
        InferTy::Record(fs) => Type::Record(
            fs.iter()
                .map(|(n, t)| (n.clone(), infer_to_type_tracked(t, map)))
                .collect(),
        ),
    }
}

/// `InferTy` に出現する型変数セルの種別を集める。
fn cell_kinds(t: &InferTy, out: &mut Vec<CellKind>) {
    match resolve(t) {
        InferTy::Var(cell) => match &*cell.borrow() {
            VarCell::Unbound { .. } => out.push(CellKind::Unbound),
            VarCell::Generic(id) => out.push(CellKind::Generic(*id)),
            VarCell::Link(_) => unreachable!("resolve 済み"),
        },
        InferTy::Con(_, args) => {
            for a in &args {
                cell_kinds(a, out);
            }
        }
        InferTy::Arrow(f, to) => {
            cell_kinds(&f, out);
            cell_kinds(&to, out);
        }
        InferTy::Record(fs) => {
            for (_, t) in &fs {
                cell_kinds(t, out);
            }
        }
    }
}

/// `InferScheme` (推論用) を immutable `Scheme` (公開用) に焼き付ける。
/// 結果は top-level / let 束縛の確定型なので閉じている前提。
fn infer_scheme_to_scheme(isc: &InferScheme) -> Scheme {
    let ty = to_type_raw(&isc.ty);
    let constraints = isc
        .constraints
        .iter()
        .map(|c| Constraint {
            class_name: c.class_name.clone(),
            ty: to_type_raw(&c.ty),
        })
        .collect();
    let vars = isc.vars.iter().map(|id| TyVar(*id)).collect();
    Scheme {
        vars,
        constraints,
        ty,
    }
}

/// `TypeExpr` の span を取り出す helper。
fn span_of_type_expr(t: &TypeExpr) -> Span {
    match t {
        TypeExpr::Var(_, s)
        | TypeExpr::Con { span: s, .. }
        | TypeExpr::Arrow { span: s, .. }
        | TypeExpr::Record(_, s) => *s,
    }
}

/// 名前ベースの type variable を `subst_map` で展開する (type alias 展開用)。
fn substitute_named(
    ty: &Type,
    subst_map: &HashMap<String, Type>,
    var_map: &HashMap<String, TyVar>,
) -> Type {
    match ty {
        Type::Var(v) => {
            let name = var_map
                .iter()
                .find(|(_, vv)| **vv == *v)
                .map(|(n, _)| n.clone());
            match name.and_then(|n| subst_map.get(&n).cloned()) {
                Some(t) => t,
                None => Type::Var(*v),
            }
        }
        Type::Con(n, args) => Type::Con(
            n.clone(),
            args.iter()
                .map(|a| substitute_named(a, subst_map, var_map))
                .collect(),
        ),
        Type::Arrow(f, t) => Type::Arrow(
            Box::new(substitute_named(f, subst_map, var_map)),
            Box::new(substitute_named(t, subst_map, var_map)),
        ),
        Type::Record(fs) => Type::Record(
            fs.iter()
                .map(|(n, t)| (n.clone(), substitute_named(t, subst_map, var_map)))
                .collect(),
        ),
    }
}

/// 値式の qualified 名 (`Foo.bar` 形式) を作る。
fn qualify(module: Option<&ModuleName>, name: &str) -> String {
    match module {
        Some(m) if !m.segments.is_empty() => format!("{}.{}", m.segments.join("."), name),
        _ => name.to_string(),
    }
}

fn qualify_str(module: &str, name: &str) -> String {
    if module.is_empty() {
        name.to_string()
    } else {
        format!("{module}.{name}")
    }
}

/// 解決済み複数モジュールの型推論。各モジュールの decl 名 → Scheme を返す。
/// main モジュールは `""` キーで引ける。
pub fn infer_unit(
    unit: &crate::module::ResolvedUnit,
    builtins: &BuiltinRegistry,
) -> (HashMap<String, HashMap<String, Scheme>>, Vec<Diagnostic>) {
    let mut per_module: HashMap<String, HashMap<String, Scheme>> = HashMap::new();
    let mut all_diag: Vec<Diagnostic> = Vec::new();

    let mut module_locals: HashMap<String, HashMap<String, Scheme>> = HashMap::new();

    for lm in &unit.modules {
        let (schemes, diag) = infer_one(lm, builtins, &module_locals);
        module_locals.insert(lm.qualified_name.clone(), schemes.clone());
        per_module.insert(lm.qualified_name.clone(), schemes);
        all_diag.extend(diag);
    }
    (per_module, all_diag)
}

fn infer_one(
    lm: &crate::module::LoadedModule,
    builtins: &BuiltinRegistry,
    module_locals: &HashMap<String, HashMap<String, Scheme>>,
) -> (HashMap<String, Scheme>, Vec<Diagnostic>) {
    let mut infer = Infer::new();
    let mut diag: Vec<Diagnostic> = Vec::new();
    let mut env = TypeEnv::new();

    // 1. builtins
    for b in builtins.iter() {
        let isc = infer.scheme_to_infer(&b.scheme);
        env = env.extend(b.name, isc);
    }

    // 2. import 由来の scheme を qualified + (exposing) unqualified で展開
    for imp in &lm.module.imports {
        let imp_qname = crate::module::join_name(&imp.module);
        let Some(imp_schemes) = module_locals.get(&imp_qname) else {
            diag.push(Diagnostic::ImportTypeNotLoaded {
                span: imp.span,
                qname: imp_qname.clone(),
            });
            continue;
        };
        let prefix = imp.alias.clone().unwrap_or_else(|| imp_qname.clone());
        for (k, sc) in imp_schemes {
            let isc = infer.scheme_to_infer(sc);
            env = env.extend(&qualify_str(&prefix, k), isc);
        }
        match &imp.exposing {
            Some(Exposing::All(_)) => {
                for (k, sc) in imp_schemes {
                    let isc = infer.scheme_to_infer(sc);
                    env = env.extend(k, isc);
                }
            }
            Some(Exposing::Some(items, _)) => {
                for item in items {
                    if let Some(sc) = imp_schemes.get(&item.name) {
                        let isc = infer.scheme_to_infer(sc);
                        env = env.extend(&item.name, isc);
                    }
                }
            }
            None => {}
        }
    }

    // 3. 自モジュールに対して既存ロジック (型 alias 収集 → ctor → signature → value) を回す
    let (schemes, mut m_diag) = infer_module_into(&lm.module, &mut infer, &mut env);
    diag.append(&mut m_diag);

    // qualified 名も追加して返す
    let mut out = HashMap::new();
    for (k, v) in &schemes {
        out.insert(k.clone(), v.clone());
        if !lm.qualified_name.is_empty() {
            out.insert(qualify_str(&lm.qualified_name, k), v.clone());
        }
    }
    (out, diag)
}

/// `infer_module` の中身。`env` には builtins と import 由来の scheme を詰めて渡す。
fn infer_module_into(
    module: &Module,
    infer: &mut Infer,
    env_in: &mut TypeEnv,
) -> (HashMap<String, Scheme>, Vec<Diagnostic>) {
    let mut diag: Vec<Diagnostic> = Vec::new();
    let mut env = env_in.clone();

    // 2. type alias / type decl / signature を先に収集
    for decl in &module.decls {
        match decl {
            Decl::TypeAlias(a) => {
                infer.aliases.insert(a.name.clone(), a.clone());
                if let TypeExpr::Record(fields, _) = &a.body {
                    let mut alias_var_map: HashMap<String, TyVar> = HashMap::new();
                    let fs: Vec<(String, Type)> = fields
                        .iter()
                        .map(|f| {
                            (
                                f.name.clone(),
                                infer.ty_of_type_expr(&f.ty, &mut alias_var_map, &mut diag),
                            )
                        })
                        .collect();
                    infer.record_aliases.insert(a.name.clone(), fs);
                }
            }
            Decl::Type(t) => {
                let param_vars: Vec<TyVar> =
                    t.params.iter().map(|_| infer.var_gen.fresh()).collect();
                let mut var_map: HashMap<String, TyVar> = t
                    .params
                    .iter()
                    .zip(param_vars.iter())
                    .map(|(n, v)| (n.clone(), *v))
                    .collect();
                let ret_ty = Type::Con(
                    t.name.clone(),
                    param_vars.iter().map(|v| Type::Var(*v)).collect(),
                );
                for ctor in &t.constructors {
                    let arg_tys: Vec<Type> = ctor
                        .args
                        .iter()
                        .map(|a| infer.ty_of_type_expr(a, &mut var_map, &mut diag))
                        .collect();
                    let ty = Type::arrows(arg_tys, ret_ty.clone());
                    let scheme = Scheme {
                        vars: param_vars.clone(),
                        constraints: Vec::new(),
                        ty,
                    };
                    infer.ctor_types.insert(ctor.name.clone(), scheme);
                }
            }
            Decl::Signature(s) => {
                let mut var_map: HashMap<String, TyVar> = HashMap::new();
                let raw_ty = infer.ty_of_type_expr(&s.ty, &mut var_map, &mut diag);
                let mut name_to_fresh: HashMap<String, TyVar> = HashMap::new();
                for n in var_map.keys() {
                    name_to_fresh.insert(n.clone(), infer.var_gen.fresh());
                }
                let final_ty = remap_named_vars(&raw_ty, &var_map, &name_to_fresh);
                let vars: Vec<TyVar> = {
                    let mut vs: Vec<TyVar> = name_to_fresh.values().copied().collect();
                    vs.sort();
                    vs
                };
                let scheme = Scheme {
                    vars,
                    constraints: Vec::new(),
                    ty: final_ty,
                };
                infer.signatures.insert(s.name.clone(), scheme);
            }
            _ => {}
        }
    }

    // 3. コンストラクタと signature を環境に展開 (InferScheme 化)
    for (name, sc) in infer.ctor_types.clone() {
        let isc = infer.scheme_to_infer(&sc);
        env = env.extend(&name, isc);
    }
    for (name, sc) in infer.signatures.clone() {
        let isc = infer.scheme_to_infer(&sc);
        env = env.extend(&name, isc);
    }

    // 4. value decl の依存解析 → SCC → 葉から順に推論
    let value_decls: Vec<&ValueDecl> = module
        .decls
        .iter()
        .filter_map(|d| match d {
            Decl::Value(v) => Some(v),
            _ => None,
        })
        .collect();
    let decl_names: HashSet<String> = value_decls.iter().map(|v| v.name.clone()).collect();

    let mut graph: HashMap<String, Vec<String>> = HashMap::new();
    for v in &value_decls {
        let used = value_decl_free_names(v);
        let mut deps: Vec<String> = used.into_iter().filter(|n| decl_names.contains(n)).collect();
        deps.sort();
        graph.insert(v.name.clone(), deps);
    }

    let sccs = tarjan_scc(&value_decls, &graph);

    let mut result: HashMap<String, Scheme> = HashMap::new();
    for scc in sccs {
        infer_value_scc(infer, &mut env, &scc, &mut diag, &mut result);
    }

    // 全 SCC を処理し終えた時点で、まだ未解決の制約をすべて判定する。
    infer.finalize_pending(&mut diag);

    *env_in = env;
    (result, diag)
}

/// 1 つの SCC を一括で推論する。SCC 内は相互参照可能なので、各メンバを fresh
/// monotype var (sig 無し) または instantiate された sig 型 (sig 有り) で
/// 一時 env に bind し、全 body を推論してから unify で整合させる。
/// 終了後に sig 無しメンバを level ベースで generalize する。
fn infer_value_scc(
    infer: &mut Infer,
    env: &mut TypeEnv,
    scc: &[&ValueDecl],
    diag: &mut Vec<Diagnostic>,
    result: &mut HashMap<String, Scheme>,
) {
    infer.enter_level();

    // 各メンバ: sig 有なら expected = instantiate(sig)、sig 無なら fresh var。
    let mut local_env = env.clone();
    let mut member_expected: HashMap<String, InferTy> = HashMap::new();
    for m in scc {
        let expected = if let Some(sc) = infer.signatures.get(&m.name).cloned() {
            let isc = infer.scheme_to_infer(&sc);
            infer.instantiate_with(&isc, m.span, &m.name)
        } else {
            let fresh = infer.fresh();
            local_env = local_env.extend(&m.name, InferScheme::mono(fresh.clone()));
            fresh
        };
        member_expected.insert(m.name.clone(), expected);
    }

    // 各 body を推論し、expected と単一化 (セルを破壊的に更新)。
    for m in scc {
        let expected = member_expected.get(&m.name).unwrap().clone();
        let expected_arg = if infer.signatures.contains_key(&m.name) {
            Some(expected.clone())
        } else {
            None
        };
        let ty = match infer_value_decl(infer, &local_env, m, expected_arg.as_ref(), diag) {
            Some(t) => t,
            None => continue,
        };
        if let Err(e) = unify(&ty, &expected) {
            diag.push(unify_diag(e, m.span, &m.name));
        }
    }

    infer.exit_level();

    // 遅延フィールドアクセスを解決 (型が伸びうる) → 制約を solve。
    resolve_field_obligations(infer, diag);
    solve_constraints(infer, diag);

    // finalize: sig 有 → 宣言スキーム、sig 無 → level ベースで generalize。
    for m in scc {
        if let Some(declared) = infer.signatures.get(&m.name).cloned() {
            let isc = infer.scheme_to_infer(&declared);
            *env = env.extend(&m.name, isc);
            result.insert(m.name.clone(), declared);
        } else {
            let expected = member_expected.get(&m.name).unwrap().clone();
            let isc = infer.generalize(&expected);
            let scheme = infer_scheme_to_scheme(&isc);
            *env = env.extend(&m.name, isc);
            result.insert(m.name.clone(), scheme);
        }
    }
}

/// pending の制約を進める。
/// - 具体型 → ClassCheck::Yes/No に従って derived を pending に / 違反を diag に。
/// - 型変数のまま → pending に残す (後で finalize_pending か generalize で処理)。
fn solve_constraints(infer: &mut Infer, diag: &mut Vec<Diagnostic>) {
    let mut keep: Vec<PendingConstraint> = Vec::new();
    let pending = std::mem::take(&mut infer.pending);
    for p in pending {
        let mut map: HashMap<u32, TyRef> = HashMap::new();
        let zt = infer_to_type_tracked(&p.constraint.ty, &mut map);
        match infer.class_registry.check(&p.constraint.class_name, &zt) {
            ClassCheck::Yes { derived } => {
                for d in derived {
                    let ity = infer.type_to_infer_via_map(&d.ty, &map);
                    keep.push(PendingConstraint {
                        constraint: InferConstraint {
                            class_name: d.class_name,
                            ty: ity,
                        },
                        span: p.span,
                        context: p.context.clone(),
                    });
                }
            }
            ClassCheck::No => {
                diag.push(Diagnostic::NoInstance {
                    span: p.span,
                    context: p.context.clone(),
                    class_name: p.constraint.class_name.clone(),
                    ty: format!("{zt}"),
                });
            }
            ClassCheck::Unknown => {
                keep.push(p);
            }
        }
    }
    infer.pending = keep;
}

/// 遅延していたフィールドアクセスを型確定後に解決する。受け手が record/alias に
/// 解決されていれば field 型と result を unify する。型変数のまま / record でない
/// 場合は診断を出す。
fn resolve_field_obligations(infer: &mut Infer, diag: &mut Vec<Diagnostic>) {
    for ob in std::mem::take(&mut infer.pending_fields) {
        match infer.lookup_field(&ob.receiver, &ob.field) {
            FieldLookup::Found(field_ty) => {
                if let Err(e) = unify(&field_ty, &ob.result) {
                    diag.push(unify_diag(e, ob.span, &format!(".{}", ob.field)));
                }
            }
            FieldLookup::NoField => diag.push(Diagnostic::RecordNoField {
                span: ob.span,
                field: ob.field,
            }),
            FieldLookup::NoFieldAlias(alias) => diag.push(Diagnostic::AliasNoField {
                span: ob.span,
                alias,
                field: ob.field,
            }),
            FieldLookup::Unresolved | FieldLookup::NotRecord => {
                diag.push(Diagnostic::NotARecord {
                    span: ob.span,
                    field: ob.field,
                    ty: format!("{}", to_type_raw(&ob.receiver)),
                })
            }
        }
    }
}

/// Tarjan's SCC algorithm。返値は **逆 topological 順** (依存グラフの葉から根) で
/// 返るので、そのまま for-each すれば依存先が先に処理される。
fn tarjan_scc<'a>(
    decls: &[&'a ValueDecl],
    graph: &HashMap<String, Vec<String>>,
) -> Vec<Vec<&'a ValueDecl>> {
    let mut by_name: HashMap<String, &'a ValueDecl> = HashMap::new();
    for v in decls {
        by_name.insert(v.name.clone(), *v);
    }
    let mut st = TarjanState::default();
    for v in decls {
        if !st.index.contains_key(&v.name) {
            tarjan_visit(&v.name, graph, &by_name, &mut st);
        }
    }
    st.sccs
}

#[derive(Default)]
struct TarjanState<'a> {
    counter: usize,
    index: HashMap<String, usize>,
    lowlink: HashMap<String, usize>,
    on_stack: HashSet<String>,
    stack: Vec<String>,
    sccs: Vec<Vec<&'a ValueDecl>>,
}

fn tarjan_visit<'a>(
    v: &str,
    graph: &HashMap<String, Vec<String>>,
    by_name: &HashMap<String, &'a ValueDecl>,
    st: &mut TarjanState<'a>,
) {
    let v_owned = v.to_string();
    st.index.insert(v_owned.clone(), st.counter);
    st.lowlink.insert(v_owned.clone(), st.counter);
    st.counter += 1;
    st.stack.push(v_owned.clone());
    st.on_stack.insert(v_owned.clone());

    if let Some(deps) = graph.get(v) {
        for w in deps {
            if !st.index.contains_key(w) {
                tarjan_visit(w, graph, by_name, st);
                let lw = st.lowlink[w];
                let lv = st.lowlink[&v_owned];
                st.lowlink.insert(v_owned.clone(), lv.min(lw));
            } else if st.on_stack.contains(w) {
                let iw = st.index[w];
                let lv = st.lowlink[&v_owned];
                st.lowlink.insert(v_owned.clone(), lv.min(iw));
            }
        }
    }

    if st.lowlink[&v_owned] == st.index[&v_owned] {
        let mut scc: Vec<&'a ValueDecl> = Vec::new();
        loop {
            let w = st.stack.pop().expect("tarjan stack underflow");
            st.on_stack.remove(&w);
            if let Some(decl) = by_name.get(&w) {
                scc.push(*decl);
            }
            if w == v_owned {
                break;
            }
        }
        st.sccs.push(scc);
    }
}

/// 1 つのモジュール全体の型推論。後方互換 API (builtins だけ環境に入れて呼ぶ)。
pub fn infer_module(
    module: &Module,
    builtins: &BuiltinRegistry,
) -> (HashMap<String, Scheme>, Vec<Diagnostic>) {
    let mut infer = Infer::new();
    let mut env = TypeEnv::new();
    for b in builtins.iter() {
        let isc = infer.scheme_to_infer(&b.scheme);
        env = env.extend(b.name, isc);
    }
    infer_module_into(module, &mut infer, &mut env)
}

fn remap_named_vars(
    ty: &Type,
    var_map: &HashMap<String, TyVar>,
    name_to_fresh: &HashMap<String, TyVar>,
) -> Type {
    match ty {
        Type::Var(v) => {
            let name = var_map
                .iter()
                .find(|(_, vv)| **vv == *v)
                .map(|(n, _)| n.clone());
            match name.and_then(|n| name_to_fresh.get(&n)) {
                Some(new_v) => Type::Var(*new_v),
                None => Type::Var(*v),
            }
        }
        Type::Con(n, args) => Type::Con(
            n.clone(),
            args.iter()
                .map(|a| remap_named_vars(a, var_map, name_to_fresh))
                .collect(),
        ),
        Type::Arrow(f, t) => Type::Arrow(
            Box::new(remap_named_vars(f, var_map, name_to_fresh)),
            Box::new(remap_named_vars(t, var_map, name_to_fresh)),
        ),
        Type::Record(fs) => Type::Record(
            fs.iter()
                .map(|(n, t)| (n.clone(), remap_named_vars(t, var_map, name_to_fresh)))
                .collect(),
        ),
    }
}

fn unify_diag(err: UnifyError, span: Span, name: &str) -> Diagnostic {
    let context = name.to_string();
    match err {
        UnifyError::Mismatch { left, right } => Diagnostic::TypeMismatch {
            span,
            context,
            left,
            right,
        },
        UnifyError::InfiniteType { ty } => Diagnostic::InfiniteType { span, context, ty },
        UnifyError::RecordFieldMismatch {
            left_fields,
            right_fields,
        } => Diagnostic::RecordFieldMismatch {
            span,
            context,
            left_fields,
            right_fields,
        },
    }
}

fn infer_value_decl(
    infer: &mut Infer,
    env: &TypeEnv,
    v: &ValueDecl,
    expected: Option<&InferTy>,
    diag: &mut Vec<Diagnostic>,
) -> Option<InferTy> {
    // signature があれば param 型を body 推論前に確定させる (record の field access 等、
    // param 型が分かっていないと推論できない式を body 内で許すため)。無ければ fresh。
    let sig_param_tys = expected
        .map(|t| split_arrows(t, v.params.len()).0)
        .unwrap_or_default();
    let mut local_env = env.clone();
    let mut param_tys: Vec<InferTy> = Vec::new();
    for (i, p) in v.params.iter().enumerate() {
        let pt = sig_param_tys.get(i).cloned().unwrap_or_else(|| infer.fresh());
        param_tys.push(pt.clone());
        bind_pattern(infer, &mut local_env, p, &pt, diag);
    }
    let body_ty = infer_expr(infer, &local_env, &v.body, diag)?;
    let mut fun_ty = body_ty;
    for pt in param_tys.into_iter().rev() {
        fun_ty = InferTy::arrow(pt, fun_ty);
    }
    Some(fun_ty)
}

/// pattern を env に束縛する。単一化はセルを破壊的に更新する。
fn bind_pattern(
    infer: &mut Infer,
    env: &mut TypeEnv,
    p: &Pattern,
    expected: &InferTy,
    diag: &mut Vec<Diagnostic>,
) {
    match p {
        Pattern::Var(name, _) => {
            *env = env.extend(name, InferScheme::mono(expected.clone()));
        }
        Pattern::Wildcard(_) => {}
        Pattern::Lit(lit, span) => {
            if let Err(e) = unify(expected, &lit_type(lit)) {
                diag.push(unify_diag(e, *span, "<literal pattern>"));
            }
        }
        Pattern::Ctor {
            name, args, span, ..
        } => {
            let sc = match infer.ctor_types.get(name).cloned() {
                Some(s) => s,
                None => {
                    diag.push(Diagnostic::CtorUndefinedInPattern {
                        span: *span,
                        name: name.clone(),
                    });
                    return;
                }
            };
            let isc = infer.scheme_to_infer(&sc);
            let inst = infer.instantiate_with(&isc, *span, name);
            let (arg_tys, ret_ty) = split_arrows(&inst, args.len());
            if let Err(e) = unify(&ret_ty, expected) {
                diag.push(unify_diag(e, *span, name));
            }
            for (sub_p, ty) in args.iter().zip(arg_tys.iter()) {
                bind_pattern(infer, env, sub_p, ty, diag);
            }
        }
        Pattern::List(items, _) => {
            let elem = infer.fresh();
            let list_ty = InferTy::app("List", vec![elem.clone()]);
            if let Err(e) = unify(&list_ty, expected) {
                diag.push(unify_diag(e, span_of_pattern(p), "[]"));
            }
            for item in items {
                bind_pattern(infer, env, item, &elem, diag);
            }
        }
        Pattern::Cons { head, tail, span } => {
            let elem = infer.fresh();
            let list_ty = InferTy::app("List", vec![elem.clone()]);
            if let Err(e) = unify(&list_ty, expected) {
                diag.push(unify_diag(e, *span, "::"));
            }
            bind_pattern(infer, env, head, &elem, diag);
            bind_pattern(infer, env, tail, &list_ty, diag);
        }
        Pattern::Record(names, span) => {
            let resolved = resolve(expected);
            let fields_opt: Option<Vec<(String, InferTy)>> = match &resolved {
                InferTy::Record(fs) => Some(fs.clone()),
                InferTy::Con(alias_name, _) => match infer.record_aliases.get(alias_name).cloned() {
                    Some(fs) => {
                        let mut out = Vec::new();
                        for (n, t) in &fs {
                            out.push((n.clone(), infer.type_to_infer_fresh(t)));
                        }
                        Some(out)
                    }
                    None => None,
                },
                InferTy::Var(_) => {
                    let fs: Vec<(String, InferTy)> =
                        names.iter().map(|n| (n.clone(), infer.fresh())).collect();
                    let r_ty = InferTy::Record(fs.clone());
                    if let Err(e) = unify(&r_ty, &resolved) {
                        diag.push(unify_diag(e, *span, "<record pattern>"));
                    }
                    Some(fs)
                }
                _ => None,
            };
            let fields = match fields_opt {
                Some(fs) => fs,
                None => {
                    diag.push(Diagnostic::NotARecord {
                        span: *span,
                        field: names.first().cloned().unwrap_or_default(),
                        ty: format!("{}", to_type_raw(&resolved)),
                    });
                    return;
                }
            };
            for n in names {
                match fields.iter().find(|(fname, _)| fname == n) {
                    Some((_, t)) => {
                        *env = env.extend(n, InferScheme::mono(t.clone()));
                    }
                    None => diag.push(Diagnostic::RecordNoField {
                        span: *span,
                        field: n.clone(),
                    }),
                }
            }
        }
        Pattern::As { inner, name, .. } => {
            *env = env.extend(name, InferScheme::mono(expected.clone()));
            bind_pattern(infer, env, inner, expected, diag);
        }
    }
}

fn span_of_pattern(p: &Pattern) -> Span {
    match p {
        Pattern::Var(_, s)
        | Pattern::Wildcard(s)
        | Pattern::Lit(_, s)
        | Pattern::Ctor { span: s, .. }
        | Pattern::List(_, s)
        | Pattern::Cons { span: s, .. }
        | Pattern::Record(_, s)
        | Pattern::As { span: s, .. } => *s,
    }
}

/// 関数型から先頭 n 個の引数を切り出す。残りは戻り値型。リンクは辿る。
fn split_arrows(t: &InferTy, n: usize) -> (Vec<InferTy>, InferTy) {
    let mut args = Vec::new();
    let mut cur = resolve(t);
    for _ in 0..n {
        match cur {
            InferTy::Arrow(f, to) => {
                args.push(resolve(&f));
                cur = resolve(&to);
            }
            other => {
                cur = other;
                break;
            }
        }
    }
    (args, cur)
}

/// 式の型推論。エラーは diag に積んで `None` を返す。単一化はセルを破壊的に更新する。
pub fn infer_expr(
    infer: &mut Infer,
    env: &TypeEnv,
    e: &Expr,
    diag: &mut Vec<Diagnostic>,
) -> Option<InferTy> {
    match e {
        Expr::Lit(l, _) => Some(lit_type(l)),
        Expr::Var { module, name, span } => {
            let key = qualify(module.as_ref(), name);
            match env.lookup(&key) {
                Some(sc) => {
                    let sc = sc.clone();
                    Some(infer.instantiate_with(&sc, *span, &key))
                }
                None => {
                    diag.push(Diagnostic::UndefinedVar {
                        span: *span,
                        name: key,
                    });
                    None
                }
            }
        }
        Expr::Ctor { module, name, span } => {
            let key = qualify(module.as_ref(), name);
            match env.lookup(&key) {
                Some(sc) => {
                    let sc = sc.clone();
                    Some(infer.instantiate_with(&sc, *span, &key))
                }
                None => {
                    diag.push(Diagnostic::UndefinedCtor {
                        span: *span,
                        name: key,
                    });
                    None
                }
            }
        }
        Expr::App { func, arg, span } => {
            let f_ty = infer_expr(infer, env, func, diag)?;
            let a_ty = infer_expr(infer, env, arg, diag)?;
            let result = infer.fresh();
            let expected = InferTy::arrow(a_ty, result.clone());
            match unify(&f_ty, &expected) {
                Ok(()) => Some(result),
                Err(e) => {
                    diag.push(unify_diag(e, *span, "<関数適用>"));
                    None
                }
            }
        }
        Expr::Lambda { params, body, .. } => {
            let mut local_env = env.clone();
            let mut param_tys: Vec<InferTy> = Vec::new();
            for p in params {
                let pt = infer.fresh();
                param_tys.push(pt.clone());
                bind_pattern(infer, &mut local_env, p, &pt, diag);
            }
            let body_ty = infer_expr(infer, &local_env, body, diag)?;
            let mut fun_ty = body_ty;
            for pt in param_tys.into_iter().rev() {
                fun_ty = InferTy::arrow(pt, fun_ty);
            }
            Some(fun_ty)
        }
        Expr::Let { bindings, body, .. } => {
            // 各 binding を順次推論し、generalize して env に追加 (非再帰 let)。
            let mut env_acc = env.clone();
            for b in bindings {
                infer.enter_level();
                let ty = infer_value_decl(infer, &env_acc, b, None, diag);
                infer.exit_level();
                let Some(ty) = ty else { continue };
                let isc = infer.generalize(&ty);
                env_acc = env_acc.extend(&b.name, isc);
            }
            infer_expr(infer, &env_acc, body, diag)
        }
        Expr::If {
            cond,
            then_branch,
            else_branch,
            span,
        } => {
            let c_ty = infer_expr(infer, env, cond, diag)?;
            if let Err(e) = unify(&c_ty, &InferTy::con("Bool")) {
                diag.push(unify_diag(e, *span, "<if 条件>"));
                return None;
            }
            let t_ty = infer_expr(infer, env, then_branch, diag)?;
            let e_ty = infer_expr(infer, env, else_branch, diag)?;
            if let Err(err) = unify(&t_ty, &e_ty) {
                diag.push(unify_diag(err, *span, "<if 分岐>"));
                return None;
            }
            Some(t_ty)
        }
        Expr::BinOp {
            op,
            left,
            right,
            span,
        } => {
            let (op_args, op_ret, constraints) = binop_type(*op, infer);
            for c in constraints {
                infer.pending.push(PendingConstraint {
                    constraint: c,
                    span: *span,
                    context: format!("<{op:?}>"),
                });
            }
            let l_ty = infer_expr(infer, env, left, diag)?;
            if let Err(err) = unify(&l_ty, &op_args.0) {
                diag.push(unify_diag(err, *span, &format!("<{op:?}>")));
                return None;
            }
            let r_ty = infer_expr(infer, env, right, diag)?;
            if let Err(err) = unify(&r_ty, &op_args.1) {
                diag.push(unify_diag(err, *span, &format!("<{op:?}>")));
                return None;
            }
            Some(op_ret)
        }
        Expr::Negate(inner, span) => {
            let ty = infer_expr(infer, env, inner, diag)?;
            infer.pending.push(PendingConstraint {
                constraint: InferConstraint {
                    class_name: "Num".to_string(),
                    ty: ty.clone(),
                },
                span: *span,
                context: "<negate>".to_string(),
            });
            Some(ty)
        }
        Expr::List(items, _) => {
            let elem = infer.fresh();
            for item in items {
                let it_ty = infer_expr(infer, env, item, diag)?;
                if let Err(err) = unify(&elem, &it_ty) {
                    diag.push(unify_diag(err, span_of_expr(item), "<list 要素>"));
                    return None;
                }
            }
            Some(InferTy::app("List", vec![elem]))
        }
        Expr::Range { lo, hi, span } => {
            let lo_ty = infer_expr(infer, env, lo, diag)?;
            let hi_ty = infer_expr(infer, env, hi, diag)?;
            if let Err(err) = unify(&lo_ty, &hi_ty) {
                diag.push(unify_diag(err, *span, "Range"));
                return None;
            }
            Some(InferTy::app("Range", vec![lo_ty]))
        }
        Expr::Record(fields, _) => {
            let mut fs: Vec<(String, InferTy)> = Vec::new();
            for f in fields {
                let ty = infer_expr(infer, env, &f.value, diag)?;
                fs.push((f.name.clone(), ty));
            }
            Some(InferTy::Record(fs))
        }
        Expr::Field {
            receiver,
            name,
            span,
        } => {
            let r_ty = infer_expr(infer, env, receiver, diag)?;
            match infer.lookup_field(&r_ty, name) {
                FieldLookup::Found(t) => Some(t),
                FieldLookup::NoField => {
                    diag.push(Diagnostic::RecordNoField {
                        span: *span,
                        field: name.clone(),
                    });
                    None
                }
                FieldLookup::NoFieldAlias(alias) => {
                    diag.push(Diagnostic::AliasNoField {
                        span: *span,
                        alias,
                        field: name.clone(),
                    });
                    None
                }
                // 受け手がまだ型変数。`map (\p -> p.x) xs` のように要素型が後で
                // 決まるケース。解決を SCC 境界に遅延する。
                FieldLookup::Unresolved => {
                    let tres = infer.fresh();
                    infer.pending_fields.push(FieldObligation {
                        receiver: r_ty,
                        field: name.clone(),
                        result: tres.clone(),
                        span: *span,
                    });
                    Some(tres)
                }
                FieldLookup::NotRecord => {
                    diag.push(Diagnostic::NotARecord {
                        span: *span,
                        field: name.clone(),
                        ty: format!("{}", to_type_raw(&r_ty)),
                    });
                    None
                }
            }
        }
        Expr::RecordUpdate {
            base,
            updates,
            span,
        } => {
            let b_ty = infer_expr(infer, env, base, diag)?;
            let resolved = resolve(&b_ty);
            // base が record literal の型か record alias の場合のみフィールド名を引ける。
            // 型変数の場合は updates から再構成。
            let fields_opt: Option<Vec<(String, InferTy)>> = match &resolved {
                InferTy::Record(fs) => Some(fs.clone()),
                InferTy::Con(alias_name, _) => {
                    match infer.record_aliases.get(alias_name).cloned() {
                        Some(fs) => {
                            let mut out = Vec::new();
                            for (n, t) in &fs {
                                out.push((n.clone(), infer.type_to_infer_fresh(t)));
                            }
                            Some(out)
                        }
                        None => None,
                    }
                }
                InferTy::Var(_) => None,
                _ => {
                    diag.push(Diagnostic::NotARecord {
                        span: *span,
                        field: updates.first().map(|f| f.name.clone()).unwrap_or_default(),
                        ty: format!("{}", to_type_raw(&resolved)),
                    });
                    return None;
                }
            };
            let mut update_tys: Vec<(String, InferTy, Span)> = Vec::new();
            for u in updates {
                let u_ty = match infer_expr(infer, env, &u.value, diag) {
                    Some(t) => t,
                    None => continue,
                };
                update_tys.push((u.name.clone(), u_ty, u.span));
            }
            let fields = match fields_opt {
                Some(fs) => fs,
                None => {
                    let fs: Vec<(String, InferTy)> = update_tys
                        .iter()
                        .map(|(n, _, _)| (n.clone(), infer.fresh()))
                        .collect();
                    let r_ty = InferTy::Record(fs.clone());
                    if let Err(err) = unify(&r_ty, &resolved) {
                        diag.push(unify_diag(err, *span, "<record update>"));
                        return None;
                    }
                    fs
                }
            };
            for (name, u_ty, u_span) in &update_tys {
                let Some((_, expected_ty)) = fields.iter().find(|(n, _)| n == name) else {
                    diag.push(Diagnostic::RecordNoField {
                        span: *u_span,
                        field: name.clone(),
                    });
                    continue;
                };
                if let Err(err) = unify(u_ty, expected_ty) {
                    diag.push(unify_diag(err, *u_span, &format!("<update {name}>")));
                    continue;
                }
            }
            Some(b_ty)
        }
        Expr::Case {
            scrutinee,
            arms,
            span,
        } => {
            let sc_ty = infer_expr(infer, env, scrutinee, diag)?;
            let result = infer.fresh();
            if arms.is_empty() {
                diag.push(Diagnostic::ParseErrorExpr { span: *span });
                return None;
            }
            for arm in arms {
                let mut arm_env = env.clone();
                bind_pattern(infer, &mut arm_env, &arm.pattern, &sc_ty, diag);
                if let Some(g) = &arm.guard {
                    let g_ty = match infer_expr(infer, &arm_env, g, diag) {
                        Some(t) => t,
                        None => continue,
                    };
                    if let Err(err) = unify(&g_ty, &InferTy::con("Bool")) {
                        diag.push(unify_diag(err, arm.span, "<case guard>"));
                        continue;
                    }
                }
                let b_ty = match infer_expr(infer, &arm_env, &arm.body, diag) {
                    Some(t) => t,
                    None => continue,
                };
                if let Err(err) = unify(&result, &b_ty) {
                    diag.push(unify_diag(err, arm.span, "<case 分岐>"));
                    continue;
                }
            }
            Some(result)
        }
        Expr::Error(span) => {
            diag.push(Diagnostic::ParseErrorExpr { span: *span });
            None
        }
    }
}

fn lit_type(l: &Lit) -> InferTy {
    match l {
        Lit::Int(_) => InferTy::con("Int"),
        Lit::Float(_) => InferTy::con("Float"),
        Lit::String(_) => InferTy::con("String"),
        Lit::Bool(_) => InferTy::con("Bool"),
    }
}

fn binop_type(
    op: BinOp,
    infer: &mut Infer,
) -> ((InferTy, InferTy), InferTy, Vec<InferConstraint>) {
    use BinOp::*;
    match op {
        // Num a => a -> a -> a
        Add | Sub | Mul | Div => {
            let v = infer.fresh();
            let c = InferConstraint {
                class_name: "Num".to_string(),
                ty: v.clone(),
            };
            ((v.clone(), v.clone()), v, vec![c])
        }
        // Eq a => a -> a -> Bool
        Eq | NotEq => {
            let v = infer.fresh();
            let c = InferConstraint {
                class_name: "Eq".to_string(),
                ty: v.clone(),
            };
            ((v.clone(), v.clone()), InferTy::con("Bool"), vec![c])
        }
        // Ord a => a -> a -> Bool
        Lt | Le | Gt | Ge => {
            let v = infer.fresh();
            let c = InferConstraint {
                class_name: "Ord".to_string(),
                ty: v.clone(),
            };
            ((v.clone(), v.clone()), InferTy::con("Bool"), vec![c])
        }
        And | Or => (
            (InferTy::con("Bool"), InferTy::con("Bool")),
            InferTy::con("Bool"),
            vec![],
        ),
        Cons => {
            let v = infer.fresh();
            (
                (v.clone(), InferTy::app("List", vec![v.clone()])),
                InferTy::app("List", vec![v]),
                vec![],
            )
        }
        Append => {
            let v = infer.fresh();
            (
                (
                    InferTy::app("List", vec![v.clone()]),
                    InferTy::app("List", vec![v.clone()]),
                ),
                InferTy::app("List", vec![v]),
                vec![],
            )
        }
        Compose | ComposeR => {
            let a = infer.fresh();
            let b = infer.fresh();
            let c = infer.fresh();
            (
                (
                    InferTy::arrow(a.clone(), b.clone()),
                    InferTy::arrow(b.clone(), c.clone()),
                ),
                InferTy::arrow(a, c),
                vec![],
            )
        }
        ApplyL => {
            let a = infer.fresh();
            let b = infer.fresh();
            ((InferTy::arrow(a.clone(), b.clone()), a), b, vec![])
        }
        ApplyR => {
            let a = infer.fresh();
            let b = infer.fresh();
            ((a.clone(), InferTy::arrow(a, b.clone())), b, vec![])
        }
    }
}

fn span_of_expr(e: &Expr) -> Span {
    match e {
        Expr::Var { span, .. }
        | Expr::Ctor { span, .. }
        | Expr::Lit(_, span)
        | Expr::List(_, span)
        | Expr::Record(_, span)
        | Expr::RecordUpdate { span, .. }
        | Expr::Field { span, .. }
        | Expr::App { span, .. }
        | Expr::Lambda { span, .. }
        | Expr::Let { span, .. }
        | Expr::If { span, .. }
        | Expr::Case { span, .. }
        | Expr::BinOp { span, .. }
        | Expr::Negate(_, span)
        | Expr::Range { span, .. }
        | Expr::Error(span) => *span,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::parse::parse;

    fn infer_src(src: &str) -> (HashMap<String, Scheme>, Vec<Diagnostic>) {
        let m = parse(src).unwrap();
        let r = crate::sema::builtin::registry();
        infer_module(&m, &r)
    }

    #[test]
    fn int_literal() {
        let (schemes, diag) = infer_src("x = 42");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["x"].ty.to_string(), "Int");
    }

    #[test]
    fn float_literal() {
        let (schemes, diag) = infer_src("x = 3.14");
        assert!(diag.is_empty());
        assert_eq!(schemes["x"].ty.to_string(), "Float");
    }

    #[test]
    fn arrow_function() {
        let (schemes, diag) = infer_src("f x = x + 1.0");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["f"].ty.to_string(), "Float -> Float");
    }

    #[test]
    fn signature_drives_param_type() {
        let (schemes, diag) = infer_src("f : Float -> Float\nf x = x");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["f"].ty.to_string(), "Float -> Float");
    }

    #[test]
    fn cube_application() {
        let (schemes, diag) = infer_src("shape = cube 10.0 10.0 10.0");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["shape"].ty.to_string(), "Shape3D");
    }

    #[test]
    fn type_mismatch_reports_diag() {
        let (_schemes, diag) = infer_src("f : Float -> Float\nf x = True");
        assert!(!diag.is_empty(), "型エラーが期待されたが diags は空でした");
    }

    #[test]
    fn let_polymorphism() {
        let (schemes, diag) = infer_src("f = let id x = x in id 1.0");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["f"].ty.to_string(), "Float");
    }

    #[test]
    fn range_inference() {
        let (schemes, diag) = infer_src("r = 1.0 .. 10.0");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["r"].ty.to_string(), "Range Float");
    }

    // --- Algorithm J 移行の回帰ガード ---
    // 旧 substitution-passing 実装は App / If / BinOp で「subst を env に適用してから
    // 次の部分式を推論する」手順を取りこぼしやすかった。in-place 単一化では共有セルが
    // 自動で更新されるため、これらの伝播は構造的に正しい。

    #[test]
    fn nested_application_determines_param_type() {
        // ((\g -> \x -> g (g x)) (\n -> n + 1.0)) 2.0 → Float。
        // g に渡した関数から param 型 (Float) が決まり、ネスト適用全体に伝播する。
        let (schemes, diag) = infer_src("f = (\\g -> \\x -> g (g x)) (\\n -> n + 1.0) 2.0");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["f"].ty.to_string(), "Float");
    }

    #[test]
    fn let_poly_used_at_two_types() {
        // let 束縛した id を Bool (条件) と Int (分岐) の両方で使う。
        let (schemes, diag) =
            infer_src("f = let id = \\x -> x in if id True then id 3 else id 4");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["f"].ty.to_string(), "Int");
    }

    #[test]
    fn if_branches_share_condition_refinement() {
        // cond で x : Bool と確定した後、then / else でも x は Bool として共有される。
        let (schemes, diag) = infer_src("g x = if x then x else x");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["g"].ty.to_string(), "Bool -> Bool");
    }

    #[test]
    fn derived_constraint_over_polymorphic_var_is_lifted() {
        // [x] == [x] → Eq (List a) → Eq a に分解され、a が generalize 対象なら
        // scheme の制約として持ち上げるべき。分解時に変数セルの同一性を失うと
        // 偽の AmbiguousConstraint になる (回帰防止)。
        let (schemes, diag) = infer_src("f x = [x] == [x]");
        assert!(diag.is_empty(), "diags: {diag:?}");
        assert_eq!(schemes["f"].constraints.len(), 1, "Eq 制約が 1 つ持ち上がるはず");
        assert_eq!(schemes["f"].constraints[0].class_name, "Eq");
        assert!(schemes["f"].ty.to_string().ends_with("-> Bool"));
    }
}
