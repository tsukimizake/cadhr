//! Algorithm W ベースの HM 型推論。
//!
//! 各 AST node に対して `infer_expr` で「推論結果型 + 累積置換」を返す。最終的に
//! トップレベル decl の集合に対して `infer_module` を呼び、各値定義の型を確定する。
//!
//! 多相は rank-1 (let-polymorphism + シグネチャ全称量化)。シグネチャ宣言と value
//! decl の name 一致で型を紐付ける。

use crate::diagnostic::{Diagnostic, Span};
use crate::sema::builtin::BuiltinRegistry;
use crate::sema::class::{ClassCheck, ClassRegistry, Constraint};
use crate::sema::env::TypeEnv;
use crate::sema::subst::{Subst, UnifyError, unify};
use crate::sema::ty::{Scheme, TyVar, TyVarGen, Type};
use crate::syntax::ast::*;
use crate::syntax::free_vars::value_decl_free_names;
use std::collections::{HashMap, HashSet};

#[derive(Debug, Default)]
pub struct Infer {
    pub var_gen: TyVarGen,
    /// シグネチャ宣言 (`f : T`) から得られた scheme。後続の value decl で参照する。
    pub signatures: HashMap<String, Scheme>,
    /// type alias 宣言。typecheck 時に展開する。
    pub aliases: HashMap<String, TypeAliasDecl>,
    /// ADT 型宣言。コンストラクタ → 関数型を引く。
    pub ctor_types: HashMap<String, Scheme>,
    /// record type alias の field マップ (record literal / update / field access で参照)。
    pub record_aliases: HashMap<String, Vec<(String, Type)>>,
    /// type class registry。標準クラス (Num/Ord/Eq) を持つ。
    pub class_registry: ClassRegistry,
    /// 推論中に蓄積した未解決制約。SCC / decl 単位で `solve_constraints` を
    /// 呼んで解消する。span は診断用。
    pub pending: Vec<PendingConstraint>,
}

#[derive(Clone, Debug)]
pub struct PendingConstraint {
    pub constraint: Constraint,
    pub span: Span,
    pub context: String,
}

impl Infer {
    pub fn new() -> Self {
        Self {
            class_registry: ClassRegistry::standard(),
            ..Self::default()
        }
    }

    /// fresh な単型変数を生成。
    pub fn fresh(&mut self) -> Type {
        Type::Var(self.var_gen.fresh())
    }

    /// Scheme を fresh 変数で instantiate する。`forall a b. a -> b` → `t0 -> t1`。
    /// 制約付き scheme の場合は、制約を rename して pending に積む。span/context は
    /// 「呼び出し元のどこで使われたか」を載せる。
    pub fn instantiate(&mut self, sc: &Scheme) -> Type {
        self.instantiate_with(sc, Span { start: 0, end: 0 }, "")
    }

    pub fn instantiate_with(&mut self, sc: &Scheme, span: Span, context: &str) -> Type {
        let mut s = Subst::empty();
        for v in &sc.vars {
            let fresh = self.fresh();
            s.insert(*v, fresh);
        }
        for c in &sc.constraints {
            let mapped = s.apply_constraint(c);
            self.pending.push(PendingConstraint {
                constraint: mapped,
                span,
                context: context.to_string(),
            });
        }
        s.apply(&sc.ty)
    }

    /// pending の各制約に subst を適用する。
    pub fn apply_subst_to_pending(&mut self, subst: &Subst) {
        for p in &mut self.pending {
            p.constraint = subst.apply_constraint(&p.constraint);
        }
    }

    /// 全 decl 推論を終えた時点で残った制約を処理する。
    /// - 具体型に解決された制約 → ClassCheck で Yes/No 判定 (Yes なら derived を解決)。
    /// - 型変数のまま残った制約 → `AmbiguousConstraint` エラー (Haskell の default
    ///   のような暗黙フォールバックは行わない: 呼び出し側で型注釈が必要)。
    pub fn finalize_pending(&mut self, diag: &mut Vec<Diagnostic>) {
        // 派生制約の連鎖を全部解消するまでループ。1 パスで処理できないのは、
        // Eq (List Int) → Eq Int のような派生が発生するため。
        loop {
            let pending = std::mem::take(&mut self.pending);
            if pending.is_empty() {
                return;
            }
            for p in pending {
                match self.class_registry.check(&p.constraint.class_name, &p.constraint.ty) {
                    ClassCheck::Yes { derived } => {
                        for d in derived {
                            self.pending.push(PendingConstraint {
                                constraint: d,
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
                            ty: format!("{}", p.constraint.ty),
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

    /// 環境 env における型 ty の generalize: env 自由でない型変数のみ全称化する。
    /// `pending` のうち、generalize 対象 var に絡む制約だけを scheme.constraints に
    /// 「持ち上げ」、それ以外は pending に残す。
    pub fn generalize(&mut self, env: &TypeEnv, ty: &Type) -> Scheme {
        let env_free = env.free_vars();
        let ty_free = ty.free_vars();
        let vars_set: HashSet<TyVar> = ty_free.difference(&env_free).copied().collect();
        let mut vars: Vec<TyVar> = vars_set.iter().copied().collect();
        vars.sort();
        let mut lifted: Vec<Constraint> = Vec::new();
        let mut keep: Vec<PendingConstraint> = Vec::new();
        for p in self.pending.drain(..) {
            // 制約に出てくる自由変数が「すべて generalize 対象に含まれる」場合
            // のみ scheme に持ち上げる (env に固定された var が混ざっていれば pending に残す)。
            let fvs = p.constraint.ty.free_vars();
            if fvs.is_empty() {
                // 具体型: ここで判定して終わり
                match self.class_registry.check(&p.constraint.class_name, &p.constraint.ty) {
                    ClassCheck::Yes { derived } => {
                        for d in derived {
                            keep.push(PendingConstraint {
                                constraint: d,
                                span: p.span,
                                context: p.context.clone(),
                            });
                        }
                    }
                    ClassCheck::No => {
                        // 具体型でインスタンスなし → 違反だが、ここでは collect だけ。
                        // 呼び出し側 (solve) で diag に変換するため pending に戻す。
                        keep.push(p);
                    }
                    ClassCheck::Unknown => unreachable!(),
                }
            } else if fvs.iter().all(|v| vars_set.contains(v)) {
                lifted.push(p.constraint);
            } else {
                keep.push(p);
            }
        }
        self.pending = keep;
        Scheme {
            vars,
            constraints: lifted,
            ty: ty.clone(),
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
                    // alias の params を仮 var に対応させて展開
                    let mut subst_map: HashMap<String, Type> = HashMap::new();
                    for (p, a) in alias.params.iter().zip(args_ty.iter()) {
                        subst_map.insert(p.clone(), a.clone());
                    }
                    let mut alias_var_map: HashMap<String, TyVar> = HashMap::new();
                    let raw = self.ty_of_type_expr(&alias.body, &mut alias_var_map, diag);
                    return substitute_named(&raw, &subst_map, &alias_var_map);
                }
                // Point2D / Point3D は組み込みの構造的 record 型に解決する
                // (sema::builtin の point2d / point3d と同一形)。
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

/// `TypeExpr` の span を取り出す helper。
fn span_of_type_expr(t: &TypeExpr) -> Span {
    match t {
        TypeExpr::Var(_, s)
        | TypeExpr::Con { span: s, .. }
        | TypeExpr::Arrow { span: s, .. }
        | TypeExpr::Record(_, s) => *s,
    }
}

/// 仮実装: 名前ベースの type variable を `subst_map` で展開する。
fn substitute_named(
    ty: &Type,
    subst_map: &HashMap<String, Type>,
    var_map: &HashMap<String, TyVar>,
) -> Type {
    match ty {
        Type::Var(v) => {
            // var_map 経由で名前を引いて、subst_map から取る
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

    // import 解決時に「公開する local 名 → Scheme」が必要。順を追って構築する。
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
        env = env.extend(b.name, b.scheme.clone());
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
            env = env.extend(&qualify_str(&prefix, k), sc.clone());
        }
        match &imp.exposing {
            Some(Exposing::All(_)) => {
                for (k, sc) in imp_schemes {
                    env = env.extend(k, sc.clone());
                }
            }
            Some(Exposing::Some(items, _)) => {
                for item in items {
                    if let Some(sc) = imp_schemes.get(&item.name) {
                        env = env.extend(&item.name, sc.clone());
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

/// `infer_module` の中身を抜き出した本体。`env` には builtins と import 由来の
/// scheme をあらかじめ詰めて渡す。
fn infer_module_into(
    module: &Module,
    infer: &mut Infer,
    env_in: &mut TypeEnv,
) -> (HashMap<String, Scheme>, Vec<Diagnostic>) {
    let mut diag: Vec<Diagnostic> = Vec::new();
    let mut env = env_in.clone();

    // 2. type alias / type decl / signature を先に収集
    let mut sig_var_maps: HashMap<String, HashMap<String, TyVar>> = HashMap::new();
    for decl in &module.decls {
        match decl {
            Decl::TypeAlias(a) => {
                infer.aliases.insert(a.name.clone(), a.clone());
                // record alias なら record_aliases に登録
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
                // 各コンストラクタを「args ->...-> T params」型として登録
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
                    // param_vars はすべて全称化
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
                // ダミー var を fresh に置き換える
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
                    vars: vars.clone(),
                    constraints: Vec::new(),
                    ty: final_ty,
                };
                infer.signatures.insert(s.name.clone(), scheme);
                sig_var_maps.insert(s.name.clone(), name_to_fresh);
            }
            _ => {}
        }
    }

    // 3. コンストラクタを環境に展開
    for (name, sc) in &infer.ctor_types {
        env = env.extend(name, sc.clone());
    }
    // signature も環境に (再帰関数を許す: 後で sig を持つ name を再帰で呼ぶため)
    for (name, sc) in &infer.signatures {
        env = env.extend(name, sc.clone());
    }

    // 4. value decl の依存解析 → SCC → 葉から順に推論
    // これにより signature が無い decl でも前方参照・相互再帰が動き、
    // SCC 内では monotype 共有 → 終了後に generalize するので let-poly が効く。
    let value_decls: Vec<&ValueDecl> = module
        .decls
        .iter()
        .filter_map(|d| match d {
            Decl::Value(v) => Some(v),
            _ => None,
        })
        .collect();
    let decl_names: HashSet<String> = value_decls.iter().map(|v| v.name.clone()).collect();

    // 依存グラフ: name -> body が参照する同モジュール内 value decl 名
    let mut graph: HashMap<String, Vec<String>> = HashMap::new();
    for v in &value_decls {
        let used = value_decl_free_names(v);
        let mut deps: Vec<String> = used.into_iter().filter(|n| decl_names.contains(n)).collect();
        deps.sort(); // 決定的な順序
        graph.insert(v.name.clone(), deps);
    }

    let sccs = tarjan_scc(&value_decls, &graph);

    let mut result: HashMap<String, Scheme> = HashMap::new();
    for scc in sccs {
        infer_value_scc(infer, &mut env, &scc, &graph, &mut diag, &mut result);
    }

    // 全 SCC を処理し終わった時点で、まだ未解決の制約をすべて判定する。
    // 型変数のまま残った制約はエラー (型注釈での解消を要求)。
    infer.finalize_pending(&mut diag);

    *env_in = env;
    (result, diag)
}

/// 1 つの SCC を一括で推論する。SCC 内は相互参照可能なので、各メンバを fresh
/// monotype var (sig 無し) または instantiate された sig 型 (sig 有り) で
/// 一時 env に bind し、全 body を推論してから unify で整合させる。
/// 終了後に sig 無しメンバを (SCC 進入前の env に対して) generalize する。
fn infer_value_scc(
    infer: &mut Infer,
    env: &mut TypeEnv,
    scc: &[&ValueDecl],
    graph: &HashMap<String, Vec<String>>,
    diag: &mut Vec<Diagnostic>,
    result: &mut HashMap<String, Scheme>,
) {
    // generalize 時の参照環境は SCC 進入前のもの (SCC メンバ自身を含まない)
    let env_before = env.clone();

    // 各メンバについて: sig 有なら expected = instantiate(sig)、sig 無なら fresh var。
    // どちらも SCC 内の相互参照に使える monotype として local_env に入れる。
    let mut local_env = env.clone();
    let mut member_expected: HashMap<String, Type> = HashMap::new();
    for m in scc {
        let expected = if let Some(sc) = infer.signatures.get(&m.name).cloned() {
            let inst = infer.instantiate_with(&sc, m.span, &m.name);
            // sig 有りは scheme として bind 済みなので、SCC 内では再 instantiate
            // した monotype を別途持っておけば十分 (env 側は polymorphic のまま)。
            inst
        } else {
            let fresh = infer.fresh();
            local_env = local_env.extend(&m.name, Scheme::mono(fresh.clone()));
            fresh
        };
        member_expected.insert(m.name.clone(), expected);
    }

    // 各 body を推論し、累積 subst を作る。
    let mut subst = Subst::empty();
    for m in scc {
        let expected = subst.apply(member_expected.get(&m.name).unwrap());
        let expected_arg = if infer.signatures.contains_key(&m.name) {
            Some(expected.clone())
        } else {
            None
        };
        // local_env に subst を反映 (sig 無しメンバの fresh var を更新)
        let env_for_body = apply_subst_to_env(&local_env, &subst);
        let (ty, s) = match infer_value_decl(
            infer,
            &env_for_body,
            m,
            expected_arg.as_ref(),
            diag,
        ) {
            Some(r) => r,
            None => continue,
        };
        let composed = subst.compose(&s);
        let inferred = composed.apply(&ty);
        let expected_now = composed.apply(&expected);
        match unify(&inferred, &expected_now) {
            Ok(s2) => {
                subst = composed.compose(&s2);
            }
            Err(e) => {
                diag.push(unify_diag(e, m.span, &m.name));
                subst = composed;
            }
        }
    }

    // SCC 全体の body 推論が終わったら、subst を pending に適用して solve を試みる。
    infer.apply_subst_to_pending(&subst);
    solve_constraints(infer, diag);

    // finalize: sig 有 → declared scheme、sig 無 → 解決済み型を env_before で generalize
    for m in scc {
        let resolved = subst.apply(member_expected.get(&m.name).unwrap());
        let scheme = if let Some(declared) = infer.signatures.get(&m.name).cloned() {
            Scheme {
                vars: declared.vars.clone(),
                constraints: declared
                    .constraints
                    .iter()
                    .map(|c| subst.apply_constraint(c))
                    .collect(),
                ty: resolved,
            }
        } else {
            infer.generalize(&env_before, &resolved)
        };
        *env = env.extend(&m.name, scheme.clone());
        result.insert(m.name.clone(), scheme);
    }

    let _ = graph; // 将来 SCC 内自己再帰判定に使うかも
}

/// 環境内の各 scheme に置換を適用した新しい環境を返す。
/// scheme の bound vars に subst が触ることは無い前提 (普通は monotype しか subst しない)。
/// pending の制約を進める。
/// - 具体型 → ClassCheck::Yes/No に従って derived を pending に / 違反を diag に。
/// - 型変数のまま → pending に残す (後で finalize_pending か generalize で処理)。
fn solve_constraints(infer: &mut Infer, diag: &mut Vec<Diagnostic>) {
    let mut keep: Vec<PendingConstraint> = Vec::new();
    let pending = std::mem::take(&mut infer.pending);
    for p in pending {
        match infer.class_registry.check(&p.constraint.class_name, &p.constraint.ty) {
            ClassCheck::Yes { derived } => {
                for d in derived {
                    keep.push(PendingConstraint {
                        constraint: d,
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
                    ty: format!("{}", p.constraint.ty),
                });
            }
            ClassCheck::Unknown => {
                keep.push(p);
            }
        }
    }
    infer.pending = keep;
}

fn apply_subst_to_env(env: &TypeEnv, subst: &Subst) -> TypeEnv {
    let mut new = TypeEnv::new();
    for (name, sc) in env.iter() {
        let new_sc = Scheme {
            vars: sc.vars.clone(),
            constraints: sc
                .constraints
                .iter()
                .map(|c| subst.apply_constraint(c))
                .collect(),
            ty: subst.apply(&sc.ty),
        };
        new = new.extend(name, new_sc);
    }
    new
}

/// Tarjan's SCC algorithm。返値はSCCのリスト (各 SCC 内のメンバ順は不定)、
/// **逆 topological 順** (依存グラフの葉から根の順) で返るので、そのまま
/// for-each すれば依存先が先に処理される。
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
        env = env.extend(b.name, b.scheme.clone());
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
            left: format!("{left}"),
            right: format!("{right}"),
        },
        UnifyError::InfiniteType { var: _, ty } => Diagnostic::InfiniteType {
            span,
            context,
            ty: format!("{ty}"),
        },
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
    expected: Option<&Type>,
    diag: &mut Vec<Diagnostic>,
) -> Option<(Type, Subst)> {
    // signature があれば param 型を body 推論前に確定させる (record の field access 等、
    // param 型が分かっていないと推論できない式を body 内で許すため)。無ければ fresh。
    let sig_param_tys = expected
        .map(|t| split_arrows(t, v.params.len()).0)
        .unwrap_or_default();
    let mut local_env = env.clone();
    let mut param_tys: Vec<Type> = Vec::new();
    let mut param_subst = Subst::empty();
    for (i, p) in v.params.iter().enumerate() {
        let pt = sig_param_tys
            .get(i)
            .cloned()
            .unwrap_or_else(|| infer.fresh());
        param_tys.push(pt.clone());
        let s = bind_pattern(infer, &mut local_env, p, &pt, diag);
        param_subst = param_subst.compose(&s);
    }
    let local_env = apply_subst_to_env(&local_env, &param_subst);
    let (body_ty, subst) = infer_expr(infer, &local_env, &v.body, diag)?;
    let subst = param_subst.compose(&subst);
    let mut fun_ty = body_ty;
    for pt in param_tys.into_iter().rev() {
        fun_ty = Type::arrow(subst.apply(&pt), fun_ty);
    }
    Some((fun_ty, subst))
}

/// pattern を env に束縛する。返り値は内部の unify で得た累積 Subst。
/// 呼び出し側はこれを自分の Subst に compose する必要がある (case のように
/// `expected` が型変数の場合、その解決を伝播させないと scrutinee 側の型が
/// 取り残される)。
fn bind_pattern(
    infer: &mut Infer,
    env: &mut TypeEnv,
    p: &Pattern,
    expected: &Type,
    diag: &mut Vec<Diagnostic>,
) -> Subst {
    let mut subst = Subst::empty();
    match p {
        Pattern::Var(name, _) => {
            *env = env.extend(name, Scheme::mono(expected.clone()));
        }
        Pattern::Wildcard(_) => {}
        Pattern::Lit(lit, span) => {
            // リテラルパターンは expected が同型でなければエラー。
            let lt = lit_type(lit);
            match unify(&subst.apply(expected), &lt) {
                Ok(s) => subst = subst.compose(&s),
                Err(e) => diag.push(unify_diag(e, *span, "<literal pattern>")),
            }
        }
        Pattern::Ctor {
            name, args, span, ..
        } => {
            // コンストラクタの型を instantiate して、引数型と patterns を再帰束縛
            let sc = match infer.ctor_types.get(name).cloned() {
                Some(s) => s,
                None => {
                    diag.push(Diagnostic::CtorUndefinedInPattern {
                        span: *span,
                        name: name.clone(),
                    });
                    return subst;
                }
            };
            let inst = infer.instantiate_with(&sc, *span, name);
            // inst は a1 -> ... -> an -> RetTy の curried 形式
            let (arg_tys, ret_ty) = split_arrows(&inst, args.len());
            // ret_ty を expected と unify
            match unify(&subst.apply(&ret_ty), &subst.apply(expected)) {
                Ok(s) => subst = subst.compose(&s),
                Err(e) => diag.push(unify_diag(e, *span, name)),
            }
            for (sub_p, ty) in args.iter().zip(arg_tys.iter()) {
                let resolved = subst.apply(ty);
                let s = bind_pattern(infer, env, sub_p, &resolved, diag);
                subst = subst.compose(&s);
            }
        }
        Pattern::List(items, _) => {
            // expected は List elem。expected と List a を unify
            let elem = infer.fresh();
            let list_ty = Type::app("List", vec![elem.clone()]);
            match unify(&list_ty, &subst.apply(expected)) {
                Ok(s) => subst = subst.compose(&s),
                Err(e) => diag.push(unify_diag(e, span_of_pattern(p), "[]")),
            }
            for item in items {
                let resolved = subst.apply(&elem);
                let s = bind_pattern(infer, env, item, &resolved, diag);
                subst = subst.compose(&s);
            }
        }
        Pattern::Cons { head, tail, span } => {
            let elem = infer.fresh();
            let list_ty = Type::app("List", vec![elem.clone()]);
            match unify(&list_ty, &subst.apply(expected)) {
                Ok(s) => subst = subst.compose(&s),
                Err(e) => diag.push(unify_diag(e, *span, "::")),
            }
            let head_ty = subst.apply(&elem);
            let s = bind_pattern(infer, env, head, &head_ty, diag);
            subst = subst.compose(&s);
            let tail_ty = subst.apply(&list_ty);
            let s = bind_pattern(infer, env, tail, &tail_ty, diag);
            subst = subst.compose(&s);
        }
        Pattern::Record(names, span) => {
            // 各 name に対応するフィールド型を expected (record / record alias / 型変数)
            // から引き出して env に束縛する。
            let resolved = subst.apply(expected);
            let fields_opt: Option<Vec<(String, Type)>> = match &resolved {
                Type::Record(fs) => Some(fs.clone()),
                Type::Con(alias_name, _) => infer.record_aliases.get(alias_name).cloned(),
                Type::Var(_) => {
                    // 各 name に fresh 型を割当てて record として unify する。
                    let fs: Vec<(String, Type)> =
                        names.iter().map(|n| (n.clone(), infer.fresh())).collect();
                    let r_ty = Type::Record(fs.clone());
                    match unify(&r_ty, &resolved) {
                        Ok(s) => subst = subst.compose(&s),
                        Err(e) => diag.push(unify_diag(e, *span, "<record pattern>")),
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
                        ty: format!("{resolved}"),
                    });
                    return subst;
                }
            };
            for n in names {
                match fields.iter().find(|(fname, _)| fname == n) {
                    Some((_, t)) => {
                        let bound = subst.apply(t);
                        *env = env.extend(n, Scheme::mono(bound));
                    }
                    None => diag.push(Diagnostic::RecordNoField {
                        span: *span,
                        field: n.clone(),
                    }),
                }
            }
        }
        Pattern::As { inner, name, .. } => {
            *env = env.extend(name, Scheme::mono(expected.clone()));
            let s = bind_pattern(infer, env, inner, expected, diag);
            subst = subst.compose(&s);
        }
    }
    subst
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

/// 関数型から先頭 n 個の引数を切り出す。残りは戻り値型。
fn split_arrows(t: &Type, n: usize) -> (Vec<Type>, Type) {
    let mut args = Vec::new();
    let mut cur = t.clone();
    for _ in 0..n {
        match cur {
            Type::Arrow(f, to) => {
                args.push(*f);
                cur = *to;
            }
            _ => break,
        }
    }
    (args, cur)
}

/// 式の型推論。返り値は (型, 累積置換)。エラーは diag に積んで `None` を返す。
pub fn infer_expr(
    infer: &mut Infer,
    env: &TypeEnv,
    e: &Expr,
    diag: &mut Vec<Diagnostic>,
) -> Option<(Type, Subst)> {
    match e {
        Expr::Lit(l, _) => Some((lit_type(l), Subst::empty())),
        Expr::Var { module, name, span } => {
            let key = qualify(module.as_ref(), name);
            match env.lookup(&key) {
                Some(sc) => Some((infer.instantiate_with(sc, *span, &key), Subst::empty())),
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
                Some(sc) => Some((infer.instantiate_with(sc, *span, &key), Subst::empty())),
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
            let (f_ty, s1) = infer_expr(infer, env, func, diag)?;
            let env2 = env.clone();
            let (a_ty, s2) = infer_expr(infer, &env2, arg, diag)?;
            let result = infer.fresh();
            let expected = Type::arrow(a_ty.clone(), result.clone());
            let f_ty2 = s2.apply(&f_ty);
            match unify(&f_ty2, &expected) {
                Ok(s3) => {
                    let s_all = s1.compose(&s2).compose(&s3);
                    Some((s_all.apply(&result), s_all))
                }
                Err(e) => {
                    diag.push(unify_diag(e, *span, "<関数適用>"));
                    None
                }
            }
        }
        Expr::Lambda { params, body, .. } => {
            let mut local_env = env.clone();
            let mut param_tys: Vec<Type> = Vec::new();
            let mut pat_subst = Subst::empty();
            for p in params {
                let pt = infer.fresh();
                param_tys.push(pt.clone());
                let s = bind_pattern(infer, &mut local_env, p, &pt, diag);
                pat_subst = pat_subst.compose(&s);
            }
            let local_env = apply_subst_to_env(&local_env, &pat_subst);
            let (body_ty, s) = infer_expr(infer, &local_env, body, diag)?;
            let s = pat_subst.compose(&s);
            let mut fun_ty = body_ty;
            for pt in param_tys.into_iter().rev() {
                fun_ty = Type::arrow(s.apply(&pt), fun_ty);
            }
            Some((fun_ty, s))
        }
        Expr::Let { bindings, body, .. } => {
            // 各 binding を順次推論し、env に generalize して追加
            let mut env_acc = env.clone();
            let mut subst_acc = Subst::empty();
            for b in bindings {
                let (ty, s) = match infer_value_decl(infer, &env_acc, b, None, diag) {
                    Some(r) => r,
                    None => continue,
                };
                let final_ty = s.apply(&ty);
                let sc = infer.generalize(&env_acc, &final_ty);
                env_acc = env_acc.extend(&b.name, sc);
                subst_acc = subst_acc.compose(&s);
            }
            let (body_ty, s_body) = infer_expr(infer, &env_acc, body, diag)?;
            Some((body_ty, subst_acc.compose(&s_body)))
        }
        Expr::If {
            cond,
            then_branch,
            else_branch,
            span,
        } => {
            let (c_ty, s1) = infer_expr(infer, env, cond, diag)?;
            let s_c = match unify(&c_ty, &Type::con("Bool")) {
                Ok(s) => s,
                Err(e) => {
                    diag.push(unify_diag(e, *span, "<if 条件>"));
                    return None;
                }
            };
            let s = s1.compose(&s_c);
            let env2 = env.clone();
            let (t_ty, s2) = infer_expr(infer, &env2, then_branch, diag)?;
            let s = s.compose(&s2);
            let (e_ty, s3) = infer_expr(infer, &env2, else_branch, diag)?;
            let s = s.compose(&s3);
            let s_branch = match unify(&s.apply(&t_ty), &s.apply(&e_ty)) {
                Ok(s) => s,
                Err(err) => {
                    diag.push(unify_diag(err, *span, "<if 分岐>"));
                    return None;
                }
            };
            let s = s.compose(&s_branch);
            Some((s.apply(&t_ty), s))
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
            let (l_ty, s1) = infer_expr(infer, env, left, diag)?;
            let s_l = match unify(&l_ty, &op_args.0) {
                Ok(s) => s,
                Err(err) => {
                    diag.push(unify_diag(err, *span, &format!("<{op:?}>")));
                    return None;
                }
            };
            let s = s1.compose(&s_l);
            let (r_ty, s2) = infer_expr(infer, env, right, diag)?;
            let s = s.compose(&s2);
            let s_r = match unify(&s.apply(&r_ty), &s.apply(&op_args.1)) {
                Ok(s) => s,
                Err(err) => {
                    diag.push(unify_diag(err, *span, &format!("<{op:?}>")));
                    return None;
                }
            };
            let s = s.compose(&s_r);
            Some((s.apply(&op_ret), s))
        }
        Expr::Negate(inner, span) => {
            // Num a => a -> a
            let (ty, s) = infer_expr(infer, env, inner, diag)?;
            infer.pending.push(PendingConstraint {
                constraint: Constraint {
                    class_name: "Num".to_string(),
                    ty: s.apply(&ty),
                },
                span: *span,
                context: "<negate>".to_string(),
            });
            Some((s.apply(&ty), s))
        }
        Expr::List(items, _) => {
            let elem = infer.fresh();
            let mut s = Subst::empty();
            for item in items {
                let (it_ty, s_it) = infer_expr(infer, env, item, diag)?;
                s = s.compose(&s_it);
                match unify(&s.apply(&elem), &s.apply(&it_ty)) {
                    Ok(s2) => s = s.compose(&s2),
                    Err(err) => {
                        diag.push(unify_diag(err, span_of_expr(item), "<list 要素>"));
                        return None;
                    }
                }
            }
            Some((Type::app("List", vec![s.apply(&elem)]), s))
        }
        Expr::Range { lo, hi, span } => {
            let (lo_ty, s1) = infer_expr(infer, env, lo, diag)?;
            let (hi_ty, s2) = infer_expr(infer, env, hi, diag)?;
            let s = s1.compose(&s2);
            let s_eq = match unify(&s.apply(&lo_ty), &s.apply(&hi_ty)) {
                Ok(s) => s,
                Err(err) => {
                    diag.push(unify_diag(err, *span, "Range"));
                    return None;
                }
            };
            let s = s.compose(&s_eq);
            Some((Type::app("Range", vec![s.apply(&lo_ty)]), s))
        }
        Expr::Record(fields, _) => {
            let mut subst = Subst::empty();
            let mut fs: Vec<(String, Type)> = Vec::new();
            for f in fields {
                let (ty, s) = infer_expr(infer, env, &f.value, diag)?;
                subst = subst.compose(&s);
                fs.push((f.name.clone(), ty));
            }
            // record literal は anonymous record 型として推論する。
            Some((Type::Record(fs), subst))
        }
        Expr::Field {
            receiver,
            name,
            span,
        } => {
            let (r_ty, s) = infer_expr(infer, env, receiver, diag)?;
            let r_ty = s.apply(&r_ty);
            // r_ty が Record なら field 名で引く
            match &r_ty {
                Type::Record(fs) => match fs.iter().find(|(n, _)| n == name) {
                    Some((_, t)) => Some((t.clone(), s)),
                    None => {
                        diag.push(Diagnostic::RecordNoField {
                            span: *span,
                            field: name.clone(),
                        });
                        None
                    }
                },
                Type::Con(alias_name, _) if infer.record_aliases.contains_key(alias_name) => {
                    let fields = infer.record_aliases.get(alias_name).unwrap().clone();
                    match fields.iter().find(|(n, _)| n == name) {
                        Some((_, t)) => Some((t.clone(), s)),
                        None => {
                            diag.push(Diagnostic::AliasNoField {
                                span: *span,
                                alias: alias_name.clone(),
                                field: name.clone(),
                            });
                            None
                        }
                    }
                }
                _ => {
                    diag.push(Diagnostic::NotARecord {
                        span: *span,
                        field: name.clone(),
                        ty: format!("{r_ty}"),
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
            let (b_ty, s_base) = infer_expr(infer, env, base, diag)?;
            let mut s = s_base;
            let resolved = s.apply(&b_ty);
            // base が record literal の型か record alias の場合のみフィールド名を引ける。
            // 型変数の場合は updates から再構成。
            let fields_opt: Option<Vec<(String, Type)>> = match &resolved {
                Type::Record(fs) => Some(fs.clone()),
                Type::Con(alias_name, _) => infer.record_aliases.get(alias_name).cloned(),
                Type::Var(_) => None, // 後で fresh record で unify する
                _ => {
                    diag.push(Diagnostic::NotARecord {
                        span: *span,
                        field: updates
                            .first()
                            .map(|f| f.name.clone())
                            .unwrap_or_default(),
                        ty: format!("{resolved}"),
                    });
                    return None;
                }
            };
            // updates 側の field 値を推論
            let mut update_tys: Vec<(String, Type, Span)> = Vec::new();
            for u in updates {
                let (u_ty, us) = match infer_expr(infer, env, &u.value, diag) {
                    Some(r) => r,
                    None => continue,
                };
                s = s.compose(&us);
                update_tys.push((u.name.clone(), u_ty, u.span));
            }
            let fields = match fields_opt {
                Some(fs) => fs,
                None => {
                    // type var の場合: updates の field と fresh で record を組んで unify。
                    // 不足フィールドは検査不可なので無視 (rank-1 HM の素朴な実装)。
                    let fs: Vec<(String, Type)> = update_tys
                        .iter()
                        .map(|(n, _, _)| (n.clone(), infer.fresh()))
                        .collect();
                    let r_ty = Type::Record(fs.clone());
                    match unify(&r_ty, &s.apply(&resolved)) {
                        Ok(s2) => s = s.compose(&s2),
                        Err(err) => {
                            diag.push(unify_diag(err, *span, "<record update>"));
                            return None;
                        }
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
                let s_u = match unify(&s.apply(u_ty), &s.apply(expected_ty)) {
                    Ok(s2) => s2,
                    Err(err) => {
                        diag.push(unify_diag(err, *u_span, &format!("<update {name}>")));
                        continue;
                    }
                };
                s = s.compose(&s_u);
            }
            Some((s.apply(&b_ty), s))
        }
        Expr::Case {
            scrutinee,
            arms,
            span,
        } => {
            let (sc_ty, s_sc) = infer_expr(infer, env, scrutinee, diag)?;
            let mut s = s_sc;
            let result = infer.fresh();
            if arms.is_empty() {
                diag.push(Diagnostic::ParseErrorExpr { span: *span });
                return None;
            }
            for arm in arms {
                let mut arm_env = apply_subst_to_env(env, &s);
                let sc_ty_now = s.apply(&sc_ty);
                let s_pat = bind_pattern(infer, &mut arm_env, &arm.pattern, &sc_ty_now, diag);
                s = s.compose(&s_pat);
                let arm_env = apply_subst_to_env(&arm_env, &s);
                if let Some(g) = &arm.guard {
                    let (g_ty, gs) = match infer_expr(infer, &arm_env, g, diag) {
                        Some(r) => r,
                        None => continue,
                    };
                    s = s.compose(&gs);
                    match unify(&s.apply(&g_ty), &Type::con("Bool")) {
                        Ok(s2) => s = s.compose(&s2),
                        Err(err) => {
                            diag.push(unify_diag(err, arm.span, "<case guard>"));
                            continue;
                        }
                    }
                }
                let arm_env = apply_subst_to_env(&arm_env, &s);
                let (b_ty, bs) = match infer_expr(infer, &arm_env, &arm.body, diag) {
                    Some(r) => r,
                    None => continue,
                };
                s = s.compose(&bs);
                match unify(&s.apply(&result), &s.apply(&b_ty)) {
                    Ok(s2) => s = s.compose(&s2),
                    Err(err) => {
                        diag.push(unify_diag(err, arm.span, "<case 分岐>"));
                        continue;
                    }
                }
            }
            Some((s.apply(&result), s))
        }
        Expr::Error(span) => {
            diag.push(Diagnostic::ParseErrorExpr { span: *span });
            None
        }
    }
}

fn lit_type(l: &Lit) -> Type {
    match l {
        Lit::Int(_) => Type::con("Int"),
        Lit::Float(_) => Type::con("Float"),
        Lit::String(_) => Type::con("String"),
        Lit::Bool(_) => Type::con("Bool"),
    }
}

fn binop_type(op: BinOp, infer: &mut Infer) -> ((Type, Type), Type, Vec<Constraint>) {
    use BinOp::*;
    match op {
        // Num a => a -> a -> a
        Add | Sub | Mul | Div => {
            let v = infer.fresh();
            let c = Constraint {
                class_name: "Num".to_string(),
                ty: v.clone(),
            };
            ((v.clone(), v.clone()), v, vec![c])
        }
        // Eq a => a -> a -> Bool
        Eq | NotEq => {
            let v = infer.fresh();
            let c = Constraint {
                class_name: "Eq".to_string(),
                ty: v.clone(),
            };
            ((v.clone(), v.clone()), Type::con("Bool"), vec![c])
        }
        // Ord a => a -> a -> Bool
        Lt | Le | Gt | Ge => {
            let v = infer.fresh();
            let c = Constraint {
                class_name: "Ord".to_string(),
                ty: v.clone(),
            };
            ((v.clone(), v.clone()), Type::con("Bool"), vec![c])
        }
        And | Or => (
            (Type::con("Bool"), Type::con("Bool")),
            Type::con("Bool"),
            vec![],
        ),
        Cons => {
            let v = infer.fresh();
            (
                (v.clone(), Type::app("List", vec![v.clone()])),
                Type::app("List", vec![v]),
                vec![],
            )
        }
        Append => {
            let v = infer.fresh();
            (
                (
                    Type::app("List", vec![v.clone()]),
                    Type::app("List", vec![v.clone()]),
                ),
                Type::app("List", vec![v]),
                vec![],
            )
        }
        Compose | ComposeR => {
            let a = infer.fresh();
            let b = infer.fresh();
            let c = infer.fresh();
            (
                (
                    Type::arrow(a.clone(), b.clone()),
                    Type::arrow(b.clone(), c.clone()),
                ),
                Type::arrow(a, c),
                vec![],
            )
        }
        ApplyL => {
            // f <| x : (a -> b) <| a → b
            let a = infer.fresh();
            let b = infer.fresh();
            ((Type::arrow(a.clone(), b.clone()), a), b, vec![])
        }
        ApplyR => {
            // x |> f : a |> (a -> b) → b
            let a = infer.fresh();
            let b = infer.fresh();
            ((a.clone(), Type::arrow(a, b.clone())), b, vec![])
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
        // id を let で導入し、2 つの型で使う
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
}
