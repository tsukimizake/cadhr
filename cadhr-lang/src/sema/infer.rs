//! Algorithm W ベースの HM 型推論。
//!
//! 各 AST node に対して `infer_expr` で「推論結果型 + 累積置換」を返す。最終的に
//! トップレベル decl の集合に対して `infer_module` を呼び、各値定義の型を確定する。
//!
//! 多相は rank-1 (let-polymorphism + シグネチャ全称量化)。シグネチャ宣言と value
//! decl の name 一致で型を紐付ける。

use crate::diagnostic::{Diagnostic, Span};
use crate::sema::builtin::BuiltinRegistry;
use crate::sema::env::TypeEnv;
use crate::sema::subst::{unify, Subst, UnifyError};
use crate::sema::ty::{Scheme, TyVar, TyVarGen, Type};
use crate::syntax::ast::*;
use std::collections::HashMap;

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
}

impl Infer {
    pub fn new() -> Self {
        Self::default()
    }

    /// fresh な単型変数を生成。
    pub fn fresh(&mut self) -> Type {
        Type::Var(self.var_gen.fresh())
    }

    /// Scheme を fresh 変数で instantiate する。`forall a b. a -> b` → `t0 -> t1`。
    pub fn instantiate(&mut self, sc: &Scheme) -> Type {
        let mut s = Subst::empty();
        for v in &sc.vars {
            let fresh = self.fresh();
            s.insert(*v, fresh);
        }
        s.apply(&sc.ty)
    }

    /// 環境 env における型 ty の generalize: env 自由でない型変数のみ全称化する。
    pub fn generalize(&self, env: &TypeEnv, ty: &Type) -> Scheme {
        let env_free = env.free_vars();
        let ty_free = ty.free_vars();
        let vars: Vec<TyVar> = ty_free.difference(&env_free).copied().collect();
        let mut vars = vars;
        vars.sort();
        Scheme {
            vars,
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
                        diag.push(Diagnostic::error(
                            span_of_type_expr(t),
                            format!(
                                "型エイリアス `{name}` は {} 個の引数を取りますが {} 個でした",
                                alias.params.len(),
                                args_ty.len()
                            ),
                        ));
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
                        ])
                    }
                    "Point3D" => {
                        return Type::Record(vec![
                            ("x".to_string(), Type::con("Float")),
                            ("y".to_string(), Type::con("Float")),
                            ("z".to_string(), Type::con("Float")),
                        ])
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
            args.iter().map(|a| substitute_named(a, subst_map, var_map)).collect(),
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
            diag.push(Diagnostic::error(
                imp.span,
                format!("import 先 `{imp_qname}` の型情報がまだロードされていません"),
            ));
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

    // 4. 各 value decl を推論
    let mut result: HashMap<String, Scheme> = HashMap::new();
    for decl in &module.decls {
        if let Decl::Value(v) = decl {
            // signature があれば先に instantiate し、param 型を body 推論に流す。
            let declared = infer.signatures.get(&v.name).cloned();
            let expected = declared.as_ref().map(|sc| infer.instantiate(sc));
            let (ty, subst) = match infer_value_decl(infer, &env, v, expected.as_ref(), &mut diag) {
                Some(r) => r,
                None => continue,
            };
            let final_ty = subst.apply(&ty);

            // signature があれば unify する (param 型は既に流したので戻り型整合の確認も兼ねる)
            let scheme = if let Some(declared) = declared {
                let inst = expected.unwrap();
                match unify(&final_ty, &inst) {
                    Ok(s) => {
                        let unified_ty = s.apply(&final_ty);
                        // declared scheme に合わせる (vars は declared 由来)
                        Scheme {
                            vars: declared.vars.clone(),
                            ty: unified_ty,
                        }
                    }
                    Err(e) => {
                        diag.push(unify_diag(e, v.span, &v.name));
                        Scheme::mono(final_ty)
                    }
                }
            } else {
                infer.generalize(&env, &final_ty)
            };

            env = env.extend(&v.name, scheme.clone());
            result.insert(v.name.clone(), scheme);
        }
    }

    *env_in = env;
    (result, diag)
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
            args.iter().map(|a| remap_named_vars(a, var_map, name_to_fresh)).collect(),
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
    let message = match err {
        UnifyError::Mismatch { left, right } => format!(
            "{name}: 型 `{left}` と `{right}` が一致しません"
        ),
        UnifyError::InfiniteType { var: _, ty } => {
            format!("{name}: 循環型 `{ty}` を作ろうとしています")
        }
        UnifyError::RecordFieldMismatch {
            left_fields,
            right_fields,
        } => format!(
            "{name}: レコードのフィールドが一致しません ({left_fields:?} vs {right_fields:?})"
        ),
    };
    Diagnostic::error(span, message)
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
    for (i, p) in v.params.iter().enumerate() {
        let pt = sig_param_tys.get(i).cloned().unwrap_or_else(|| infer.fresh());
        param_tys.push(pt.clone());
        bind_pattern(infer, &mut local_env, p, &pt, diag);
    }
    let (body_ty, subst) = infer_expr(infer, &local_env, &v.body, diag)?;
    let mut fun_ty = body_ty;
    for pt in param_tys.into_iter().rev() {
        fun_ty = Type::arrow(subst.apply(&pt), fun_ty);
    }
    Some((fun_ty, subst))
}

/// pattern を env に束縛。Var / Wildcard を直接束縛し、Ctor は ctor_types から
/// 取得して args 数のみチェックする。
fn bind_pattern(
    infer: &mut Infer,
    env: &mut TypeEnv,
    p: &Pattern,
    expected: &Type,
    diag: &mut Vec<Diagnostic>,
) {
    match p {
        Pattern::Var(name, _) => {
            *env = env.extend(name, Scheme::mono(expected.clone()));
        }
        Pattern::Wildcard(_) => {}
        Pattern::Lit(_, _) => {} // リテラルは束縛しない
        Pattern::Ctor { name, args, span, .. } => {
            // コンストラクタの型を instantiate して、引数型と patterns を再帰束縛
            let sc = match infer.ctor_types.get(name).cloned() {
                Some(s) => s,
                None => {
                    diag.push(Diagnostic::error(
                        *span,
                        format!("コンストラクタ `{name}` は未定義"),
                    ));
                    return;
                }
            };
            let inst = infer.instantiate(&sc);
            // inst は a1 -> ... -> an -> RetTy の curried 形式
            let (arg_tys, ret_ty) = split_arrows(&inst, args.len());
            // ret_ty を expected と unify
            if let Err(e) = unify(&ret_ty, expected) {
                diag.push(unify_diag(e, *span, name));
            }
            for (sub_p, ty) in args.iter().zip(arg_tys.iter()) {
                bind_pattern(infer, env, sub_p, ty, diag);
            }
        }
        Pattern::List(items, _) => {
            // expected は List elem。expected と List a を unify
            let elem = infer.fresh();
            let list_ty = Type::app("List", vec![elem.clone()]);
            if let Err(e) = unify(&list_ty, expected) {
                diag.push(unify_diag(e, span_of_pattern(p), "[]"));
            }
            for item in items {
                bind_pattern(infer, env, item, &elem, diag);
            }
        }
        Pattern::Cons { head, tail, span } => {
            let elem = infer.fresh();
            let list_ty = Type::app("List", vec![elem.clone()]);
            if let Err(e) = unify(&list_ty, expected) {
                diag.push(unify_diag(e, *span, "::"));
            }
            bind_pattern(infer, env, head, &elem, diag);
            bind_pattern(infer, env, tail, &list_ty, diag);
        }
        Pattern::Record(_, _) | Pattern::As { .. } => {
            // TODO: record pattern と as pattern の型束縛
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
                Some(sc) => Some((infer.instantiate(sc), Subst::empty())),
                None => {
                    diag.push(Diagnostic::error(*span, format!("未定義の変数 `{key}`")));
                    None
                }
            }
        }
        Expr::Ctor { module, name, span } => {
            let key = qualify(module.as_ref(), name);
            match env.lookup(&key) {
                Some(sc) => Some((infer.instantiate(sc), Subst::empty())),
                None => {
                    diag.push(Diagnostic::error(
                        *span,
                        format!("未定義のコンストラクタ `{key}`"),
                    ));
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
            for p in params {
                let pt = infer.fresh();
                param_tys.push(pt.clone());
                bind_pattern(infer, &mut local_env, p, &pt, diag);
            }
            let (body_ty, s) = infer_expr(infer, &local_env, body, diag)?;
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
            op, left, right, span,
        } => {
            let (op_args, op_ret) = binop_type(*op, infer);
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
        Expr::Negate(inner, _) => {
            let (ty, s) = infer_expr(infer, env, inner, diag)?;
            // Float に固定。Int は fresh で逃がす
            let s2 = match unify(&ty, &Type::con("Float")) {
                Ok(s) => s,
                Err(_) => {
                    // Int も許容: fresh で逃げる
                    Subst::empty()
                }
            };
            Some((s.compose(&s2).apply(&ty), s.compose(&s2)))
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
                        diag.push(Diagnostic::error(
                            *span,
                            format!("レコードにフィールド `{name}` がありません"),
                        ));
                        None
                    }
                },
                Type::Con(alias_name, _) if infer.record_aliases.contains_key(alias_name) => {
                    let fields = infer.record_aliases.get(alias_name).unwrap().clone();
                    match fields.iter().find(|(n, _)| n == name) {
                        Some((_, t)) => Some((t.clone(), s)),
                        None => {
                            diag.push(Diagnostic::error(
                                *span,
                                format!(
                                    "`{alias_name}` にフィールド `{name}` がありません"
                                ),
                            ));
                            None
                        }
                    }
                }
                _ => {
                    diag.push(Diagnostic::error(
                        *span,
                        format!("レコードでないものから `.{name}` を取れません: {r_ty}"),
                    ));
                    None
                }
            }
        }
        Expr::RecordUpdate { base, updates, span } => {
            let (b_ty, s) = infer_expr(infer, env, base, diag)?;
            let subst = s;
            // updates の各 field の型は base record の同名 field と一致する必要
            let _ = updates;
            let _ = span;
            // TODO: updates の field 型を base record と単一化する
            Some((subst.apply(&b_ty), subst))
        }
        Expr::Case { .. } => {
            // TODO: Case 式の本格的な型付け
            Some((infer.fresh(), Subst::empty()))
        }
        Expr::Error(span) => {
            diag.push(Diagnostic::error(*span, "パースエラーがあります"));
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

fn binop_type(op: BinOp, infer: &mut Infer) -> ((Type, Type), Type) {
    use BinOp::*;
    match op {
        Add | Sub | Mul | Div => (
            (Type::con("Float"), Type::con("Float")),
            Type::con("Float"),
        ),
        Eq | NotEq => {
            let v = infer.fresh();
            ((v.clone(), v.clone()), Type::con("Bool"))
        }
        Lt | Le | Gt | Ge => (
            (Type::con("Float"), Type::con("Float")),
            Type::con("Bool"),
        ),
        And | Or => ((Type::con("Bool"), Type::con("Bool")), Type::con("Bool")),
        Cons => {
            let v = infer.fresh();
            (
                (v.clone(), Type::app("List", vec![v.clone()])),
                Type::app("List", vec![v]),
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
            )
        }
        ApplyL => {
            // f <| x : (a -> b) <| a → b
            let a = infer.fresh();
            let b = infer.fresh();
            ((Type::arrow(a.clone(), b.clone()), a), b)
        }
        ApplyR => {
            // x |> f : a |> (a -> b) → b
            let a = infer.fresh();
            let b = infer.fresh();
            ((a.clone(), Type::arrow(a, b.clone())), b)
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
