use std::cmp::Ordering;
use std::collections::HashMap;
use std::fmt;

use crate::constraint::{ArithEq, ArithExpr, solve_constraints};
use crate::parse::{
    ArithOp, Bound, Clause, QueryParam, ScopeId, ScopedTerm, SrcSpan, Term, VarAnnotation,
    first_span, list, number, struc, var,
};
use crate::term_processor::{
    all_builtin_functors, is_builtin_functor, is_builtin_functor_with_arity, should_resolve_args,
};

pub type Env = HashMap<String, ScopedTerm>;

/// 1 つの論理変数 (= UF の同一同値類) に紐付く情報。canonical な entry のみが意味を持ち、
/// 非 canonical な entry は parent pointer 経由で root に解決する。
#[derive(Debug, Clone, Default)]
pub struct VarInfo {
    pub value: Option<ScopedTerm>,
    pub annotation: VarAnnotation,
}

/// (scope, name) を key とする Union-Find ベースの環境。
/// `insert(scope, name, term)`:
///   - term が Var なら union (同値類を merge)
///   - term が非 Var なら canonical の value にセット
/// `get(scope, name)`: canonical の value (Option<&ScopedTerm>)
/// `merge_annotation(scope, name, ann)` / `get_annotation`: canonical の annotation
#[derive(Debug, Clone)]
pub struct ScopedEnv {
    parent: HashMap<(ScopeId, String), (ScopeId, String)>,
    info: HashMap<(ScopeId, String), VarInfo>,
}

impl ScopedEnv {
    pub fn new() -> Self {
        Self {
            parent: HashMap::new(),
            info: HashMap::new(),
        }
    }

    fn find_mut(&mut self, scope: ScopeId, name: &str) -> (ScopeId, String) {
        let mut current = (scope, name.to_string());
        let mut path = Vec::new();
        while let Some(p) = self.parent.get(&current) {
            if p == &current {
                break;
            }
            path.push(current.clone());
            current = p.clone();
        }
        for k in path {
            self.parent.insert(k, current.clone());
        }
        current
    }

    fn find_immutable(&self, scope: ScopeId, name: &str) -> (ScopeId, String) {
        let mut current = (scope, name.to_string());
        while let Some(p) = self.parent.get(&current) {
            if p == &current {
                break;
            }
            current = p.clone();
        }
        current
    }

    /// (scope, name) の同値類に term をバインドする。
    /// term が Var なら union、それ以外なら canonical の value にセット。
    pub fn insert(&mut self, scope: ScopeId, name: String, term: ScopedTerm) {
        if let Term::Var {
            name: n2,
            scope: s2,
            ..
        } = &term
            && n2 != "_"
        {
            self.union(scope, &name, *s2, n2);
            return;
        }
        let root = self.find_mut(scope, &name);
        self.info.entry(root).or_default().value = Some(term);
    }

    /// canonical の value を返す (チェーン解決済み)。binding が無ければ None。
    pub fn get(&self, scope: ScopeId, name: &str) -> Option<&ScopedTerm> {
        let root = self.find_immutable(scope, name);
        self.info.get(&root).and_then(|i| i.value.as_ref())
    }

    /// (scope, name) の同値類における canonical (root) を返す。Union-Find のルートが自身と
    /// 異なる場合のみ Some を返す (Var-Var unify によって他の Var と連結されているケース)。
    /// 連結が無い (root == 自身) なら None。値 (Number 等) が bind されているかは無関係。
    pub fn canonical_var(&self, scope: ScopeId, name: &str) -> Option<(ScopeId, String)> {
        let root = self.find_immutable(scope, name);
        if root == (scope, name.to_string()) {
            None
        } else {
            Some(root)
        }
    }

    /// canonical の annotation に merge (self 優先)。
    pub fn merge_annotation(&mut self, scope: ScopeId, name: &str, value: VarAnnotation) {
        let root = self.find_mut(scope, name);
        self.info.entry(root).or_default().annotation.absorb(&value);
    }

    /// canonical の annotation を読み取る (path compression なし、`&self` で使える)。
    pub fn get_annotation(&self, scope: ScopeId, name: &str) -> VarAnnotation {
        let root = self.find_immutable(scope, name);
        self.info
            .get(&root)
            .map(|i| i.annotation.clone())
            .unwrap_or_default()
    }

    /// 2 つの同値類を結合し、value と annotation を merge する。
    pub fn union(&mut self, s1: ScopeId, n1: &str, s2: ScopeId, n2: &str) {
        let r1 = self.find_mut(s1, n1);
        let r2 = self.find_mut(s2, n2);
        if r1 == r2 {
            return;
        }
        let info2 = self.info.remove(&r2).unwrap_or_default();
        let entry = self.info.entry(r1.clone()).or_default();
        // value: r1 を優先、無ければ r2 のを取る
        if entry.value.is_none() {
            entry.value = info2.value;
        }
        entry.annotation.absorb(&info2.annotation);
        self.parent.insert(r2, r1);
    }
}

const RESOLVE_DEPTH_LIMIT: usize = 256;

/// envを参照して変数を再帰的に解決する。
pub fn resolve(term: &ScopedTerm, env: &ScopedEnv) -> ScopedTerm {
    resolve_inner(term, env, 0)
}

/// `base` が Var なら、ScopedEnv (Union-Find) から canonical な annotation を取り出して
/// Term::Var フィールドに反映する。Term::Var が source of truth ではなく、UF env が
/// 同一論理変数の集約された annotation を保持するため、resolve のたびにここで同期する。
fn apply_canonical_annotation(base: ScopedTerm, env: &ScopedEnv) -> ScopedTerm {
    if let Term::Var {
        name,
        scope,
        default_value,
        min,
        max,
        span,
    } = base
    {
        let canon = env.get_annotation(scope, &name);
        Term::Var {
            default_value: default_value.or(canon.default_value),
            min: min.or(canon.min),
            max: max.or(canon.max),
            span: span.or(canon.span),
            name,
            scope,
        }
    } else {
        base
    }
}

fn resolve_inner(term: &ScopedTerm, env: &ScopedEnv, depth: usize) -> ScopedTerm {
    if depth > RESOLVE_DEPTH_LIMIT {
        panic!(
            "resolve depth limit exceeded, possible cyclic bindings in env: {:?}",
            env
        );
    }
    match term {
        Term::Var {
            name,
            scope,
            default_value,
            min,
            max,
            span,
        } if name != "_" => {
            let has_annotation = default_value.is_some() || min.is_some() || max.is_some();
            let sid = *scope;
            match env.get(sid, name) {
                Some(Term::Number { value: new_val }) if has_annotation => Term::Var {
                    name: name.clone(),
                    scope: *scope,
                    default_value: Some(new_val.clone()),
                    min: min.clone(),
                    max: max.clone(),
                    span: *span,
                },
                Some(val) => {
                    let resolved = resolve_inner(val, env, depth + 1);
                    // 解決後の値が Var で、元の Var に annotation があれば、副 env から
                    // canonical な annotation を当てはめて引き継ぐ(unify で他箇所と merge
                    // された min/max/default が反映される)。元の Var が bare な場合は
                    // 過剰な補完を避けるためそのまま返す。
                    if has_annotation {
                        apply_canonical_annotation(resolved, env)
                    } else {
                        resolved
                    }
                }
                // 値 bind 無し: Var-Var unify (union) によって他の Var と同値類に連結されて
                // いるなら canonical (root) の Var に正規化する。これがないと、user-defined
                // predicate を介して unify された別 scope の同名 Var が、後段の (name, scope)
                // 完全一致での substitution (例: extract_control_points の bare-Var unify)
                // に拾われず、別物として残ってしまう。
                None => {
                    if let Some((canon_scope, canon_name)) = env.canonical_var(sid, name) {
                        let canon_ann = env.get_annotation(canon_scope, &canon_name);
                        Term::Var {
                            name: canon_name,
                            scope: canon_scope,
                            default_value: default_value.clone().or(canon_ann.default_value),
                            min: min.clone().or(canon_ann.min),
                            max: max.clone().or(canon_ann.max),
                            span: span.or(canon_ann.span),
                        }
                    } else {
                        term.clone()
                    }
                }
            }
        }
        Term::InfixExpr { op, left, right } => {
            let new_left = resolve_inner(left, env, depth + 1);
            let new_right = resolve_inner(right, env, depth + 1);
            let new_term = Term::InfixExpr {
                op: *op,
                left: Box::new(new_left),
                right: Box::new(new_right),
            };
            if let Some(val) = try_fold_number_literals(&new_term) {
                number(val)
            } else {
                new_term
            }
        }
        Term::Struct {
            functor,
            args,
            span,
        } => Term::Struct {
            functor: functor.clone(),
            args: args
                .iter()
                .map(|a| resolve_inner(a, env, depth + 1))
                .collect(),
            span: *span,
        },
        Term::List { items, tail } => Term::List {
            items: items
                .iter()
                .map(|i| resolve_inner(i, env, depth + 1))
                .collect(),
            tail: tail
                .as_ref()
                .map(|t| Box::new(resolve_inner(t, env, depth + 1))),
        },
        Term::Constraint { left, right } => Term::Constraint {
            left: Box::new(resolve_inner(left, env, depth + 1)),
            right: Box::new(resolve_inner(right, env, depth + 1)),
        },
        _ => term.clone(),
    }
}

/// Constraint内のVarから name→scope のマッピングを収集する
fn collect_var_scopes_from_term(term: &ScopedTerm, scopes: &mut HashMap<String, ScopeId>) {
    match term {
        Term::Var { name, scope, .. } if name != "_" => {
            scopes.entry(name.clone()).or_insert(*scope);
        }
        Term::InfixExpr { left, right, .. } => {
            collect_var_scopes_from_term(left, scopes);
            collect_var_scopes_from_term(right, scopes);
        }
        Term::Constraint { left, right } => {
            collect_var_scopes_from_term(left, scopes);
            collect_var_scopes_from_term(right, scopes);
        }
        _ => {}
    }
}

/// Check if a term is a built-in primitive that should not be rewritten
fn is_builtin_term<S>(term: &Term<S>) -> bool {
    match term {
        Term::Struct { functor, .. } => is_builtin_functor(functor),
        // InfixExpr (+, -, *) with CAD primitives as operands is also builtin
        Term::InfixExpr { left, right, .. } => is_builtin_term(left) && is_builtin_term(right),
        _ => false,
    }
}

/// ジオメトリではないメタデータ用のビルトインファンクタ（bom等）かどうか
fn is_metadata_term<S>(term: &Term<S>) -> bool {
    matches!(term, Term::Struct { functor, .. }
        if is_builtin_functor(functor) && !should_resolve_args(functor))
}

/// control2d / control3d のいずれかかどうか
fn is_control_term<S>(term: &Term<S>) -> bool {
    matches!(term, Term::Struct { functor, .. } if functor == "control2d" || functor == "control3d")
}

fn builtin_fact(functor: &str, arity: usize) -> Clause {
    let args = (0..arity)
        .map(|idx| var(format!("__builtin_arg_{}", idx)))
        .collect();
    Clause::Fact(struc(functor.to_string(), args))
}

fn builtin_cad_facts() -> Vec<Clause> {
    all_builtin_functors()
        .into_iter()
        .flat_map(|(name, arities)| {
            arities
                .iter()
                .copied()
                .map(move |arity| builtin_fact(name, arity))
        })
        .collect()
}

/// 単一化エラー
#[derive(Debug, Clone)]
pub struct UnifyError {
    pub message: String,
    pub term1: ScopedTerm,
    pub term2: ScopedTerm,
}

impl fmt::Display for UnifyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for UnifyError {}

/// 書き換えステップのエラー
#[derive(Debug, Clone)]
pub struct RewriteError {
    pub message: String,
    pub goal: ScopedTerm,
}

impl fmt::Display for RewriteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {:?}", self.message, self.goal)
    }
}

impl std::error::Error for RewriteError {}

pub trait CadhrError: std::error::Error {
    fn error_message(&self) -> String;
    fn span(&self) -> Option<SrcSpan>;
}

impl CadhrError for UnifyError {
    fn error_message(&self) -> String {
        self.message.clone()
    }
    fn span(&self) -> Option<SrcSpan> {
        first_span::<ScopeId>(&self.term1).or_else(|| first_span::<ScopeId>(&self.term2))
    }
}

impl CadhrError for RewriteError {
    fn error_message(&self) -> String {
        self.message.clone()
    }
    fn span(&self) -> Option<SrcSpan> {
        first_span::<ScopeId>(&self.goal)
    }
}

fn collect_default_var_bindings(
    term: &ScopedTerm,
    bindings: &mut Vec<(String, ScopeId, crate::rational::Rational)>,
) {
    match term {
        Term::Var {
            name,
            scope,
            default_value: Some(value),
            ..
        } if name != "_" => {
            bindings.push((name.clone(), *scope, value.clone()));
        }
        Term::Var { .. } => {}
        Term::StringLit { .. } => {}
        Term::Number { .. } => {}
        Term::Struct { args, .. } => {
            for arg in args {
                collect_default_var_bindings(arg, bindings);
            }
        }
        Term::List { items, tail } => {
            for item in items {
                collect_default_var_bindings(item, bindings);
            }
            if let Some(t) = tail {
                collect_default_var_bindings(t, bindings);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            collect_default_var_bindings(left, bindings);
            collect_default_var_bindings(right, bindings);
        }
        Term::Constraint { left, right } => {
            collect_default_var_bindings(left, bindings);
            collect_default_var_bindings(right, bindings);
        }
    }
}

fn apply_default_var_bindings(term: &mut ScopedTerm, goals: &mut Vec<ScopedTerm>) {
    let mut bindings = Vec::new();
    collect_default_var_bindings(term, &mut bindings);
    let mut env = ScopedEnv::new();
    for (name, scope, value) in bindings {
        env.insert(scope, name, number(value));
    }
    *term = resolve(term, &env);
    for goal in goals.iter_mut() {
        *goal = resolve(goal, &env);
    }
}



/// term に含まれる @-default を `targets` の各項に反映させる。
/// rewrite_term_recursive 内の apply_default_var_bindings は other_goals にしか伝播しないので、
/// 同じルール本体の未処理 body 項に @-default を波及させたい時はこのヘルパーを使う。
fn propagate_defaults_to(term: &ScopedTerm, targets: &mut [ScopedTerm]) {
    let mut bindings = Vec::new();
    collect_default_var_bindings(term, &mut bindings);
    if bindings.is_empty() {
        return;
    }
    let mut env = ScopedEnv::new();
    for (name, scope, value) in bindings {
        env.insert(scope, name, number(value));
    }
    for t in targets.iter_mut() {
        *t = resolve(t, &env);
    }
}

/// Number と InfixExpr だけで構成された算術式を畳み込む。
/// Var は処理しない。unify の Var ハンドラが名前ベースの置換を伴って処理するため、
/// ここで Number に変換すると置換が抜け落ちる。
fn try_fold_number_literals<S>(term: &Term<S>) -> Option<crate::rational::Rational> {
    match term {
        Term::Number { value } => Some(value.clone()),
        Term::InfixExpr { op, left, right } => {
            let l = try_fold_number_literals(left)?;
            let r = try_fold_number_literals(right)?;
            // 0 除算は None で未解決のまま残す
            if matches!(op, ArithOp::Div) && r.is_zero() {
                return None;
            }
            Some(match op {
                ArithOp::Add => l + r,
                ArithOp::Sub => l - r,
                ArithOp::Mul => l * r,
                ArithOp::Div => l / r,
            })
        }
        Term::Var { .. }
        | Term::Struct { .. }
        | Term::List { .. }
        | Term::StringLit { .. }
        | Term::Constraint { .. } => None,
    }
}

/// Var の default_value も数値として扱い算術式を評価する。
/// unify 中では使わず、最終的な数値抽出（メッシュ生成など）で使う。
pub fn try_eval_to_number<S>(term: &Term<S>) -> Option<crate::rational::Rational> {
    match term {
        Term::Number { value } => Some(value.clone()),
        Term::Var {
            default_value: Some(value),
            ..
        } => Some(value.clone()),
        Term::InfixExpr { op, left, right } => {
            let l = try_eval_to_number(left)?;
            let r = try_eval_to_number(right)?;
            if matches!(op, ArithOp::Div) && r.is_zero() {
                return None;
            }
            Some(match op {
                ArithOp::Add => l + r,
                ArithOp::Sub => l - r,
                ArithOp::Mul => l * r,
                ArithOp::Div => l / r,
            })
        }
        Term::Var {
            default_value: None,
            ..
        }
        | Term::Struct { .. }
        | Term::List { .. }
        | Term::StringLit { .. }
        | Term::Constraint { .. } => None,
    }
}

/// 算術式をインプレースで畳み込み、可能なら数値に置き換える
pub fn fold_number_literals_in_place<S>(term: &mut Term<S>) {
    if let Some(val) = try_fold_number_literals(term) {
        *term = number(val);
    } else {
        match term {
            Term::InfixExpr { left, right, .. } => {
                fold_number_literals_in_place(left.as_mut());
                fold_number_literals_in_place(right.as_mut());
            }
            Term::Struct { args, .. } => {
                for arg in args.iter_mut() {
                    fold_number_literals_in_place(arg);
                }
            }
            Term::List { items, tail } => {
                for item in items.iter_mut() {
                    fold_number_literals_in_place(item);
                }
                if let Some(t) = tail {
                    fold_number_literals_in_place(t.as_mut());
                }
            }
            Term::Constraint { left, right } => {
                fold_number_literals_in_place(left.as_mut());
                fold_number_literals_in_place(right.as_mut());
            }
            Term::Number { .. } | Term::Var { .. } | Term::StringLit { .. } => {}
        }
    }
}

/// occurs check: 変数varが項term内に出現するか
fn occurs_check_scoped(var_name: &str, var_scope: ScopeId, term: &ScopedTerm) -> bool {
    match term {
        Term::Var { name, scope, .. } => name == var_name && *scope == var_scope,
        Term::Struct { args, .. } => args
            .iter()
            .any(|arg| occurs_check_scoped(var_name, var_scope, arg)),
        Term::List { items, tail } => {
            items
                .iter()
                .any(|item| occurs_check_scoped(var_name, var_scope, item))
                || tail
                    .as_ref()
                    .map_or(false, |t| occurs_check_scoped(var_name, var_scope, t))
        }
        Term::InfixExpr { left, right, .. } => {
            occurs_check_scoped(var_name, var_scope, left)
                || occurs_check_scoped(var_name, var_scope, right)
        }
        Term::Constraint { left, right } => {
            occurs_check_scoped(var_name, var_scope, left)
                || occurs_check_scoped(var_name, var_scope, right)
        }
        Term::Number { .. } | Term::StringLit { .. } => false,
    }
}

/// 2つの下限境界から、より厳しい方を選択
fn intersect_min(a: Option<&Bound>, b: Option<&Bound>) -> Option<Bound> {
    match (a, b) {
        (None, None) => None,
        (Some(x), None) | (None, Some(x)) => Some(x.clone()),
        (Some(a), Some(b)) => {
            if a.value > b.value {
                Some(a.clone())
            } else if b.value > a.value {
                Some(b.clone())
            } else {
                // 同じ値の場合、exclusiveの方が厳しい
                Some(Bound {
                    value: a.value.clone(),
                    inclusive: a.inclusive && b.inclusive,
                })
            }
        }
    }
}

/// 2つの上限境界から、より厳しい方を選択
fn intersect_max(a: Option<&Bound>, b: Option<&Bound>) -> Option<Bound> {
    match (a, b) {
        (None, None) => None,
        (Some(x), None) | (None, Some(x)) => Some(x.clone()),
        (Some(a), Some(b)) => {
            if a.value < b.value {
                Some(a.clone())
            } else if b.value < a.value {
                Some(b.clone())
            } else {
                // 同じ値の場合、exclusiveの方が厳しい
                Some(Bound {
                    value: a.value.clone(),
                    inclusive: a.inclusive && b.inclusive,
                })
            }
        }
    }
}

fn bound_add(a: &Bound, b: &Bound) -> Bound {
    Bound {
        value: &a.value + &b.value,
        inclusive: a.inclusive && b.inclusive,
    }
}

fn bound_sub(a: &Bound, b: &Bound) -> Bound {
    Bound {
        value: &a.value - &b.value,
        inclusive: a.inclusive && b.inclusive,
    }
}

fn bound_mul(a: &Bound, b: &Bound) -> Bound {
    Bound {
        value: &a.value * &b.value,
        inclusive: a.inclusive && b.inclusive,
    }
}

type Range = (Option<Bound>, Option<Bound>);

fn compute_term_range<S>(term: &Term<S>) -> Result<Range, String> {
    match term {
        Term::Number { value } => Ok((
            Some(Bound {
                value: value.clone(),
                inclusive: true,
            }),
            Some(Bound {
                value: value.clone(),
                inclusive: true,
            }),
        )),
        Term::Var { min, max, .. } => Ok((min.clone(), max.clone())),
        Term::InfixExpr { op, left, right } => {
            let (l_min, l_max) = compute_term_range(left)?;
            let (r_min, r_max) = compute_term_range(right)?;
            match op {
                ArithOp::Add => Ok((
                    l_min.zip(r_min).map(|(a, b)| bound_add(&a, &b)),
                    l_max.zip(r_max).map(|(a, b)| bound_add(&a, &b)),
                )),
                ArithOp::Sub => Ok((
                    l_min.zip(r_max).map(|(a, b)| bound_sub(&a, &b)),
                    l_max.zip(r_min).map(|(a, b)| bound_sub(&a, &b)),
                )),
                ArithOp::Mul => match (l_min, l_max, r_min, r_max) {
                    (Some(l_lo), Some(l_hi), Some(r_lo), Some(r_hi)) => {
                        let products = [
                            bound_mul(&l_lo, &r_lo),
                            bound_mul(&l_lo, &r_hi),
                            bound_mul(&l_hi, &r_lo),
                            bound_mul(&l_hi, &r_hi),
                        ];
                        let min = products
                            .iter()
                            .min_by(|a, b| a.value.cmp(&b.value))
                            .unwrap()
                            .clone();
                        let max = products
                            .iter()
                            .max_by(|a, b| a.value.cmp(&b.value))
                            .unwrap()
                            .clone();
                        Ok((Some(min), Some(max)))
                    }
                    _ => Ok((None, None)),
                },
                ArithOp::Div => match (r_min, r_max) {
                    (Some(r_lo), Some(r_hi)) => {
                        if r_lo.value.is_zero()
                            || r_hi.value.is_zero()
                            || (r_lo.value.to_f64() <= 0.0 && r_hi.value.to_f64() >= 0.0)
                            || (r_lo.value.to_f64() >= 0.0 && r_hi.value.to_f64() <= 0.0)
                        {
                            return Err("Division by zero: divisor range includes zero".to_string());
                        }
                        let one = crate::rational::Rational::from_integer(1);
                        let inv_lo = Bound {
                            value: &one / &r_hi.value,
                            inclusive: r_hi.inclusive,
                        };
                        let inv_hi = Bound {
                            value: &one / &r_lo.value,
                            inclusive: r_lo.inclusive,
                        };
                        match (l_min, l_max) {
                            (Some(l_lo), Some(l_hi)) => {
                                let products = [
                                    bound_mul(&l_lo, &inv_lo),
                                    bound_mul(&l_lo, &inv_hi),
                                    bound_mul(&l_hi, &inv_lo),
                                    bound_mul(&l_hi, &inv_hi),
                                ];
                                let min = products
                                    .iter()
                                    .min_by(|a, b| a.value.cmp(&b.value))
                                    .unwrap()
                                    .clone();
                                let max = products
                                    .iter()
                                    .max_by(|a, b| a.value.cmp(&b.value))
                                    .unwrap()
                                    .clone();
                                Ok((Some(min), Some(max)))
                            }
                            _ => Ok((None, None)),
                        }
                    }
                    _ => Ok((None, None)),
                },
            }
        }
        _ => Ok((None, None)),
    }
}

fn collect_ranges_from_body_terms(
    body: &[Term],
    ranges: &mut HashMap<String, (Option<Bound>, Option<Bound>)>,
) {
    for term in body {
        collect_ranges_from_body_term(term, ranges);
    }
}

fn collect_ranges_from_body_term(
    term: &Term,
    ranges: &mut HashMap<String, (Option<Bound>, Option<Bound>)>,
) {
    match term {
        Term::Var { name, min, max, .. } if name != "_" && (min.is_some() || max.is_some()) => {
            let entry = ranges.entry(name.clone()).or_insert((None, None));
            entry.0 = intersect_min(entry.0.as_ref(), min.as_ref());
            entry.1 = intersect_max(entry.1.as_ref(), max.as_ref());
        }
        Term::Struct { args, .. } => {
            for arg in args {
                collect_ranges_from_body_term(arg, ranges);
            }
        }
        Term::List { items, tail } => {
            for item in items {
                collect_ranges_from_body_term(item, ranges);
            }
            if let Some(t) = tail {
                collect_ranges_from_body_term(t, ranges);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            collect_ranges_from_body_term(left, ranges);
            collect_ranges_from_body_term(right, ranges);
        }
        Term::Constraint { left, right } => {
            collect_ranges_from_body_term(left, ranges);
            collect_ranges_from_body_term(right, ranges);
        }
        _ => {}
    }
}

pub fn infer_query_param_ranges(
    query_terms: &[Term],
    solved_db: &[Clause],
    params: &mut [QueryParam],
) -> Result<(), String> {
    for q_term in query_terms {
        if let Term::Struct {
            functor: q_func,
            args: q_args,
            ..
        } = q_term
        {
            for clause in solved_db {
                let (head, body) = match clause {
                    Clause::Rule { head, body } => (head, body.as_slice()),
                    Clause::Fact(head) => (head, [].as_slice()),
                    _ => continue,
                };
                if let Term::Struct {
                    functor: h_func,
                    args: h_args,
                    ..
                } = head
                {
                    if q_func == h_func && q_args.len() == h_args.len() {
                        // body内のrange付きVarを収集
                        let mut body_ranges: HashMap<String, (Option<Bound>, Option<Bound>)> =
                            HashMap::new();
                        collect_ranges_from_body_terms(body, &mut body_ranges);

                        // query arg → head arg name → body range のマッピング
                        for (q_arg, h_arg) in q_args.iter().zip(h_args.iter()) {
                            if let Term::Var { name: q_name, .. } = q_arg {
                                // head arg の変数名を取得
                                if let Term::Var { name: h_name, .. } = h_arg {
                                    if let Some((r_min, r_max)) = body_ranges.get(h_name) {
                                        for param in params.iter_mut() {
                                            if &param.name == q_name {
                                                param.min = intersect_min(
                                                    param.min.as_ref(),
                                                    r_min.as_ref(),
                                                );
                                                param.max = intersect_max(
                                                    param.max.as_ref(),
                                                    r_max.as_ref(),
                                                );
                                            }
                                        }
                                    }
                                }
                                // head arg が InfixExpr の場合: compute_term_range で body の range 付き変数を含めて計算
                                if let Term::InfixExpr { .. } = h_arg {
                                    // InfixExpr内の変数にbody rangeを適用してrange推定
                                    let mut patched = h_arg.clone();
                                    apply_body_ranges_to_term(&mut patched, &body_ranges);
                                    let (range_min, range_max) = compute_term_range(&patched)?;
                                    for param in params.iter_mut() {
                                        if &param.name == q_name {
                                            param.min = intersect_min(
                                                param.min.as_ref(),
                                                range_min.as_ref(),
                                            );
                                            param.max = intersect_max(
                                                param.max.as_ref(),
                                                range_max.as_ref(),
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

fn apply_body_ranges_to_term(
    term: &mut Term,
    ranges: &HashMap<String, (Option<Bound>, Option<Bound>)>,
) {
    match term {
        Term::Var { name, min, max, .. } if name != "_" => {
            if let Some((r_min, r_max)) = ranges.get(name) {
                *min = intersect_min(min.as_ref(), r_min.as_ref());
                *max = intersect_max(max.as_ref(), r_max.as_ref());
            }
        }
        Term::InfixExpr { left, right, .. } => {
            apply_body_ranges_to_term(left, ranges);
            apply_body_ranges_to_term(right, ranges);
        }
        _ => {}
    }
}

/// 値が範囲内にあるかチェック
fn value_in_range(
    value: &crate::rational::Rational,
    min: Option<&Bound>,
    max: Option<&Bound>,
) -> bool {
    let min_ok = match min {
        None => true,
        Some(b) if b.inclusive => *value >= b.value,
        Some(b) => *value > b.value,
    };
    let max_ok = match max {
        None => true,
        Some(b) if b.inclusive => *value <= b.value,
        Some(b) => *value < b.value,
    };
    min_ok && max_ok
}

/// 項が算術式として評価可能な形か（変数と数値とInfixExprのみで構成されているか）
fn is_potentially_arithmetic<S>(term: &Term<S>) -> bool {
    match term {
        Term::Number { .. } => true,
        Term::Var { .. } => true,
        Term::InfixExpr { left, right, .. } => {
            is_potentially_arithmetic(left) && is_potentially_arithmetic(right)
        }
        _ => false,
    }
}

/// 2つの項を単一化し、変数束縛をenvに蓄積する。
/// 解決できなかった遅延制約をVec<Term>として返す。
pub fn unify(
    term1: ScopedTerm,
    term2: ScopedTerm,
    env: &mut ScopedEnv,
) -> Result<Vec<ScopedTerm>, UnifyError> {
    let mut stack = vec![(term1, term2)];
    let mut deferred: Vec<(ScopedTerm, ScopedTerm)> = Vec::new();

    while let Some((t1_raw, t2_raw)) = stack.pop() {
        let mut t1 = resolve(&t1_raw, env);
        let mut t2 = resolve(&t2_raw, env);

        if let Term::Var {
            name,
            scope,
            default_value: Some(value),
            ..
        } = &t1
        {
            if name != "_" {
                env.insert(*scope, name.clone(), number(value.clone()));
            }
            t1 = number(value.clone());
        }
        if let Term::Var {
            name,
            scope,
            default_value: Some(value),
            ..
        } = &t2
        {
            if name != "_" {
                env.insert(*scope, name.clone(), number(value.clone()));
            }
            t2 = number(value.clone());
        }

        if let Some(val) = try_fold_number_literals(&t1) {
            t1 = number(val);
        }
        if let Some(val) = try_fold_number_literals(&t2) {
            t2 = number(val);
        }

        // 算術式がまだ評価できない場合は遅延
        if matches!(t1, Term::InfixExpr { .. }) || matches!(t2, Term::InfixExpr { .. }) {
            deferred.push((t1, t2));
            continue;
        }

        match (&t1, &t2) {
            // 同名同scope、annotation なし: identity
            (
                Term::Var {
                    name: n1,
                    scope: s1,
                    default_value: None,
                    min: None,
                    max: None,
                    ..
                },
                Term::Var {
                    name: n2,
                    scope: s2,
                    default_value: None,
                    min: None,
                    max: None,
                    ..
                },
            ) if n1 == n2 && s1 == s2 => {}
            // Var vs Number
            (Term::Var { name, scope, .. }, Term::Number { .. }) => {
                if name != "_" {
                    env.insert(*scope, name.clone(), t2.clone());
                }
            }
            (Term::Number { .. }, Term::Var { .. }) => {
                stack.push((t2, t1));
            }
            // Var vs other
            (Term::Var { name, scope, .. }, _) if name != "_" => {
                if occurs_check_scoped(name, *scope, &t2) {
                    return Err(UnifyError {
                        message: format!("occurs check failed: {} occurs in {:?}", name, t2),
                        term1: t1,
                        term2: t2,
                    });
                }
                // Var-Var の場合は annotation も Union-Find で集約しておく。
                if let Term::Var {
                    name: n2,
                    scope: s2,
                    ..
                } = &t2
                    && n2 != "_"
                {
                    env.union(*scope, name, *s2, n2);
                }
                env.insert(*scope, name.clone(), t2.clone());
            }
            (_, Term::Var { name, .. }) if name != "_" => {
                stack.push((t2, t1));
            }
            // wildcard
            (Term::Var { name, .. }, _) if name == "_" => {}
            (_, Term::Var { name, .. }) if name == "_" => {}
            (Term::Number { value: v1 }, Term::Number { value: v2 }) => {
                if v1 != v2 {
                    return Err(UnifyError {
                        message: format!("number mismatch: {} != {}", v1, v2),
                        term1: t1,
                        term2: t2,
                    });
                }
            }
            (Term::StringLit { value: v1 }, Term::StringLit { value: v2 }) => {
                if v1 != v2 {
                    return Err(UnifyError {
                        message: format!("string mismatch: \"{}\" != \"{}\"", v1, v2),
                        term1: t1,
                        term2: t2,
                    });
                }
            }
            (
                Term::Struct {
                    functor: f1,
                    args: args1,
                    ..
                },
                Term::Struct {
                    functor: f2,
                    args: args2,
                    ..
                },
            ) => {
                if f1 != f2 {
                    return Err(UnifyError {
                        message: format!("functor mismatch: {} != {}", f1, f2),
                        term1: t1,
                        term2: t2,
                    });
                }
                if args1.len() != args2.len() {
                    return Err(UnifyError {
                        message: format!(
                            "arity mismatch: {}/{} != {}/{}",
                            f1,
                            args1.len(),
                            f2,
                            args2.len()
                        ),
                        term1: t1,
                        term2: t2,
                    });
                }
                for (a1, a2) in args1.iter().zip(args2.iter()) {
                    stack.push((a1.clone(), a2.clone()));
                }
            }
            (
                Term::List {
                    items: items1,
                    tail: tail1,
                },
                Term::List {
                    items: items2,
                    tail: tail2,
                },
            ) => {
                let min_len = items1.len().min(items2.len());
                for i in (0..min_len).rev() {
                    stack.push((items1[i].clone(), items2[i].clone()));
                }

                match (items1.len().cmp(&items2.len()), tail1, tail2) {
                    (Ordering::Equal, Some(t1), Some(t2)) => {
                        stack.push((t1.as_ref().clone(), t2.as_ref().clone()));
                    }
                    (Ordering::Equal, None, None) => {}
                    (Ordering::Equal, Some(t1), None) => {
                        stack.push((t1.as_ref().clone(), list::<ScopeId>(vec![], None)));
                    }
                    (Ordering::Equal, None, Some(t2)) => {
                        stack.push((list::<ScopeId>(vec![], None), t2.as_ref().clone()));
                    }
                    (Ordering::Greater, _, Some(t2_tail)) => {
                        let remaining: Vec<ScopedTerm> = items1[min_len..].to_vec();
                        let new_list = list(remaining, tail1.as_ref().map(|t| t.as_ref().clone()));
                        stack.push((new_list, t2_tail.as_ref().clone()));
                    }
                    (Ordering::Greater, _, None) => {
                        return Err(UnifyError {
                            message: format!(
                                "list length mismatch: {} items vs {} items (no tail)",
                                items1.len(),
                                items2.len()
                            ),
                            term1: t1,
                            term2: t2,
                        });
                    }
                    (Ordering::Less, Some(t1_tail), _) => {
                        let remaining: Vec<ScopedTerm> = items2[min_len..].to_vec();
                        let new_list = list(remaining, tail2.as_ref().map(|t| t.as_ref().clone()));
                        stack.push((t1_tail.as_ref().clone(), new_list));
                    }
                    (Ordering::Less, None, _) => {
                        return Err(UnifyError {
                            message: format!(
                                "list length mismatch: {} items (no tail) vs {} items",
                                items1.len(),
                                items2.len()
                            ),
                            term1: t1,
                            term2: t2,
                        });
                    }
                }
            }
            _ => {
                return Err(UnifyError {
                    message: format!("cannot unify {:?} with {:?}", t1, t2),
                    term1: t1,
                    term2: t2,
                });
            }
        }

        // 変数が束縛されたので、遅延された制約を再評価
        if !deferred.is_empty() {
            let old_deferred = std::mem::take(&mut deferred);
            for constraint in old_deferred {
                stack.push(constraint);
            }
        }
    }

    // 最後に残った遅延制約を処理
    let mut constraints = Vec::new();
    for (d1, d2) in deferred {
        let t1 = resolve(&d1, env);
        let t2 = resolve(&d2, env);
        match (try_fold_number_literals(&t1), try_fold_number_literals(&t2)) {
            (Some(n1), Some(n2)) => {
                if n1 != n2 {
                    return Err(UnifyError {
                        message: format!("number mismatch: {} != {}", n1, n2),
                        term1: t1,
                        term2: t2,
                    });
                }
            }
            _ => {
                if is_potentially_arithmetic(&t1) && is_potentially_arithmetic(&t2) {
                    constraints.push(Term::Constraint {
                        left: Box::new(t1),
                        right: Box::new(t2),
                    });
                } else {
                    return Err(UnifyError {
                        message: format!("cannot unify {:?} with {:?}", t1, t2),
                        term1: t1,
                        term2: t2,
                    });
                }
            }
        }
    }

    Ok(constraints)
}

/// goals に shared_env のバインディングを適用する
fn apply_env_in_place(goals: &mut [ScopedTerm], shared_env: &ScopedEnv) {
    for g in goals.iter_mut() {
        *g = resolve(g, shared_env);
    }
}

/// body と other_goals 双方から Constraint を抜き出してまとめて処理する。
/// 各 Constraint は左右の resolved 形を見て：
///   - 両側が ArithExpr に変換できる → 線形ソルバへ
///   - 変換不能 (Struct/List 含む)   → unify()
/// 解けた束縛は shared_env に反映、残った未解決 Constraint は other_goals 末尾に戻す。
fn split_and_resolve_constraints(
    remaining_body: &mut Vec<ScopedTerm>,
    other_goals: &mut Vec<ScopedTerm>,
    shared_env: &mut ScopedEnv,
) -> Result<(), RewriteError> {
    let (body_constraints, body_rest): (Vec<_>, Vec<_>) = std::mem::take(remaining_body)
        .into_iter()
        .partition(|t| matches!(t, Term::Constraint { .. }));
    let (og_constraints, og_rest): (Vec<_>, Vec<_>) = std::mem::take(other_goals)
        .into_iter()
        .partition(|t| matches!(t, Term::Constraint { .. }));
    *remaining_body = body_rest;
    *other_goals = og_rest;
    let mut constraints = body_constraints;
    constraints.extend(og_constraints);
    if constraints.is_empty() {
        return Ok(());
    }
    // 既存の shared_env 束縛を反映してから solver/unify に渡す
    apply_env_in_place(&mut constraints, shared_env);
    try_resolve_constraints(&mut constraints, shared_env)?;
    other_goals.extend(constraints);
    Ok(())
}

/// `assert_eq(Expected, Actual)` / `assert_eq(Expected, Actual, "label")` を評価する。
/// 両辺を shared_env で展開して数値に畳み込み、一致すれば結果空のベクタを返し、
/// 不一致または評価不能なら `RewriteError` を返す。
fn handle_assert_eq(
    args: &[ScopedTerm],
    span: Option<crate::parse::SrcSpan>,
    shared_env: &ScopedEnv,
) -> Result<Vec<ScopedTerm>, RewriteError> {
    let (left, right, label) = crate::assertions::split_assert_eq_args(args);
    let label_str = label.as_deref().unwrap_or("assert_eq");

    let left_resolved = resolve(&left, shared_env);
    let right_resolved = resolve(&right, shared_env);
    let left_val = try_eval_to_number(&left_resolved);
    let right_val = try_eval_to_number(&right_resolved);

    let goal_term = Term::Struct {
        functor: "assert_eq".to_string(),
        args: args.to_vec(),
        span,
    };

    match (left_val, right_val) {
        (Some(l), Some(r)) if l == r => Ok(vec![]),
        (Some(l), Some(r)) => Err(RewriteError {
            // 非整除な厳密値 (例 40/3) のときは分数 + 近似 10 進を併記する
            message: format!(
                "{}: expected {}, got {}",
                label_str,
                l.fmt_exact(),
                r.fmt_exact()
            ),
            goal: goal_term,
        }),
        _ => Err(RewriteError {
            message: format!(
                "{}: cannot evaluate args to numbers (left={:?}, right={:?})",
                label_str, left_resolved, right_resolved
            ),
            goal: goal_term,
        }),
    }
}

/// goals 内の Constraint を評価し、解けたものは除去、解けないものは残す。
///
/// 各 Constraint について、左右が ArithExpr に変換できるかを実行時に判定する：
///   - 両側変換可能       → 算術連立方程式の一部として線形ソルバに渡す
///   - 変換不能（Struct/List 含む） → unify() で構造的単一化を試みる
/// 解けた束縛は shared_env にも反映され、以降のサブゴール展開で利用可能になる。
fn try_resolve_constraints(
    goals: &mut Vec<ScopedTerm>,
    shared_env: &mut ScopedEnv,
) -> Result<(), RewriteError> {
    // ディスパッチ判定（ArithExpr 変換可能か）が Var の最新束縛を見落とさないよう、
    // shared_env を反映してから判定する。Var → Struct 束縛があるとき、未解決 Var の
    // ままだと try_from_term が Ok を返してしまい誤って算術ソルバに渡されるため。
    apply_env_in_place(goals, shared_env);

    // 1. unifyが必要な Constraint を先に処理する。
    //    インデックスを後ろから処理することで goals.remove() のずれを防ぐ。
    let mut unify_indices = Vec::new();
    for (i, goal) in goals.iter().enumerate() {
        if let Term::Constraint { left, right } = goal {
            let left_arith = ArithExpr::try_from_term(left).is_ok();
            let right_arith = ArithExpr::try_from_term(right).is_ok();
            if !(left_arith && right_arith) {
                unify_indices.push(i);
            }
        }
    }
    for &idx in unify_indices.iter().rev() {
        let g = goals.remove(idx);
        let (left, right) = match g {
            Term::Constraint { left, right } => (*left, *right),
            _ => unreachable!(),
        };
        // unify は left/right を消費する。エラー時の goal 復元用に clone しておくが、
        // 関数冒頭の apply_env_in_place で既に env 反映済みなので resolve は不要。
        let original_goal = Term::Constraint {
            left: Box::new(left.clone()),
            right: Box::new(right.clone()),
        };
        unify(left, right, shared_env).map_err(|e| RewriteError {
            message: format!("unify failed: {}", e.message),
            goal: original_goal,
        })?;
    }
    if !unify_indices.is_empty() {
        // unify で得た束縛を残りの goals に反映
        for g in goals.iter_mut() {
            *g = resolve(g, shared_env);
        }
    }

    // 2. 残った Constraint（両側が ArithExpr に変換可能なもの）を線形ソルバに渡す。
    let mut eqs = Vec::new();
    let mut constraint_indices = Vec::new();
    for (i, goal) in goals.iter().enumerate() {
        if let Term::Constraint { left, right } = goal {
            let left_expr = ArithExpr::try_from_term(left);
            let right_expr = ArithExpr::try_from_term(right);
            if let (Ok(l), Ok(r)) = (left_expr, right_expr) {
                eqs.push(ArithEq::new(l, r));
                constraint_indices.push(i);
            }
        }
    }

    if eqs.is_empty() {
        return Ok(());
    }

    let result = solve_constraints(eqs).map_err(|msg| {
        let idx = constraint_indices[0];
        RewriteError {
            message: format!("constraint contradiction: {}", msg),
            goal: goals.remove(idx),
        }
    })?;

    if !result.bindings.is_empty() || result.fully_resolved {
        // Constraint内のVarからname→scopeマッピングを収集
        let mut var_scopes: HashMap<String, ScopeId> = HashMap::new();
        for &idx in &constraint_indices {
            collect_var_scopes_from_term(&goals[idx], &mut var_scopes);
        }

        for &idx in constraint_indices.iter().rev() {
            goals.remove(idx);
        }
        if !result.bindings.is_empty() {
            let mut scoped_env = ScopedEnv::new();
            for (var_name, value) in &result.bindings {
                if let Some(&scope) = var_scopes.get(var_name) {
                    // Phase 3: Term::Number も Rational を持つので無損失で格納できる。
                    let term_value = number(value.clone());
                    scoped_env.insert(scope, var_name.clone(), term_value.clone());
                    // shared_env にも反映: 制約は解消されて消えるので、束縛を共有 env に残さないと
                    // 後続のサブゴール展開で同じ Var を解決できなくなる。
                    shared_env.insert(scope, var_name.clone(), term_value);
                }
            }
            for goal in goals.iter_mut() {
                *goal = resolve(goal, &scoped_env);
            }
        }
    }

    Ok(())
}

fn assign_scope_to_clause(
    clause: Clause,
    scope_id: ScopeId,
    env: &mut ScopedEnv,
) -> Clause<ScopeId> {
    match clause {
        Clause::Fact(term) => Clause::Fact(assign_scope_to_term(term, scope_id, env)),
        Clause::Rule { head, body } => Clause::Rule {
            head: assign_scope_to_term(head, scope_id, env),
            body: body
                .into_iter()
                .map(|t| assign_scope_to_term(t, scope_id, env))
                .collect(),
        },
        Clause::Use { path, expose, span } => Clause::Use { path, expose, span },
    }
}

fn assign_scope_to_term(term: Term, scope_id: ScopeId, env: &mut ScopedEnv) -> ScopedTerm {
    match term {
        Term::Var {
            name,
            default_value,
            min,
            max,
            span,
            ..
        } => {
            // (scope, name) ごとの annotation を env に集約する。複数 instance が
            // 同一論理変数なら順次 merge され、UF env が source of truth になる。
            if default_value.is_some() || min.is_some() || max.is_some() || span.is_some() {
                env.merge_annotation(
                    scope_id,
                    &name,
                    VarAnnotation {
                        default_value: default_value.clone(),
                        min: min.clone(),
                        max: max.clone(),
                        span,
                    },
                );
            }
            Term::Var {
                name,
                scope: scope_id,
                default_value,
                min,
                max,
                span,
            }
        }
        Term::Number { value } => Term::Number { value },
        Term::InfixExpr { op, left, right } => Term::InfixExpr {
            op,
            left: Box::new(assign_scope_to_term(*left, scope_id, env)),
            right: Box::new(assign_scope_to_term(*right, scope_id, env)),
        },
        Term::Struct {
            functor,
            args,
            span,
        } => Term::Struct {
            functor,
            args: args
                .into_iter()
                .map(|a| assign_scope_to_term(a, scope_id, env))
                .collect(),
            span,
        },
        Term::List { items, tail } => Term::List {
            items: items
                .into_iter()
                .map(|i| assign_scope_to_term(i, scope_id, env))
                .collect(),
            tail: tail.map(|t| Box::new(assign_scope_to_term(*t, scope_id, env))),
        },
        Term::StringLit { value } => Term::StringLit { value },
        Term::Constraint { left, right } => Term::Constraint {
            left: Box::new(assign_scope_to_term(*left, scope_id, env)),
            right: Box::new(assign_scope_to_term(*right, scope_id, env)),
        },
    }
}

/// 単一の項をルールとマッチさせ、マッチすれば(書き換え後の項, 置換適用済みbody)を返す
/// マッチしなければNoneを返す
fn try_rewrite_single_with_result(
    db: &[Clause],
    clause_counter: &mut usize,
    term: &ScopedTerm,
    other_goals: &mut Vec<ScopedTerm>,
    shared_env: &mut ScopedEnv,
) -> Option<(ScopedTerm, Vec<ScopedTerm>)> {
    for clause in db.iter() {
        *clause_counter += 1;
        let scoped =
            assign_scope_to_clause(clause.clone(), *clause_counter, shared_env);
        let (head, body) = match scoped {
            Clause::Fact(t) => (t, vec![]),
            Clause::Rule { head, body } => (head, body),
            Clause::Use { .. } => continue,
        };

        let mut trial_env = shared_env.clone();
        if let Ok(constraints) = unify(term.clone(), head, &mut trial_env) {
            *shared_env = trial_env;
            let resolved_term = resolve(term, shared_env);
            let resolved_body: Vec<ScopedTerm> =
                body.iter().map(|b| resolve(b, shared_env)).collect();
            *other_goals = other_goals.iter().map(|g| resolve(g, shared_env)).collect();
            let resolved_constraints: Vec<ScopedTerm> = constraints
                .into_iter()
                .map(|c| resolve(&c, shared_env))
                .collect();
            other_goals.extend(resolved_constraints);
            return Some((resolved_term, resolved_body));
        }
    }
    None
}

/// 項を深さ優先で再帰的に書き換える
/// 書き換えが成功すれば書き換え後の項のリストを返す（複数になる場合がある）
/// other_goals は書き換え中に発生した変数束縛を反映するため
fn rewrite_term_recursive(
    db: &[Clause],
    clause_counter: &mut usize,
    term: ScopedTerm,
    other_goals: &mut Vec<ScopedTerm>,
    shared_env: &mut ScopedEnv,
) -> Result<Vec<ScopedTerm>, RewriteError> {
    let mut term = term;
    apply_default_var_bindings(&mut term, other_goals);

    // assert_eq(Expected, Actual) / assert_eq(Expected, Actual, "label"):
    // 両辺を数値に評価して比較。一致なら成功（結果空）、不一致なら RewriteError。
    if let Term::Struct { functor, args, span } = &term {
        if functor == "assert_eq" && (args.len() == 2 || args.len() == 3) {
            return handle_assert_eq(args, *span, shared_env);
        }
    }

    // range付きVarがゴールとして出現: 値の範囲チェックのみ行い、結果は返さない
    if let Term::Var {
        default_value,
        min,
        max,
        ..
    } = &term
    {
        if min.is_some() || max.is_some() {
            if let Some(dv) = default_value {
                if !value_in_range(dv, min.as_ref(), max.as_ref()) {
                    return Err(RewriteError {
                        message: format!("value {} is out of range", dv),
                        goal: term,
                    });
                }
            }
            return Ok(vec![]);
        }
    }

    // ビルトインファンクターは引数を解決してそのまま返す（builtin factとのunifyを避ける）
    if let Term::Struct {
        ref functor,
        ref args,
        ..
    } = term
    {
        if is_builtin_functor_with_arity(functor, args.len()) {
            if should_resolve_args(functor) {
                let resolved =
                    resolve_builtin_fact_args(db, clause_counter, term, other_goals, shared_env)?;
                return Ok(vec![resolved]);
            } else {
                return Ok(vec![term]);
            }
        }
    }

    // まず、この項自体がルールにマッチするか試す
    if let Some((resolved_term, body)) =
        try_rewrite_single_with_result(db, clause_counter, &term, other_goals, shared_env)
    {
        if body.is_empty() {
            let functor_name = match &resolved_term {
                Term::Struct { functor, .. } => Some(functor.as_str()),
                _ => None,
            };
            let resolved_term = if functor_name.is_some_and(|f| should_resolve_args(f)) {
                resolve_builtin_fact_args(
                    db,
                    clause_counter,
                    resolved_term,
                    other_goals,
                    shared_env,
                )?
            } else {
                resolved_term
            };
            return Ok(vec![resolved_term]);
        } else {
            // Ruleにマッチ: bodyの各項を再帰的に解決。
            // remaining_body は「未処理の非 Constraint body 項」、other_goals は「外側ゴール
            // ＋未解決 Constraint」と役割を物理的に分離する。再帰呼び出しには other_goals
            // だけを渡し、binding は shared_env 経由で伝播させる。
            let mut remaining_body: Vec<ScopedTerm> = body;
            let mut all_resolved = Vec::new();

            // body と other_goals 双方から Constraint を集めて解き、束縛を shared_env に伝播。
            // 残った未解決 Constraint は other_goals に戻す。
            split_and_resolve_constraints(&mut remaining_body, other_goals, shared_env)?;
            apply_env_in_place(&mut remaining_body, shared_env);
            apply_env_in_place(other_goals, shared_env);

            while let Some(b) = remaining_body.first().cloned() {
                remaining_body.remove(0);

                // b に含まれる @-default を未処理の body 項にも事前伝播。
                // rewrite_term_recursive 内の apply_default_var_bindings は other_goals に
                // しか伝播しないため、ここで明示的に行う必要がある。
                propagate_defaults_to(&b, &mut remaining_body);

                let resolved =
                    rewrite_term_recursive(db, clause_counter, b, other_goals, shared_env)?;
                all_resolved.extend(resolved);

                // 再帰中に other_goals に新しい Constraint が追加されたり、shared_env が
                // 更新された可能性があるので、再度 Constraint を解いて未処理 body にも反映する。
                try_resolve_constraints(other_goals, shared_env)?;
                apply_env_in_place(&mut remaining_body, shared_env);
            }

            return Ok(all_resolved);
        }
    }

    // ルールにマッチしない場合、サブタームを再帰的に書き換える
    match term {
        Term::InfixExpr { op, left, right } => {
            let new_left_terms =
                rewrite_term_recursive(db, clause_counter, *left, other_goals, shared_env)?;
            let new_right_terms =
                rewrite_term_recursive(db, clause_counter, *right, other_goals, shared_env)?;

            // メタデータ(bom等)やcontrol2d/control3dをother_goalsへ分離し、シェイプだけ残す
            let (left_shapes, left_meta): (Vec<_>, Vec<_>) =
                new_left_terms.into_iter().partition(|t| {
                    !is_metadata_term(t) && !is_control_term(t)
                });
            let (right_shapes, right_meta): (Vec<_>, Vec<_>) =
                new_right_terms.into_iter().partition(|t| {
                    !is_metadata_term(t) && !is_control_term(t)
                });

            other_goals.extend(left_meta);
            other_goals.extend(right_meta);

            if left_shapes.len() != 1 || right_shapes.len() != 1 {
                return Err(RewriteError {
                    message: "InfixExpr operand resolved to multiple terms".to_string(),
                    goal: Term::InfixExpr {
                        op,
                        left: Box::new(left_shapes.into_iter().next().unwrap_or(Term::Number {
                            value: crate::rational::Rational::zero(),
                        })),
                        right: Box::new(right_shapes.into_iter().next().unwrap_or(Term::Number {
                            value: crate::rational::Rational::zero(),
                        })),
                    },
                });
            }

            let new_left = left_shapes.into_iter().next().unwrap();
            let new_right = right_shapes.into_iter().next().unwrap();

            let new_term = Term::InfixExpr {
                op,
                left: Box::new(new_left),
                right: Box::new(new_right),
            };
            // 書き換え後の項がビルトインプリミティブならOK
            if is_builtin_term(&new_term) {
                Ok(vec![new_term])
            } else {
                Err(RewriteError {
                    message: "InfixExpr contains non-builtin terms after rewriting".to_string(),
                    goal: new_term,
                })
            }
        }
        Term::Struct {
            functor,
            args,
            span,
        } => {
            if is_builtin_functor(&functor) {
                return Ok(vec![Term::Struct {
                    functor,
                    args,
                    span,
                }]);
            }
            Err(RewriteError {
                message: "no clause matches goal".to_string(),
                goal: Term::Struct {
                    functor,
                    args,
                    span,
                },
            })
        }
        // その他の項（Number, Var, List など）はそのまま
        other => {
            if is_builtin_term(&other) {
                Ok(vec![other])
            } else {
                Err(RewriteError {
                    message: "no clause matches goal".to_string(),
                    goal: other,
                })
            }
        }
    }
}

/// ビルトインファンクタの引数内にある項を1つに解決する。
/// リテラル/変数はそのまま、リストは中身を再帰的に解決、それ以外は書き換えて1つに解決する。
fn resolve_builtin_arg(
    db: &[Clause],
    clause_counter: &mut usize,
    term: ScopedTerm,
    other_goals: &mut Vec<ScopedTerm>,
    shared_env: &mut ScopedEnv,
) -> Result<ScopedTerm, RewriteError> {
    match term {
        Term::Number { .. } | Term::Var { .. } | Term::StringLit { .. } => Ok(term),
        Term::List { items, tail } => {
            let resolved_items = items
                .into_iter()
                .map(|item| resolve_builtin_arg(db, clause_counter, item, other_goals, shared_env))
                .collect::<Result<Vec<_>, _>>()?;
            Ok(Term::List {
                items: resolved_items,
                tail,
            })
        }
        Term::Struct {
            functor,
            args,
            span,
        } if is_builtin_functor(&functor) => {
            let resolved_args = args
                .into_iter()
                .map(|arg| resolve_builtin_arg(db, clause_counter, arg, other_goals, shared_env))
                .collect::<Result<Vec<_>, _>>()?;
            Ok(Term::Struct {
                functor,
                args: resolved_args,
                span,
            })
        }
        Term::InfixExpr { op, left, right } => {
            let new_left = resolve_builtin_arg(db, clause_counter, *left, other_goals, shared_env)?;
            let new_right =
                resolve_builtin_arg(db, clause_counter, *right, other_goals, shared_env)?;
            Ok(Term::InfixExpr {
                op,
                left: Box::new(new_left),
                right: Box::new(new_right),
            })
        }
        other => {
            let mut resolved =
                rewrite_term_recursive(db, clause_counter, other, other_goals, shared_env)?;
            if resolved.len() > 1 {
                let mut shape = Vec::new();
                for t in resolved {
                    if is_control_term(&t) || is_metadata_term(&t)
                    {
                        other_goals.push(t);
                    } else {
                        shape.push(t);
                    }
                }
                resolved = shape;
            }
            if resolved.len() != 1 {
                return Err(RewriteError {
                    message: "builtin argument resolved to multiple terms".to_string(),
                    goal: resolved.into_iter().next().unwrap_or(Term::Number {
                        value: crate::rational::Rational::zero(),
                    }),
                });
            }
            Ok(resolved.remove(0))
        }
    }
}

fn resolve_builtin_fact_args(
    db: &[Clause],
    clause_counter: &mut usize,
    term: ScopedTerm,
    other_goals: &mut Vec<ScopedTerm>,
    shared_env: &mut ScopedEnv,
) -> Result<ScopedTerm, RewriteError> {
    let (functor, args, span) = match term {
        Term::Struct {
            functor,
            args,
            span,
        } if is_builtin_functor(&functor) => (functor, args, span),
        other => return Ok(other),
    };

    let resolved_args = args
        .into_iter()
        .map(|arg| resolve_builtin_arg(db, clause_counter, arg, other_goals, shared_env))
        .collect::<Result<Vec<_>, _>>()?;

    Ok(Term::Struct {
        functor,
        args: resolved_args,
        span,
    })
}

pub fn execute(
    db: &mut [Clause],
    query: Vec<Term>,
) -> Result<(Vec<ScopedTerm>, ScopedEnv), RewriteError> {
    let mut clause_counter: usize = 0;
    let mut shared_env = ScopedEnv::new();
    let mut results = Vec::new();
    let mut db_with_builtins = db.to_vec();
    db_with_builtins.extend(builtin_cad_facts());

    let scoped_query: Vec<ScopedTerm> = query
        .into_iter()
        .map(|t| assign_scope_to_term(t, 0, &mut shared_env))
        .collect();

    for term in scoped_query {
        let mut other_goals = Vec::new();
        let resolved = rewrite_term_recursive(
            &db_with_builtins,
            &mut clause_counter,
            term,
            &mut other_goals,
            &mut shared_env,
        )?;
        results.extend(resolved);
        results.extend(other_goals);

        // 各ゴールの rewrite 後に制約解決し、得られた束縛を後続に伝播
        try_resolve_constraints(&mut results, &mut shared_env)?;
    }

    // 解決済み Constraint を結果から除去
    results.retain(|t| !matches!(t, Term::Constraint { .. }));

    Ok((results, shared_env))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{arith_expr, database, query, struc, var};
    use crate::rational::Rational;

    fn var_with_range(name: &str, min: Option<Bound>, max: Option<Bound>) -> Term {
        Term::Var {
            name: name.to_string(),
            scope: (),
            default_value: None,
            min,
            max,
            span: None,
        }
    }

    /// テスト用: unscoped Term → scope 0 の ScopedTerm に変換
    fn scoped(term: Term) -> ScopedTerm {
        let mut env = ScopedEnv::new();
        assign_scope_to_term(term, 0, &mut env)
    }

    fn run_success(db_src: &str, query_src: &str) -> Vec<String> {
        let mut db = database(db_src).expect("failed to parse db");
        let q = query(query_src).expect("failed to parse query").1;
        let (resolved, _env) = execute(&mut db, q).expect("Expected success");
        resolved.iter().map(|t| format!("{:?}", t)).collect()
    }

    fn run_failure(db_src: &str, query_src: &str) {
        let mut db = database(db_src).expect("failed to parse db");
        let q = query(query_src).expect("failed to parse query").1;
        assert!(
            execute(&mut db, q).is_err(),
            "Expected failure, got success"
        );
    }

    // ===== unify tests =====

    #[test]
    fn test_unify_vars() {
        let x = scoped(var("X".to_string()));
        let a = scoped(struc("a".to_string(), vec![]));
        let mut env = ScopedEnv::new();
        unify(x, a.clone(), &mut env).unwrap();
        assert_eq!(resolve(&scoped(var("X".to_string())), &env), a);
    }

    #[test]
    fn test_unify_structs() {
        let t1 = scoped(struc("f".to_string(), vec![var("X".to_string())]));
        let t2 = scoped(struc("f".to_string(), vec![struc("a".to_string(), vec![])]));
        let mut env = ScopedEnv::new();
        unify(t1, t2, &mut env).unwrap();
        assert_eq!(
            resolve(&scoped(var("X".to_string())), &env),
            scoped(struc("a".to_string(), vec![]))
        );
    }

    #[test]
    fn test_unify_fail() {
        let t1 = scoped(struc("f".to_string(), vec![]));
        let t2 = scoped(struc("g".to_string(), vec![]));
        assert!(unify(t1, t2, &mut ScopedEnv::new()).is_err());
    }

    // ===== RangeVar unify tests =====

    // ===== arithmetic tests =====

    #[test]
    fn test_arith_simple_add() {
        use crate::parse::number_int;
        let expr =
            crate::parse::arith_expr(crate::parse::ArithOp::Add, number_int(3), number_int(5));
        let n = number_int(8);
        assert!(unify(scoped(expr), scoped(n), &mut ScopedEnv::new()).is_ok());
    }

    #[test]
    fn test_arith_simple_sub() {
        use crate::parse::number_int;
        let expr =
            crate::parse::arith_expr(crate::parse::ArithOp::Sub, number_int(10), number_int(3));
        let n = number_int(7);
        assert!(unify(scoped(expr), scoped(n), &mut ScopedEnv::new()).is_ok());
    }

    #[test]
    fn test_arith_simple_mul() {
        use crate::parse::number_int;
        let expr =
            crate::parse::arith_expr(crate::parse::ArithOp::Mul, number_int(4), number_int(5));
        let n = number_int(20);
        assert!(unify(scoped(expr), scoped(n), &mut ScopedEnv::new()).is_ok());
    }

    #[test]
    fn test_arith_simple_div() {
        use crate::parse::{number, number_int};
        let expr =
            crate::parse::arith_expr(crate::parse::ArithOp::Div, number_int(10), number_int(3));
        // 10/3 は厳密な有理数として表現される
        let n = number(crate::rational::Rational::from_ratio(10, 3));
        assert!(unify(scoped(expr), scoped(n), &mut ScopedEnv::new()).is_ok());
    }

    #[test]
    fn test_arith_in_rule() {
        let resolved = run_success("cube(3, 7, 3).", "cube(3, 10-3, 3).");
        assert_eq!(resolved, vec!["cube(3, 7, 3)"]);
    }

    #[test]
    fn test_arith_with_var() {
        let resolved = run_success("f(5, 5).", "f(X, 10 - X).");
        assert_eq!(resolved, vec!["f(5, 5)"]);
    }

    #[test]
    fn test_arith_expr_before_var() {
        let resolved = run_success("f(10, 5).", "f(X * 2, X).");
        assert_eq!(resolved, vec!["f(10, 5)"]);
    }

    #[test]
    fn test_arith_multiple_vars_order() {
        let resolved = run_success("f(3, 1, 2).", "f(X + Y, X, Y).");
        assert_eq!(resolved, vec!["f(3, 1, 2)"]);
    }

    #[test]
    fn test_arith_precedence() {
        let resolved = run_success("result(14).", "result(2 + 3 * 4).");
        assert_eq!(resolved, vec!["result(14)"]);
    }

    #[test]
    fn test_arith_compound_expr() {
        let resolved = run_success("f(10, 2, 3).", "f((X + Y) * 2, X, Y).");
        assert_eq!(resolved, vec!["f(10, 2, 3)"]);
    }

    #[test]
    fn test_arith_nested_expr() {
        let resolved = run_success("f(25, 3, 4).", "f(X * X + Y * Y, X, Y).");
        assert_eq!(resolved, vec!["f(25, 3, 4)"]);
    }

    #[test]
    fn test_arith_both_sides_expr() {
        let resolved = run_success("f(5).", "f(X).");
        assert_eq!(resolved, vec!["f(5)"]);
    }

    #[test]
    fn default_var_matches_annotated_value() {
        let resolved = run_success("f(25).", "f(X@25).");
        assert_eq!(resolved, vec!["f(X@25)"]);
    }

    #[test]
    fn default_var_conflict_fails() {
        run_failure("f(30).", "f(X@25).");
    }

    #[test]
    fn default_var_propagates_within_rule_body() {
        let resolved = run_success(
            "cut(W) :- cube(W, 50, 260). main :- cube(X@25, 50, 300) - (cut(W@5) |> translate(p(0, 0, 0), p(X / 2 - W, 0, 0))).",
            "main.",
        );
        assert_eq!(
            resolved,
            vec![
                "(cube(X@25, 50, 300) - translate(cube(5, 50, 260), p(0, 0, 0), p(7.5, 0, 0)))"
            ]
        );
    }

    // ===== basic fact tests =====

    #[test]
    fn simple_atom_match() {
        run_success("hello.", "hello.");
    }

    #[test]
    fn fail_unmatched() {
        run_failure("hello.", "bye.");
    }

    #[test]
    fn db_var_matches_constant_query() {
        run_success("honi(X).", "honi(fuwa).");
    }

    #[test]
    fn query_var_binds_to_constant_fact() {
        let resolved = run_success("honi(fuwa).", "honi(X).");
        assert_eq!(resolved, vec!["honi(fuwa)"]);
    }

    #[test]
    fn var_to_var_binding() {
        // DBの変数とクエリの変数がマッチ -> resolved goalでは変数のまま
        let resolved = run_success("honi(X).", "honi(Y).");
        // Y_1のような変数名になる
        assert!(resolved[0].starts_with("honi("));
    }

    #[test]
    fn multiple_usages_of_same_variable() {
        let resolved = run_success("likes(X, X).", "likes(fuwa, Y).");
        assert_eq!(resolved, vec!["likes(fuwa, fuwa)"]);
    }

    // ===== nested struct tests =====

    #[test]
    fn deep_struct_on_db() {
        let resolved = run_success("a(b(c)).", "a(X).");
        assert_eq!(resolved, vec!["a(b(c))"]);
    }

    #[test]
    fn deep_struct_on_query() {
        let resolved = run_success("a(X).", "a(b(c)).");
        assert_eq!(resolved, vec!["a(b(c))"]);
    }

    #[test]
    fn recursive_unify_nested_struct_match() {
        let resolved = run_success("f(a(b)).", "f(a(b)).");
        assert_eq!(resolved, vec!["f(a(b))"]);
    }

    #[test]
    fn recursive_unify_nested_struct_mismatch_inner() {
        run_failure("f(a(b)).", "f(a(c)).");
    }

    #[test]
    fn recursive_unify_nested_struct_mismatch_functor() {
        run_failure("f(a(b)).", "f(c(b)).");
    }

    #[test]
    fn recursive_unify_var_in_nested_struct() {
        let resolved = run_success("f(a(X)).", "f(a(b)).");
        assert_eq!(resolved, vec!["f(a(b))"]);
    }

    #[test]
    fn recursive_unify_query_var_binds_in_nested() {
        let resolved = run_success("f(a(b)).", "f(a(X)).");
        assert_eq!(resolved, vec!["f(a(b))"]);
    }

    #[test]
    fn recursive_unify_multiple_args() {
        let resolved = run_success("f(a(b), c(d)).", "f(a(b), c(d)).");
        assert_eq!(resolved, vec!["f(a(b), c(d))"]);
    }

    #[test]
    fn recursive_unify_multiple_args_one_mismatch() {
        run_failure("f(a(b), c(d)).", "f(a(b), c(e)).");
    }

    #[test]
    fn recursive_unify_three_levels_deep() {
        let resolved = run_success("f(a(b(c))).", "f(a(b(c))).");
        assert_eq!(resolved, vec!["f(a(b(c)))"]);
    }

    #[test]
    fn recursive_unify_three_levels_deep_mismatch() {
        run_failure("f(a(b(c))).", "f(a(b(d))).");
    }

    #[test]
    fn recursive_unify_var_at_deep_level() {
        let resolved = run_success("f(a(b(X))).", "f(a(b(c))).");
        assert_eq!(resolved, vec!["f(a(b(c)))"]);
    }

    // ===== rule tests =====

    #[test]
    fn resolved_goals_returned() {
        // Ruleにマッチするとheadがbodyで置換される
        let resolved = run_success("p :- q(X, Z), r(Z, Y). q(a, b). r(b, c).", "p.");
        assert_eq!(resolved, vec!["q(a, b)", "r(b, c)"]);
    }

    #[test]
    fn rule_single_goal() {
        let resolved = run_success("parent(X) :- father(X). father(tom).", "parent(tom).");
        assert_eq!(resolved, vec!["father(tom)"]);
    }

    #[test]
    fn rule_single_goal_with_var_query() {
        let resolved = run_success("parent(X) :- father(X). father(tom).", "parent(Y).");
        assert_eq!(resolved, vec!["father(tom)"]);
    }

    #[test]
    fn grandparent_rule() {
        let db = r#"
            parent(alice, bob).
            parent(bob, carol).
            grandparent(X, Y) :- parent(X, Z), parent(Z, Y).
        "#;
        let resolved = run_success(db, "grandparent(alice, Who).");
        assert_eq!(resolved, vec!["parent(alice, bob)", "parent(bob, carol)"]);
    }

    // ===== list tests =====

    #[test]
    fn list_empty_match() {
        let resolved = run_success("f([]).", "f([]).");
        assert_eq!(resolved, vec!["f([])"]);
    }

    #[test]
    fn list_simple_match() {
        let resolved = run_success("f([a, b, c]).", "f([a, b, c]).");
        assert_eq!(resolved, vec!["f([a, b, c])"]);
    }

    #[test]
    fn list_mismatch() {
        run_failure("f([a, b]).", "f([a, c]).");
    }

    #[test]
    fn list_var_binding() {
        let resolved = run_success("f([a, b, c]).", "f(X).");
        assert_eq!(resolved, vec!["f([a, b, c])"]);
    }
    #[test]
    fn list_traverse() {
        let resolved = run_success("f([]). f([X|T]) :- f(T).", "f([a, b, c]).");
        assert_eq!(resolved, vec!["f([])"]);
    }

    #[test]
    fn list_head_tail_pattern() {
        let resolved = run_success("f([a, b, c]).", "f([H|T]).");
        assert_eq!(resolved, vec!["f([a | [b, c]])"]);
    }

    #[test]
    fn member_first() {
        let resolved = run_success("member(X, [X|_]).", "member(a, [a, b, c]).");
        assert_eq!(resolved, vec!["member(a, [a, b, c])"]);
    }

    // ===== rule failure tests =====

    #[test]
    fn rule_fails_first_subgoal() {
        run_failure("p(X) :- q(X), r(X). q(b). r(a).", "p(a).");
    }

    #[test]
    fn rule_fails_second_subgoal() {
        run_failure("p(X) :- q(X), r(X). q(a). r(b).", "p(a).");
    }

    #[test]
    fn rule_fails_no_matching_fact() {
        run_failure("p(X) :- q(X). q(a).", "p(b).");
    }

    // ===== rule chain tests =====

    #[test]
    fn rule_chain_two_levels() {
        let resolved = run_success("a(X) :- b(X). b(X) :- c(X). c(foo).", "a(foo).");
        assert_eq!(resolved, vec!["c(foo)"]);
    }

    #[test]
    fn rule_chain_three_levels() {
        let resolved = run_success(
            "a(X) :- b(X). b(X) :- c(X). c(X) :- d(X). d(bar).",
            "a(bar).",
        );
        assert_eq!(resolved, vec!["d(bar)"]);
    }

    #[test]
    fn rule_chain_with_var_binding() {
        let resolved = run_success("a(X) :- b(X). b(X) :- c(X). c(baz).", "a(Y).");
        assert_eq!(resolved, vec!["c(baz)"]);
    }

    // ===== rule with nested struct tests =====

    #[test]
    fn rule_with_nested_struct_in_fact() {
        let resolved = run_success(
            "outer(X) :- inner(X). inner(pair(a, b)).",
            "outer(pair(a, b)).",
        );
        assert_eq!(resolved, vec!["inner(pair(a, b))"]);
    }

    #[test]
    fn rule_with_nested_struct_var_binding() {
        let resolved = run_success("outer(X) :- inner(X). inner(pair(a, b)).", "outer(Y).");
        assert_eq!(resolved, vec!["inner(pair(a, b))"]);
    }

    #[test]
    fn rule_with_deeply_nested_struct() {
        let resolved = run_success(
            "wrap(X) :- data(X). data(node(leaf(a), leaf(b))).",
            "wrap(node(leaf(a), leaf(b))).",
        );
        assert_eq!(resolved, vec!["data(node(leaf(a), leaf(b)))"]);
    }

    #[test]
    fn rule_shared_variable_in_body() {
        let resolved = run_success("same(X) :- eq(X, X). eq(a, a).", "same(a).");
        assert_eq!(resolved, vec!["eq(a, a)"]);
    }

    // ===== rule with multiple args =====

    #[test]
    fn rule_three_args() {
        let resolved = run_success(
            "triple(X, Y, Z) :- first(X), second(Y), third(Z). first(a). second(b). third(c).",
            "triple(A, B, C).",
        );
        assert_eq!(resolved, vec!["first(a)", "second(b)", "third(c)"]);
    }

    // ===== rule head with struct =====

    #[test]
    fn rule_head_with_struct() {
        let resolved = run_success(
            "make_pair(pair(X, Y)) :- left(X), right(Y). left(a). right(b).",
            "make_pair(pair(a, b)).",
        );
        assert_eq!(resolved, vec!["left(a)", "right(b)"]);
    }

    #[test]
    fn rule_head_with_struct_var_query() {
        let resolved = run_success(
            "make_pair(pair(X, Y)) :- left(X), right(Y). left(a). right(b).",
            "make_pair(P).",
        );
        assert_eq!(resolved, vec!["left(a)", "right(b)"]);
    }

    #[test]
    fn arith_with_user_defined_rule() {
        // ob :- cube(1,1,1). main :- ob + cube(2,2,2).
        // ob should be resolved to cube(1,1,1), then the whole thing becomes cube + cube
        let resolved = run_success("ob :- cube(1,1,1). main :- ob + cube(2,2,2).", "main.");
        assert_eq!(resolved, vec!["(cube(1, 1, 1) + cube(2, 2, 2))"]);
    }

    #[test]
    fn arith_with_chained_rules() {
        // ob :- foo. foo :- cube(1,1,1). main :- ob + cube(2,2,2).
        let resolved = run_success(
            "ob :- foo. foo :- cube(1,1,1). main :- ob + cube(2,2,2).",
            "main.",
        );
        assert_eq!(resolved, vec!["(cube(1, 1, 1) + cube(2, 2, 2))"]);
    }

    #[test]
    fn arith_with_pipe_and_rule() {
        // ob :- cube(1,1,1) |> translate(p(0,0,0), p(10,0,0)). main :- ob + cube(2,2,2).
        let resolved = run_success(
            "ob :- cube(1,1,1) |> translate(p(0,0,0), p(10,0,0)). main :- ob + cube(2,2,2).",
            "main.",
        );
        assert_eq!(
            resolved,
            vec![
                "(translate(cube(1, 1, 1), p(0, 0, 0), p(10, 0, 0)) + cube(2, 2, 2))"
            ]
        );
    }

    #[test]
    fn arith_with_builtin_arg_clause_reference() {
        let resolved = run_success(
            "cub :- cube(40,90,50). main :- (cub - rotate(cub, 0, 30, 0)).",
            "main.",
        );
        assert_eq!(
            resolved,
            vec!["(cube(40, 90, 50) - rotate(cube(40, 90, 50), 0, 30, 0))"]
        );
    }

    #[test]
    fn builtin_arg_rule_with_control_separation() {
        let resolved = run_success(
            "blade_cut :- path(p(0, 0), [line_to(p(10, 0)), line_to(p(10, 20))]), control3d(p(X@0, Y@20, 0)). main :- linear_extrude(blade_cut, 100).",
            "main.",
        );
        assert_eq!(resolved.len(), 2);
        assert!(resolved[0].starts_with("linear_extrude(path("));
        assert!(resolved[1].starts_with("control3d("));
    }

    #[test]
    fn builtin_arg_rule_with_bom_separation() {
        let resolved = run_success(
            "part(X) :- cube(X, 10, 10), bom(\"cube\", []). main :- translate(part(5), p(0, 0, 0), p(10, 0, 0)).",
            "main.",
        );
        assert_eq!(resolved.len(), 2);
        assert!(resolved[0].starts_with("translate(cube("));
        assert!(resolved[1].starts_with("bom("));
    }

    #[test]
    fn infix_expr_with_bom_separation() {
        let resolved = run_success(
            "part :- cube(10, 10, 10), bom(\"cube\", []). main :- part + cube(5, 5, 5).",
            "main.",
        );
        assert_eq!(resolved.len(), 2);
        assert!(resolved[0].contains("+"));
        assert!(resolved[1].starts_with("bom("));
    }

    #[test]
    fn pipe_with_bom_separation() {
        let resolved = run_success(
            "part(X) :- cube(X, 10, 10), bom(\"cube\", []). main :- part(5) |> translate(p(0, 0, 0), p(10, 0, 0)).",
            "main.",
        );
        assert_eq!(resolved.len(), 2);
        assert!(resolved[0].starts_with("translate(cube("));
        assert!(resolved[1].starts_with("bom("));
    }

    #[test]
    fn constraint_solving_binds_variable() {
        let resolved = run_success("box(X+Y, Y) :- cube(X, Y, 10).", "box(7, 3).");
        assert_eq!(resolved, vec!["cube(4, 3, 10)"]);
    }

    #[test]
    fn constraint_contradiction_detected() {
        let mut db = database("f(X+Y, Y) :- h(X), g(Y). h(4). g(3).").expect("parse db");
        let q = query("f(100, 3).").expect("parse query").1;
        let result = execute(&mut db, q);
        assert!(result.is_err());
    }

    #[test]
    fn body_range_constraint() {
        let resolved = run_success("foo(X) :- 0<X<10, cube(X, X, X).", "foo(5).");
        assert_eq!(resolved, vec!["cube(5, 5, 5)"]);
    }

    #[test]
    fn body_eq_constraint_with_rule() {
        let resolved = run_success(
            "cut(SLIT, W, H) :- X=(W-SLIT)/2, sketch(p(X, 0), [line_to(p(SLIT+W, 0)), line_to(p(SLIT+W, H-20)), line_to(p(X, H-20))]).",
            "cut(18, 40, 120).",
        );
        assert_eq!(
            resolved,
            vec!["sketch(p(11, 0), [line_to(p(58, 0)), line_to(p(58, 100)), line_to(p(11, 100))])"]
        );
    }

    #[test]
    fn constraint_solving_does_not_leak_across_scopes() {
        // f で X+Y=7, Y=3 → X=4。body は g(4, Z=2)。
        // g(A, X) :- cube(A, X, X) で X_scope2 は 2 になるべき。
        // scope leak があると X_scope2 が 4 に汚染されて失敗する。
        let resolved = run_success(
            "f(X+Y, Y, Z) :- g(X, Z). g(A, X) :- cube(A, X, X).",
            "f(7, 3, 2).",
        );
        assert_eq!(resolved, vec!["cube(4, 2, 2)"]);
    }

    #[test]
    fn compute_term_range_add() {
        use crate::parse::{ArithOp, Bound};
        let term = arith_expr(
            ArithOp::Add,
            var_with_range(
                "X",
                Some(Bound {
                    value: crate::rational::Rational::from_integer(0),
                    inclusive: false,
                }),
                Some(Bound {
                    value: crate::rational::Rational::from_integer(10),
                    inclusive: false,
                }),
            ),
            var_with_range(
                "Y",
                Some(Bound {
                    value: crate::rational::Rational::from_integer(0),
                    inclusive: false,
                }),
                Some(Bound {
                    value: crate::rational::Rational::from_integer(5),
                    inclusive: false,
                }),
            ),
        );
        let (min, max) = compute_term_range(&term).unwrap();
        assert_eq!(min.unwrap().value, crate::rational::Rational::from_integer(0));
        assert_eq!(max.unwrap().value, crate::rational::Rational::from_integer(15));
    }

    #[test]
    fn compute_term_range_sub() {
        use crate::parse::{ArithOp, Bound};
        let term = arith_expr(
            ArithOp::Sub,
            var_with_range(
                "X",
                Some(Bound {
                    value: crate::rational::Rational::from_integer(0),
                    inclusive: false,
                }),
                Some(Bound {
                    value: crate::rational::Rational::from_integer(10),
                    inclusive: false,
                }),
            ),
            var_with_range(
                "Y",
                Some(Bound {
                    value: crate::rational::Rational::from_integer(0),
                    inclusive: false,
                }),
                Some(Bound {
                    value: crate::rational::Rational::from_integer(5),
                    inclusive: false,
                }),
            ),
        );
        let (min, max) = compute_term_range(&term).unwrap();
        assert_eq!(min.unwrap().value, crate::rational::Rational::from_integer(-5));
        assert_eq!(max.unwrap().value, crate::rational::Rational::from_integer(10));
    }

    #[test]
    fn infer_query_param_ranges_from_body() {
        use crate::parse::{Bound, collect_query_params};
        let db_src = "box(X) :- 0<X<100, cube(X, 10, 10).";
        let query_src = "box(A).";
        let db = database(db_src).expect("failed to parse db");
        let (_, query_terms) = query(query_src).expect("failed to parse query");
        let mut params = collect_query_params(&query_terms);
        assert_eq!(params.len(), 1);
        infer_query_param_ranges(&query_terms, &db, &mut params).unwrap();
        assert_eq!(
            params[0].min.clone().unwrap(),
            Bound {
                value: crate::rational::Rational::from_integer(0),
                inclusive: false
            }
        );
        assert_eq!(
            params[0].max.clone().unwrap(),
            Bound {
                value: crate::rational::Rational::from_integer(100),
                inclusive: false
            }
        );
    }

    #[test]
    fn infer_query_param_ranges_infix_from_body() {
        use crate::parse::{Bound, collect_query_params};
        let db_src = "box(X + Y) :- 0<X<10, 0<Y<5, cube(X, Y, 10).";
        let query_src = "box(A).";
        let db = database(db_src).expect("failed to parse db");
        let (_, query_terms) = query(query_src).expect("failed to parse query");
        let mut params = collect_query_params(&query_terms);
        assert_eq!(params.len(), 1);
        infer_query_param_ranges(&query_terms, &db, &mut params).unwrap();
        assert_eq!(
            params[0].min.clone().unwrap(),
            Bound {
                value: crate::rational::Rational::from_integer(0),
                inclusive: false
            }
        );
        assert_eq!(
            params[0].max.clone().unwrap(),
            Bound {
                value: crate::rational::Rational::from_integer(15),
                inclusive: false
            }
        );
    }

    #[test]
    fn rule_body_with_constraints_does_not_panic() {
        // 再帰中の制約解消で temp_other_goals が縮むケース + 本体内サブゴール展開時に
        // @-default が constraint に伝播するケースをまとめてカバーする。
        let db_src = "\
main :- battery_box.\n\
battery_box :- ((cube(X@20, Y@18, OUTLEN@58)\n\
                - wire_hole(X/2, WH_Y1, 0)\n\
                - wire_hole(X/2, WH_Y2, 0)\n\
                ) |> translate(p(0, 0, 0), p(0-X/2, 0-Y/2, 0))),\n\
                WH_Y1 = Y/4, WH_Y2 = Y*3/4.\n\
wire_hole(X, Y, H) :- cylinder(0.4, 20) |> translate(p(0, 0, 0), p(X, Y, H)).\n\
";
        let mut db = database(db_src).expect("failed to parse db");
        let (_, query_terms) = query("main.").expect("failed to parse query");
        let (results, _) = execute(&mut db, query_terms).expect("execute should succeed");
        // Model3D::from_term が Var を見つけたら Err(UnboundVariable(...)) を返す。
        for t in &results {
            let r = crate::manifold_bridge::Model3D::from_term(t);
            assert!(
                !matches!(
                    r,
                    Err(crate::manifold_bridge::ConversionError::UnboundVariable(_))
                ),
                "term has unbound variable: term={:?} err={:?}",
                t,
                r.err()
            );
        }
    }

    #[test]
    fn query_body_range_propagation() {
        use crate::parse::{Bound, collect_query_params};
        let db_src = "main(X) :- 0<X<10, cube(X, 10, 10).";
        let query_src = "main(A).";
        let db = database(db_src).expect("failed to parse db");
        let (_, query_terms) = query(query_src).expect("failed to parse query");
        let mut params = collect_query_params(&query_terms);
        assert_eq!(params.len(), 1);
        assert!(params[0].min.is_none());
        infer_query_param_ranges(&query_terms, &db, &mut params).unwrap();
        assert_eq!(
            params[0].min.clone().unwrap(),
            Bound {
                value: crate::rational::Rational::from_integer(0),
                inclusive: false,
            }
        );
        assert_eq!(
            params[0].max.clone().unwrap(),
            Bound {
                value: crate::rational::Rational::from_integer(10),
                inclusive: false,
            }
        );
    }

    // ===== 機構モデリングを支える基礎挙動のテスト =====
    // body の `=` の動的ディスパッチ、リスト要素変数の共有、線形ソルバの
    // 歯車比解決、距離整合の二乗式、範囲注釈付きデフォルトの伝播が揃って
    // 機構ライブラリ (`gear pair` の噛み合い検証) の前提となる。

    /// body の `[1, 2, 3] = [A, B, C]` は構造的単一化として処理され、A=1, B=2, C=3 を束縛する。
    #[test]
    fn body_eq_unifies_lists() {
        let mut db =
            database("test :- L = [1, 2, 3], L = [A, B, C], cube(A, B, C).").expect("parse db");
        let q = query("test.").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        assert_eq!(strs, vec!["cube(1, 2, 3)"]);
    }

    /// body で構造体を変数に束縛し、後続の `=` で各フィールドを取り出せる。
    /// 機構ライブラリで `G = gp(_, M, Z, _, _, _, _)` のように歯車のフィールド分解に使う。
    #[test]
    fn body_eq_unifies_structs() {
        let mut db = database(
            "test :- G = gp(g1, 1, 20, X@10, Y@20, T@5, 5),\n\
                     G = gp(_, M, Z, _, _, _, _),\n\
                     cube(M, Z, T).",
        )
        .expect("parse db");
        let q = query("test.").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        assert_eq!(strs, vec!["cube(1, 20, 5)"]);
    }

    /// `=` の左右が parse 時には両方 Var でも、ランタイムで Struct に束縛されれば
    /// 単一化として処理される。`try_resolve_constraints` での実行時ディスパッチの動作確認。
    #[test]
    fn body_eq_var_to_var_unifies_when_struct_bound_at_runtime() {
        let mut db = database(
            "mesh(GA, GB) :- GA = GB.\n\
             test :- mesh(gp(g1, 1, M, X, Y), gp(g1, 1, 20, 30, 40)),\n\
                     cube(M, X, Y).\n",
        )
        .expect("parse db");
        let q = query("test.").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        assert_eq!(strs, vec!["cube(20, 30, 40)"]);
    }

    /// リスト要素として埋め込まれた変数が、複数の述語呼び出しを跨いで束縛を共有する。
    /// 機構ライブラリでは `[gp(g1,...,T1,...), gp(g2,...,T2,...), ...]` をリストで持ち回り、
    /// 噛み合い制約と描画の両方で同じ T1, T2 が見えることが必須。
    ///
    /// constrain で B = 7, C = 3 が束縛される。
    /// 続く show でも同じ B, C が値として展開され、cylinder(7, 7) / cylinder(3, 3) が出る。
    #[test]
    fn list_element_var_shared_across_predicates() {
        let db_src = "\
constrain([_]).\n\
constrain([A, B | Rest]) :- pair_eq(A, B), constrain([B | Rest]).\n\
pair_eq(item(_, X), item(_, Y)) :- X + Y = 10.\n\
show([]).\n\
show([item(_, X) | R]) :- cylinder(X, X), show(R).\n\
test :- constrain([item(a, 3), item(b, B), item(c, C)]),\n\
        show([item(a, 3), item(b, B), item(c, C)]).\n";
        let mut db = database(db_src).expect("parse db");
        let q = query("test.").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        assert!(
            strs.iter().any(|s| s == "cylinder(7, 7)"),
            "B=7 should propagate through show: got {:?}",
            strs
        );
        assert!(
            strs.iter().any(|s| s == "cylinder(3, 3)"),
            "C=3 should propagate through show: got {:?}",
            strs
        );
    }

    /// 線形ソルバが歯車比 `Z1*T1 + Z2*T2 = 0` を T2 について解く。
    /// Z1, Z2, T1 が既知なら T2 = -(Z1*T1)/Z2。
    /// Z1=20, Z2=40, T1=5 なら T2 = -2.5。
    #[test]
    fn linear_solver_solves_gear_ratio_for_t2() {
        let db_src = "test(T2) :- Z1 = 20, Z2 = 40, T1 = 5,\n\
                                 Z1 * T1 + Z2 * T2 = 0,\n\
                                 cylinder(T2, T2).\n";
        let mut db = database(db_src).expect("parse db");
        let q = query("test(T).").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        assert_eq!(strs, vec!["cylinder(-2.5, -2.5)"]);
    }

    /// 中心距離整合: `DX*DX + DY*DY = (M*(Z1+Z2)/2)^2` が定数だけで成り立つ場合は通る。
    /// 二乗形式を採るのは sqrt が無いため。M=1, Z1=20, Z2=40, X2-X1=30, Y2-Y1=0 では
    /// 30*30 + 0 = 900 = 30*30 で整合する。
    #[test]
    fn gear_distance_squared_form_matches() {
        let db_src = "test :- M = 1, Z1 = 20, Z2 = 40,\n\
                              X1 = 0, Y1 = 0, X2 = 30, Y2 = 0,\n\
                              DX = X2 - X1, DY = Y2 - Y1,\n\
                              DistSq = DX*DX + DY*DY,\n\
                              R = M*(Z1+Z2)/2,\n\
                              DistSq = R*R,\n\
                              cube(M, Z1, Z2).\n";
        let mut db = database(db_src).expect("parse db");
        let q = query("test.").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        assert_eq!(strs, vec!["cube(1, 20, 40)"]);
    }

    /// 中心距離不整合: 距離が規格と合わなければ算術矛盾で実行が失敗する。
    /// X2=31 (期待値 30) のとき 31*31 = 961 ≠ 900 = 30*30。
    #[test]
    fn gear_distance_mismatch_yields_contradiction() {
        let db_src = "test :- M = 1, Z1 = 20, Z2 = 40,\n\
                              X1 = 0, Y1 = 0, X2 = 31, Y2 = 0,\n\
                              DX = X2 - X1, DY = Y2 - Y1,\n\
                              DistSq = DX*DX + DY*DY,\n\
                              R = M*(Z1+Z2)/2,\n\
                              DistSq = R*R,\n\
                              cube(M, Z1, Z2).\n";
        let mut db = database(db_src).expect("parse db");
        let q = query("test.").expect("parse query").1;
        let err = execute(&mut db, q).expect_err("distance mismatch should fail");
        assert!(
            err.error_message().contains("contradiction"),
            "expected constraint contradiction, got: {}",
            err.error_message()
        );
    }

    /// 範囲注釈付きデフォルトの線形伝播: `-180 < T1 @ 0 < 180` で T1 にデフォルト 0 と
    /// 範囲制約を与え、`T2 = 0 - T1` で T2 が連動する。query で T1=30 を渡すと T2=-30。
    /// GUI でスライダーから T1 を変えた時の連動回転に対応。
    #[test]
    fn range_annotated_default_propagates_linearly() {
        let db_src = "test(T1) :- -180 < T1 @ 0 < 180,\n\
                                  T2 = 0 - T1,\n\
                                  cylinder(5, T2).\n";
        let mut db = database(db_src).expect("parse db");
        let q = query("test(30).").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        assert_eq!(strs, vec!["cylinder(5, -30)"]);
    }

    /// `assert_eq(L, R)` は両辺が等しければ結果に何も残さず通過する。
    #[test]
    fn assert_eq_passes_when_equal() {
        let mut db = database(
            "test :- X = 30, assert_eq(X*X, 900), cube(X, X, X).",
        )
        .expect("parse db");
        let q = query("test.").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        assert_eq!(strs, vec!["cube(30, 30, 30)"]);
    }

    /// `assert_eq(L, R)` は両辺が異なれば RewriteError を返す。
    #[test]
    fn assert_eq_fails_when_unequal() {
        let mut db = database("test :- assert_eq(2 + 2, 5), cube(1, 1, 1).")
            .expect("parse db");
        let q = query("test.").expect("parse query").1;
        let err = execute(&mut db, q).expect_err("assert_eq mismatch should fail");
        assert!(
            err.error_message().contains("expected 4")
                && err.error_message().contains("got 5"),
            "expected message to mention expected/got values: {}",
            err.error_message()
        );
    }

    /// 3 引数版 `assert_eq(L, R, "label")` のラベルがエラーメッセージに含まれる。
    /// 機構ライブラリで「中心距離整合」「歯車比」など人間に読めるエラーを出すために使う。
    #[test]
    fn assert_eq_includes_label_in_error_message() {
        let mut db = database(
            "test :- M = 1, Z1 = 20, Z2 = 40, X1 = 0, X2 = 31, Y1 = 0, Y2 = 0,\n\
                     DX = X2 - X1, DY = Y2 - Y1,\n\
                     R = M*(Z1+Z2)/2,\n\
                     assert_eq(DX*DX + DY*DY, R*R, \"gear center distance\"),\n\
                     cube(M, Z1, Z2).",
        )
        .expect("parse db");
        let q = query("test.").expect("parse query").1;
        let err = execute(&mut db, q).expect_err("distance mismatch should fail");
        assert!(
            err.error_message().contains("gear center distance"),
            "expected label in error: {}",
            err.error_message()
        );
        // 数値も含まれるはず: 期待 900, 実測 961
        assert!(
            err.error_message().contains("900") && err.error_message().contains("961"),
            "expected expected/got values in message: {}",
            err.error_message()
        );
    }

    /// 線形ソルバが、ユーザ呼び出し側で1つの変数が定数に束縛されたときに、
    /// 別ルールの body 内の制約 (`A*X + B*Y = 0`) を Y について解けることを確認する。
    /// 機構ライブラリの連鎖伝播で必要な挙動。
    #[test]
    fn linear_solver_propagates_through_rule_call() {
        let mut db = database(
            "foo(T1, T2) :- 40*T1 + 20*T2 = 0.\n\
             test :- T1 = -10, foo(T1, T2), cylinder(T2, T2).",
        )
        .expect("parse db");
        let q = query("test.").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        // T1 = -10, 40*(-10) + 20*T2 = 0 → T2 = 20
        assert_eq!(strs, vec!["cylinder(20, 20)"]);
    }

    /// 線形ソルバの丸め許容: 非整除な除算でも、近似値で変数を束縛し連鎖を継続できる。
    /// `30 * T2 = 400` のような完全整除しないケースで T2 = 13.33 (本来 13.333...) を返す。
    /// 機構ライブラリで歯数比が割り切れない組み合わせ (Z1*T1 / Z2 が非整数) でも
    /// 駆動角伝播ができるようにする。
    #[test]
    fn linear_solver_rounds_non_exact_division() {
        let mut db = database(
            "foo(T1, T2) :- 40*T1 + 30*T2 = 0.\n\
             test :- T1 = -10, foo(T1, T2), cylinder(T2, T2).",
        )
        .expect("parse db");
        let q = query("test.").expect("parse query").1;
        let (resolved, _) = execute(&mut db, q).expect("execute should succeed");
        let strs: Vec<String> = resolved.iter().map(|t| format!("{:?}", t)).collect();
        // 40*(-10) + 30*T2 = 0 → T2 = 400/30 = 13.333... → FixedPoint 13.33
        assert_eq!(strs, vec!["cylinder(13.33, 13.33)"]);
    }

    /// assert_eq の不整合エラーが、非整除な値を含む場合に分数表記を併記する。
    /// 旧 FixedPoint では `13.33` のような丸め値しか表示できず、誤差由来の偽の
    /// 不整合と本物の不整合の区別が難しかった。Rational + fmt_exact なら
    /// `40/3 (≈ 13.33)` のように厳密値が見える。
    #[test]
    fn assert_eq_error_shows_exact_form_for_non_finite_decimal() {
        // 40/3 != 14 の不整合。期待値 40/3 (= 13.33...) が分数で表示される。
        let mut db = database("test :- M = 40, D = 3, assert_eq(M/D, 14, \"ratio\").")
            .expect("parse db");
        let q = query("test.").expect("parse query").1;
        let err = execute(&mut db, q).expect_err("should fail");
        let msg = err.error_message();
        assert!(
            msg.contains("40/3") && msg.contains("13.33"),
            "expected exact form '40/3 (≈ 13.33)' in: {}",
            msg
        );
    }

    /// 引数が未束縛のままだと、評価不能エラーが出る。
    /// 単に黙って通り抜けないことを保証する。
    #[test]
    fn assert_eq_fails_when_args_unevaluable() {
        let mut db = database("test :- assert_eq(X, 5, \"unbound\").").expect("parse db");
        let q = query("test.").expect("parse query").1;
        let err = execute(&mut db, q).expect_err("unbound arg should fail");
        assert!(
            err.error_message().contains("cannot evaluate"),
            "expected 'cannot evaluate' in: {}",
            err.error_message()
        );
    }
}
