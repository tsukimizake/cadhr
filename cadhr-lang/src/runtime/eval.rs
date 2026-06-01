//! 正格評価器。`Module` の top-level decl をすべて評価し、`Env` (識別子 → 値) を
//! 完成させる。`run_main(env, inputs)` で main を呼ぶ。
//!
//! 式レベルの構文 (Lit / Var / Ctor / App / Lambda / Let / If / Case / BinOp /
//! List / Record / RecordUpdate / Field / Range / Negate) を実装。
//! ADT コンストラクタを `Value::Ctor` として表現し、`case` で pattern match する。

use crate::diagnostic::{Diagnostic, Span};
use crate::module::{LoadedModule, ResolvedUnit};
use crate::runtime::builtin::{BuiltinEval, BuiltinEvalRegistry};
use crate::runtime::value::{ClosureBody, Env, Value};
use crate::syntax::ast::*;
use std::collections::HashMap;
use std::rc::Rc;

pub struct Evaluator<'r> {
    pub builtins: &'r BuiltinEvalRegistry,
    /// ユーザー定義 ADT のコンストラクタ arity (引数 0 個も「裸の値」として扱う)。
    pub ctor_arities: std::collections::HashMap<String, usize>,
}

impl<'r> Evaluator<'r> {
    pub fn new(builtins: &'r BuiltinEvalRegistry) -> Self {
        Self {
            builtins,
            ctor_arities: Default::default(),
        }
    }

    /// 解決済み複数モジュール (`ResolvedUnit`) を依存順に評価し、
    /// 各モジュールの完成 env を `qualified_name → Env` のマップで返す。
    /// main モジュール (`unit.main_index`) は空名 `""` で引ける。
    pub fn eval_unit(
        &mut self,
        unit: &ResolvedUnit,
    ) -> Result<HashMap<String, Rc<Env>>, Vec<Diagnostic>> {
        // ADT ctor arity を全モジュールから集める (qualified / unqualified 両方)
        for lm in &unit.modules {
            for d in &lm.module.decls {
                if let Decl::Type(t) = d {
                    for c in &t.constructors {
                        self.ctor_arities.insert(c.name.clone(), c.args.len());
                        if !lm.qualified_name.is_empty() {
                            self.ctor_arities
                                .insert(qualify_name(&lm.qualified_name, &c.name), c.args.len());
                        }
                    }
                }
            }
        }

        // 各モジュールの「local 名 → Value」テーブル (import 公開時に再利用)
        let mut module_locals: HashMap<String, HashMap<String, Value>> = HashMap::new();
        let mut module_envs: HashMap<String, Rc<Env>> = HashMap::new();
        let mut all_diag: Vec<Diagnostic> = Vec::new();

        for lm in &unit.modules {
            match self.eval_one_module(lm, &module_locals, &mut module_envs) {
                Ok(locals) => {
                    module_locals.insert(lm.qualified_name.clone(), locals);
                }
                Err(diags) => all_diag.extend(diags),
            }
        }

        if all_diag.is_empty() {
            Ok(module_envs)
        } else {
            Err(all_diag)
        }
    }

    fn eval_one_module(
        &self,
        lm: &LoadedModule,
        module_locals: &HashMap<String, HashMap<String, Value>>,
        module_envs: &mut HashMap<String, Rc<Env>>,
    ) -> Result<HashMap<String, Value>, Vec<Diagnostic>> {
        let mut env = Env::new();
        let mut locals: HashMap<String, Value> = HashMap::new();

        // 1. builtins (全モジュール共通)
        for (name, b) in &self.builtins.by_name {
            env = env.extend(
                name,
                Value::Builtin {
                    name: name.to_string(),
                    arity: b.arity,
                    args: Vec::new(),
                },
            );
        }

        // 2. 自モジュールの ADT ctor (unqualified + qualified)
        for d in &lm.module.decls {
            if let Decl::Type(t) = d {
                for c in &t.constructors {
                    let v = make_ctor_value(c);
                    env = env.extend(&c.name, v.clone());
                    if !lm.qualified_name.is_empty() {
                        env = env
                            .extend(&qualify_name(&lm.qualified_name, &c.name), v.clone());
                    }
                    locals.insert(c.name.clone(), v);
                }
            }
        }

        // 3. import 解決
        for imp in &lm.module.imports {
            let imp_qname = crate::module::join_name(&imp.module);
            let imp_locals = module_locals.get(&imp_qname).cloned().ok_or_else(|| {
                vec![Diagnostic::error(
                    imp.span,
                    format!("import 先 `{imp_qname}` がまだロードされていません (resolver bug?)"),
                )]
            })?;
            let prefix = imp.alias.clone().unwrap_or_else(|| imp_qname.clone());

            // qualified: prefix.{name} で常に引けるようにする
            for (k, v) in &imp_locals {
                env = env.extend(&qualify_name(&prefix, k), v.clone());
            }

            // unqualified: exposing に従って晒す
            match &imp.exposing {
                Some(Exposing::All(_)) => {
                    for (k, v) in &imp_locals {
                        env = env.extend(k, v.clone());
                    }
                }
                Some(Exposing::Some(items, _)) => {
                    for item in items {
                        if let Some(v) = imp_locals.get(&item.name) {
                            env = env.extend(&item.name, v.clone());
                        }
                    }
                }
                None => {}
            }
        }

        // 4. local value decl を順次評価
        for d in &lm.module.decls {
            if let Decl::Value(v) = d {
                let val = self
                    .eval_value_decl(v, Rc::new(env.clone()))
                    .map_err(|e| vec![e])?;
                env = env.extend(&v.name, val.clone());
                if !lm.qualified_name.is_empty() {
                    env = env.extend(&qualify_name(&lm.qualified_name, &v.name), val.clone());
                }
                locals.insert(v.name.clone(), val);
            }
        }

        module_envs.insert(lm.qualified_name.clone(), Rc::new(env));
        Ok(locals)
    }

    /// 1 つのモジュールを評価し、top-level の `Env` を返す。
    /// signature decl は無視する (型推論は別レイヤで済んでいる前提)。
    pub fn eval_module(&mut self, m: &Module) -> Result<Rc<Env>, Vec<Diagnostic>> {
        // ADT コンストラクタの arity を先に集める
        for d in &m.decls {
            if let Decl::Type(t) = d {
                for c in &t.constructors {
                    self.ctor_arities.insert(c.name.clone(), c.args.len());
                }
            }
        }

        // builtin を環境に入れる
        let mut env = Env::new();
        for (name, b) in &self.builtins.by_name {
            env = env.extend(
                name,
                Value::Builtin {
                    name: name.to_string(),
                    arity: b.arity,
                    args: Vec::new(),
                },
            );
        }
        // ADT コンストラクタを「累積適用前の Builtin」として注入
        // (引数 0 の場合は裸の値 Value::Ctor をそのまま入れる)
        let ctor_names: Vec<(String, usize)> = self
            .ctor_arities
            .iter()
            .map(|(n, a)| (n.clone(), *a))
            .collect();
        for (name, arity) in &ctor_names {
            if *arity == 0 {
                env = env.extend(
                    name,
                    Value::Ctor {
                        tag: name.clone(),
                        args: Vec::new(),
                    },
                );
            } else {
                // 引数 1 個以上のコンストラクタは「builtin like」で curried 蓄積
                env = env.extend(
                    name,
                    Value::Builtin {
                        name: format!("__ctor_{name}"),
                        arity: *arity,
                        args: Vec::new(),
                    },
                );
            }
        }

        // 値定義を順番に評価
        let mut diag = Vec::new();
        for d in &m.decls {
            if let Decl::Value(v) = d {
                match self.eval_value_decl(v, Rc::new(env.clone())) {
                    Ok(value) => {
                        env = env.extend(&v.name, value);
                    }
                    Err(e) => diag.push(e),
                }
            }
        }
        if diag.is_empty() {
            Ok(Rc::new(env))
        } else {
            Err(diag)
        }
    }

    fn eval_value_decl(&self, v: &ValueDecl, env: Rc<Env>) -> Result<Value, Diagnostic> {
        if v.params.is_empty() {
            self.eval_expr(&v.body, &env)
        } else {
            // 関数定義: closure を作る
            Ok(Value::Closure {
                params: v.params.clone(),
                body: ClosureBody::Expr(Rc::new(v.body.clone())),
                env,
            })
        }
    }

    pub fn eval_expr(&self, e: &Expr, env: &Rc<Env>) -> Result<Value, Diagnostic> {
        match e {
            Expr::Lit(l, _) => Ok(lit_to_value(l)),
            Expr::Var { module, name, span } => {
                let key = qualify(module.as_ref(), name);
                env.lookup(&key).cloned().ok_or_else(|| {
                    Diagnostic::error(*span, format!("評価時に未定義の変数 `{key}`"))
                })
            }
            Expr::Ctor { module, name, span } => {
                let key = qualify(module.as_ref(), name);
                env.lookup(&key).cloned().ok_or_else(|| {
                    Diagnostic::error(*span, format!("評価時に未定義のコンストラクタ `{key}`"))
                })
            }
            Expr::List(items, _) => {
                let vs = items
                    .iter()
                    .map(|i| self.eval_expr(i, env))
                    .collect::<Result<Vec<_>, _>>()?;
                Ok(Value::List(vs))
            }
            Expr::Record(fields, _) => {
                let fs = fields
                    .iter()
                    .map(|f| Ok((f.name.clone(), self.eval_expr(&f.value, env)?)))
                    .collect::<Result<Vec<_>, _>>()?;
                Ok(Value::Record(fs))
            }
            Expr::RecordUpdate { base, updates, span } => {
                let base_v = self.eval_expr(base, env)?;
                let mut fields = match base_v {
                    Value::Record(fs) => fs,
                    _ => {
                        return Err(Diagnostic::error(
                            *span,
                            "record でない値の update はできません".to_string(),
                        ))
                    }
                };
                for u in updates {
                    let nv = self.eval_expr(&u.value, env)?;
                    if let Some(p) = fields.iter_mut().find(|(n, _)| n == &u.name) {
                        p.1 = nv;
                    } else {
                        return Err(Diagnostic::error(
                            u.span,
                            format!("update 対象に field `{}` がありません", u.name),
                        ));
                    }
                }
                Ok(Value::Record(fields))
            }
            Expr::Field { receiver, name, span } => {
                let v = self.eval_expr(receiver, env)?;
                match v {
                    Value::Record(fs) => fs
                        .into_iter()
                        .find(|(n, _)| n == name)
                        .map(|(_, v)| v)
                        .ok_or_else(|| {
                            Diagnostic::error(
                                *span,
                                format!("record に field `{name}` がありません"),
                            )
                        }),
                    other => Err(Diagnostic::error(
                        *span,
                        format!("record でない値 `{other}` から `.{name}` を取れません"),
                    )),
                }
            }
            Expr::App { func, arg, span } => {
                let f = self.eval_expr(func, env)?;
                let a = self.eval_expr(arg, env)?;
                self.apply(f, a, *span)
            }
            Expr::Lambda { params, body, .. } => Ok(Value::Closure {
                params: params.clone(),
                body: ClosureBody::Expr(Rc::new((**body).clone())),
                env: env.clone(),
            }),
            Expr::Let { bindings, body, .. } => {
                let mut env_acc = env.clone();
                for b in bindings {
                    let v = self.eval_value_decl(b, env_acc.clone())?;
                    env_acc = Rc::new(env_acc.extend(&b.name, v));
                }
                self.eval_expr(body, &env_acc)
            }
            Expr::If {
                cond,
                then_branch,
                else_branch,
                span,
            } => {
                let c = self.eval_expr(cond, env)?;
                match c {
                    Value::Bool(true) => self.eval_expr(then_branch, env),
                    Value::Bool(false) => self.eval_expr(else_branch, env),
                    _ => Err(Diagnostic::error(
                        *span,
                        "if の条件は Bool である必要があります".to_string(),
                    )),
                }
            }
            Expr::Case { scrutinee, arms, span } => {
                let v = self.eval_expr(scrutinee, env)?;
                for arm in arms {
                    if let Some(bindings) = match_pattern(&arm.pattern, &v) {
                        let arm_env = bindings
                            .into_iter()
                            .fold((**env).clone(), |env, (n, v)| env.extend(&n, v));
                        return self.eval_expr(&arm.body, &Rc::new(arm_env));
                    }
                }
                Err(Diagnostic::error(
                    *span,
                    format!("case: マッチするパターンがありません ({v})"),
                ))
            }
            Expr::BinOp {
                op, left, right, span,
            } => match op {
                BinOp::ApplyR => {
                    // `a |> f` → f a
                    let a = self.eval_expr(left, env)?;
                    let f = self.eval_expr(right, env)?;
                    self.apply(f, a, *span)
                }
                BinOp::ApplyL => {
                    // `f <| a` → f a
                    let f = self.eval_expr(left, env)?;
                    let a = self.eval_expr(right, env)?;
                    self.apply(f, a, *span)
                }
                _ => {
                    let l = self.eval_expr(left, env)?;
                    let r = self.eval_expr(right, env)?;
                    eval_binop(*op, l, r, *span)
                }
            },
            Expr::Negate(inner, span) => {
                let v = self.eval_expr(inner, env)?;
                match v {
                    Value::Int(n) => Ok(Value::Int(-n)),
                    Value::Float(x) => Ok(Value::Float(-x)),
                    _ => Err(Diagnostic::error(
                        *span,
                        format!("negate: 数値が必要ですが {v} でした"),
                    )),
                }
            }
            Expr::Range { lo, hi, span } => {
                let l = self.eval_expr(lo, env)?;
                let h = self.eval_expr(hi, env)?;
                let (lo_f, hi_f, is_int) = match (&l, &h) {
                    (Value::Int(a), Value::Int(b)) => (*a as f64, *b as f64, true),
                    (Value::Float(a), Value::Float(b)) => (*a, *b, false),
                    (Value::Int(a), Value::Float(b)) => (*a as f64, *b, false),
                    (Value::Float(a), Value::Int(b)) => (*a, *b as f64, false),
                    _ => {
                        return Err(Diagnostic::error(
                            *span,
                            format!("range: 数値の両端が必要ですが {l}..{h} でした"),
                        ))
                    }
                };
                Ok(Value::Range {
                    lo: lo_f,
                    hi: hi_f,
                    is_int,
                })
            }
            Expr::Error(span) => Err(Diagnostic::error(
                *span,
                "パースエラー由来の式は評価できません".to_string(),
            )),
        }
    }

    /// 関数適用 (curried)。Closure / Builtin / ADT コンストラクタの 3 パターン。
    pub fn apply(&self, f: Value, a: Value, span: Span) -> Result<Value, Diagnostic> {
        match f {
            Value::Closure { params, body, env } => {
                let mut bindings: Vec<(String, Value)> = Vec::new();
                if let Some(b) = match_pattern(&params[0], &a) {
                    bindings.extend(b);
                } else {
                    return Err(Diagnostic::error(
                        span,
                        "関数引数のパターンマッチに失敗".to_string(),
                    ));
                }
                let new_env = bindings
                    .into_iter()
                    .fold((*env).clone(), |env, (n, v)| env.extend(&n, v));
                let new_env = Rc::new(new_env);
                if params.len() == 1 {
                    let expr = match body {
                        ClosureBody::Expr(e) => e,
                    };
                    self.eval_expr(&expr, &new_env)
                } else {
                    // 部分適用 — 残りの params で新しい closure を作る
                    Ok(Value::Closure {
                        params: params[1..].to_vec(),
                        body,
                        env: new_env,
                    })
                }
            }
            Value::Builtin { name, arity, mut args } => {
                args.push(a);
                if args.len() == arity {
                    // ADT コンストラクタなら Value::Ctor、それ以外なら registry を引く
                    if let Some(ctor_name) = name.strip_prefix("__ctor_") {
                        Ok(Value::Ctor {
                            tag: ctor_name.to_string(),
                            args,
                        })
                    } else {
                        let b: &BuiltinEval = self.builtins.get(&name).ok_or_else(|| {
                            Diagnostic::error(span, format!("未登録の builtin `{name}`"))
                        })?;
                        (b.eval)(&args)
                            .map_err(|e| Diagnostic::error(span, format!("{name}: {e}")))
                    }
                } else {
                    Ok(Value::Builtin { name, arity, args })
                }
            }
            other => Err(Diagnostic::error(
                span,
                format!("関数でない値 `{other}` に引数を適用できません"),
            )),
        }
    }
}

/// `Some(ModuleName)` を持つ Var/Ctor を `Foo.bar` 形式の lookup キーに正規化する。
fn qualify(module: Option<&ModuleName>, name: &str) -> String {
    match module {
        Some(m) if !m.segments.is_empty() => format!("{}.{}", m.segments.join("."), name),
        _ => name.to_string(),
    }
}

fn qualify_name(module: &str, name: &str) -> String {
    if module.is_empty() {
        name.to_string()
    } else {
        format!("{module}.{name}")
    }
}

fn make_ctor_value(c: &Constructor) -> Value {
    if c.args.is_empty() {
        Value::Ctor {
            tag: c.name.clone(),
            args: Vec::new(),
        }
    } else {
        Value::Builtin {
            name: format!("__ctor_{}", c.name),
            arity: c.args.len(),
            args: Vec::new(),
        }
    }
}

fn lit_to_value(l: &Lit) -> Value {
    match l {
        Lit::Int(n) => Value::Int(*n),
        Lit::Float(x) => Value::Float(*x),
        Lit::String(s) => Value::String(s.clone()),
        Lit::Bool(b) => Value::Bool(*b),
    }
}

/// pattern が value にマッチするか試し、マッチしたら束縛リストを返す。
pub fn match_pattern(p: &Pattern, v: &Value) -> Option<Vec<(String, Value)>> {
    match (p, v) {
        (Pattern::Var(n, _), _) => Some(vec![(n.clone(), v.clone())]),
        (Pattern::Wildcard(_), _) => Some(vec![]),
        (Pattern::Lit(l, _), v) => match (l, v) {
            (Lit::Int(a), Value::Int(b)) if a == b => Some(vec![]),
            (Lit::Float(a), Value::Float(b)) if a == b => Some(vec![]),
            (Lit::String(a), Value::String(b)) if a == b => Some(vec![]),
            (Lit::Bool(a), Value::Bool(b)) if a == b => Some(vec![]),
            _ => None,
        },
        (
            Pattern::Ctor {
                name, args: pargs, ..
            },
            Value::Ctor { tag, args: vargs },
        ) if name == tag && pargs.len() == vargs.len() => {
            let mut out = Vec::new();
            for (p, v) in pargs.iter().zip(vargs.iter()) {
                let b = match_pattern(p, v)?;
                out.extend(b);
            }
            Some(out)
        }
        (Pattern::List(items, _), Value::List(vs)) if items.len() == vs.len() => {
            let mut out = Vec::new();
            for (p, v) in items.iter().zip(vs.iter()) {
                let b = match_pattern(p, v)?;
                out.extend(b);
            }
            Some(out)
        }
        (Pattern::Cons { head, tail, .. }, Value::List(vs)) if !vs.is_empty() => {
            let mut out = match_pattern(head, &vs[0])?;
            let rest = Value::List(vs[1..].to_vec());
            let tail_b = match_pattern(tail, &rest)?;
            out.extend(tail_b);
            Some(out)
        }
        (Pattern::As { inner, name, .. }, v) => {
            let mut b = match_pattern(inner, v)?;
            b.push((name.clone(), v.clone()));
            Some(b)
        }
        (Pattern::Record(fields, _), Value::Record(vs)) => {
            let mut out = Vec::new();
            for f in fields {
                let (_, v) = vs.iter().find(|(n, _)| n == f)?;
                out.push((f.clone(), v.clone()));
            }
            Some(out)
        }
        _ => None,
    }
}

fn eval_binop(op: BinOp, l: Value, r: Value, span: Span) -> Result<Value, Diagnostic> {
    use BinOp::*;
    match op {
        Add | Sub | Mul | Div => {
            let (la, ra) = num_pair(&l, &r, span)?;
            let v = match op {
                Add => la + ra,
                Sub => la - ra,
                Mul => la * ra,
                Div => {
                    if ra == 0.0 {
                        return Err(Diagnostic::error(span, "0 除算".to_string()));
                    }
                    la / ra
                }
                _ => unreachable!(),
            };
            // 入力が両方 Int なら Int を返す (実装簡略化: + - * は Int-keeps、/ は Float)
            match (&l, &r, op) {
                (Value::Int(_), Value::Int(_), Add | Sub | Mul) => Ok(Value::Int(v as i64)),
                _ => Ok(Value::Float(v)),
            }
        }
        Eq => Ok(Value::Bool(value_eq(&l, &r))),
        NotEq => Ok(Value::Bool(!value_eq(&l, &r))),
        Lt | Le | Gt | Ge => {
            let (la, ra) = num_pair(&l, &r, span)?;
            let b = match op {
                Lt => la < ra,
                Le => la <= ra,
                Gt => la > ra,
                Ge => la >= ra,
                _ => unreachable!(),
            };
            Ok(Value::Bool(b))
        }
        And => match (&l, &r) {
            (Value::Bool(a), Value::Bool(b)) => Ok(Value::Bool(*a && *b)),
            _ => Err(Diagnostic::error(span, "&& は Bool を要求".to_string())),
        },
        Or => match (&l, &r) {
            (Value::Bool(a), Value::Bool(b)) => Ok(Value::Bool(*a || *b)),
            _ => Err(Diagnostic::error(span, "|| は Bool を要求".to_string())),
        },
        Cons => match r {
            Value::List(mut vs) => {
                vs.insert(0, l);
                Ok(Value::List(vs))
            }
            _ => Err(Diagnostic::error(
                span,
                ":: は List を右辺に要求".to_string(),
            )),
        },
        Append => match (l, r) {
            (Value::List(mut a), Value::List(b)) => {
                a.extend(b);
                Ok(Value::List(a))
            }
            _ => Err(Diagnostic::error(
                span,
                "++ は List 同士を要求".to_string(),
            )),
        },
        ApplyR => Err(Diagnostic::error(
            span,
            "|> は構文的に App として処理されるべき".to_string(),
        )),
        ApplyL => Err(Diagnostic::error(
            span,
            "<| は構文的に App として処理されるべき".to_string(),
        )),
        Compose | ComposeR => Err(Diagnostic::error(
            span,
            "<< / >> は当面サポート外".to_string(),
        )),
    }
}

fn num_pair(l: &Value, r: &Value, span: Span) -> Result<(f64, f64), Diagnostic> {
    let lf = match l {
        Value::Int(n) => *n as f64,
        Value::Float(x) => *x,
        _ => {
            return Err(Diagnostic::error(
                span,
                format!("数値が必要ですが左辺は {l}"),
            ))
        }
    };
    let rf = match r {
        Value::Int(n) => *n as f64,
        Value::Float(x) => *x,
        _ => {
            return Err(Diagnostic::error(
                span,
                format!("数値が必要ですが右辺は {r}"),
            ))
        }
    };
    Ok((lf, rf))
}

fn value_eq(a: &Value, b: &Value) -> bool {
    match (a, b) {
        (Value::Int(x), Value::Int(y)) => x == y,
        (Value::Float(x), Value::Float(y)) => x == y,
        (Value::String(x), Value::String(y)) => x == y,
        (Value::Bool(x), Value::Bool(y)) => x == y,
        (Value::Ctor { tag: t1, args: a1 }, Value::Ctor { tag: t2, args: a2 }) => {
            t1 == t2 && a1.len() == a2.len() && a1.iter().zip(a2.iter()).all(|(x, y)| value_eq(x, y))
        }
        (Value::List(xs), Value::List(ys)) => {
            xs.len() == ys.len() && xs.iter().zip(ys.iter()).all(|(x, y)| value_eq(x, y))
        }
        _ => false,
    }
}

/// 高レベル API: `main` を呼び出して `Value` を返す (引数を Vec で受ける形)。
pub fn run_main(env: &Env, args: Vec<Value>) -> Result<Value, Diagnostic> {
    let main = env
        .lookup("main")
        .cloned()
        .ok_or_else(|| Diagnostic::error(Span::empty(), "main 関数が定義されていません".to_string()))?;
    let mut cur = main;
    let evaluator_dummy = BuiltinEvalRegistry::new(); // dummy: apply は builtins を使わない closure case のみ
    let evaluator = Evaluator::new(&evaluator_dummy);
    let _ = evaluator;
    // 引数を一つずつ適用
    // 実際には Evaluator が必要 — 呼び出し側で持つ。ここはあくまでヘルパ。
    for a in args {
        cur = match cur {
            Value::Closure {
                params,
                body,
                env: cenv,
            } => {
                let mut bindings: Vec<(String, Value)> = Vec::new();
                if let Some(b) = match_pattern(&params[0], &a) {
                    bindings.extend(b);
                } else {
                    return Err(Diagnostic::error(
                        Span::empty(),
                        "main 引数のパターンマッチ失敗".to_string(),
                    ));
                }
                let new_env = bindings
                    .into_iter()
                    .fold((*cenv).clone(), |env, (n, v)| env.extend(&n, v));
                let new_env = Rc::new(new_env);
                if params.len() == 1 {
                    let expr = match body {
                        ClosureBody::Expr(e) => e,
                    };
                    // evaluator 必要 — run_main は使われない簡易ヘルパとして残す
                    let _ = expr;
                    return Err(Diagnostic::error(
                        Span::empty(),
                        "run_main は再帰的 eval を呼べないため Evaluator 経由で使ってください"
                            .to_string(),
                    ));
                } else {
                    Value::Closure {
                        params: params[1..].to_vec(),
                        body,
                        env: new_env,
                    }
                }
            }
            _ => break,
        };
    }
    Ok(cur)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runtime::builtin::registry as eval_registry;
    use crate::syntax::parse::parse;

    fn eval_src(src: &str) -> (Rc<Env>, Vec<Diagnostic>) {
        let m = parse(src).unwrap();
        let reg = eval_registry();
        let mut ev = Evaluator::new(&reg);
        match ev.eval_module(&m) {
            Ok(env) => (env, vec![]),
            Err(diag) => (Rc::new(Env::new()), diag),
        }
    }

    fn val_of<'a>(env: &'a Env, name: &str) -> &'a Value {
        env.lookup(name).expect("name not found")
    }

    #[test]
    fn eval_int() {
        let (env, diag) = eval_src("x = 42");
        assert!(diag.is_empty());
        assert!(matches!(val_of(&env, "x"), Value::Int(42)));
    }

    #[test]
    fn eval_float_arith() {
        let (env, diag) = eval_src("x = 1.0 + 2.0 * 3.0");
        assert!(diag.is_empty());
        assert!(matches!(val_of(&env, "x"), Value::Float(7.0)));
    }

    #[test]
    fn eval_let() {
        let (env, diag) = eval_src("x = let a = 1 in let b = 2 in a + b");
        assert!(diag.is_empty());
        assert!(matches!(val_of(&env, "x"), Value::Int(3)));
    }

    #[test]
    fn eval_lambda_app() {
        let (env, diag) = eval_src("f = \\x y -> x + y\ng = f 1 2");
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert!(matches!(val_of(&env, "g"), Value::Int(3)));
    }

    #[test]
    fn eval_pipe() {
        let (env, diag) = eval_src("f x = x + 1\ng = 5 |> f");
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert!(matches!(val_of(&env, "g"), Value::Int(6)));
    }

    #[test]
    fn eval_if() {
        let (env, diag) = eval_src("x = if True then 1 else 2");
        assert!(diag.is_empty());
        assert!(matches!(val_of(&env, "x"), Value::Int(1)));
    }

    #[test]
    fn eval_cube_builtin() {
        let (env, diag) = eval_src("s = cube 1.0 2.0 3.0");
        assert!(diag.is_empty(), "diag: {diag:?}");
        match val_of(&env, "s") {
            Value::Shape3D(crate::runtime::value::Model3D::Cube { x, y, z }) => {
                assert_eq!(*x, 1.0);
                assert_eq!(*y, 2.0);
                assert_eq!(*z, 3.0);
            }
            other => panic!("expected Shape3D Cube, got {other}"),
        }
    }

    #[test]
    fn eval_translate3d_chain() {
        let src = "s = cube 1.0 1.0 1.0 |> translate3d (p3 0.0 0.0 0.0) (p3 0.0 0.0 5.0)";
        let (env, diag) = eval_src(src);
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert!(matches!(val_of(&env, "s"), Value::Shape3D(_)));
    }

    #[test]
    fn eval_range_literal() {
        let (env, diag) = eval_src("r = 6.0 .. 80.0");
        assert!(diag.is_empty());
        match val_of(&env, "r") {
            Value::Range { lo, hi, .. } => {
                assert_eq!(*lo, 6.0);
                assert_eq!(*hi, 80.0);
            }
            other => panic!("expected Range, got {other}"),
        }
    }

    #[test]
    fn eval_intersect_range() {
        let (env, diag) = eval_src("r = intersect (1.0 .. 100.0) (5.0 .. 50.0)");
        assert!(diag.is_empty(), "diag: {diag:?}");
        match val_of(&env, "r") {
            Value::Range { lo, hi, .. } => {
                assert_eq!(*lo, 5.0);
                assert_eq!(*hi, 50.0);
            }
            other => panic!("expected Range, got {other}"),
        }
    }

    #[test]
    fn eval_adt_constructor() {
        let src = "type Shape = Cube Float Float Float | Sphere Float\n\
                   s = Sphere 5.0";
        let (env, diag) = eval_src(src);
        assert!(diag.is_empty(), "diag: {diag:?}");
        match val_of(&env, "s") {
            Value::Ctor { tag, args } => {
                assert_eq!(tag, "Sphere");
                assert_eq!(args.len(), 1);
            }
            other => panic!("expected Ctor, got {other}"),
        }
    }

    #[test]
    fn eval_case_pattern_match() {
        let src = "type Shape = Cube Float Float Float | Sphere Float\n\
                   describe s = case s of | Cube _ _ _ -> 1 | Sphere _ -> 2\n\
                   result = describe (Sphere 5.0)";
        let (env, diag) = eval_src(src);
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert!(matches!(val_of(&env, "result"), Value::Int(2)));
    }

    #[test]
    fn eval_record_and_field() {
        let (env, diag) = eval_src("p = { x = 1, y = 2 }\nq = p.x");
        assert!(diag.is_empty());
        assert!(matches!(val_of(&env, "q"), Value::Int(1)));
    }

    #[test]
    fn eval_list_cons_match() {
        let src = "head_or xs = case xs of | x :: _ -> x | [] -> 0\n\
                   r1 = head_or [1, 2, 3]\n\
                   r2 = head_or []";
        let (env, diag) = eval_src(src);
        assert!(diag.is_empty(), "diag: {diag:?}");
        assert!(matches!(val_of(&env, "r1"), Value::Int(1)));
        assert!(matches!(val_of(&env, "r2"), Value::Int(0)));
    }
}
