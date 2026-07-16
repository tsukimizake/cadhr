//! 式・パターンの自由変数解析。eval 層 (top-level eager の依存順評価) と
//! sema 層 (top-level の SCC ベース型推論) の両方から使う。
//!
//! - `free_var_names_in(e, bound, out)`: `e` 中の裸の `Var`参照のうち `bound` に
//!   無いものを `out` に追加する。qualified Var (`Foo.bar`) や Ctor 参照は対象外。
//! - `pattern_bound_names_into(p, out)`: パターンが束縛する名前を集める。
//!
//! lambda / let / case のスコープを正しく扱う。`let` は相互再帰として扱い、
//! 全 binding 名を bound に入れてから body と各 binding 本体を辿る。

use crate::syntax::ast::*;
use std::collections::HashSet;

/// パターンが束縛する名前を `out` に追加する。
pub fn pattern_bound_names_into(p: &Pattern, out: &mut HashSet<String>) {
    match p {
        Pattern::Var(n, _) => {
            out.insert(n.clone());
        }
        Pattern::Wildcard(_) | Pattern::Lit(_, _) => {}
        Pattern::Ctor { args, .. } => {
            for a in args {
                pattern_bound_names_into(a, out);
            }
        }
        Pattern::List(items, _) => {
            for i in items {
                pattern_bound_names_into(i, out);
            }
        }
        Pattern::Cons { head, tail, .. } => {
            pattern_bound_names_into(head, out);
            pattern_bound_names_into(tail, out);
        }
        Pattern::Record(fields, _) => {
            for f in fields {
                out.insert(f.clone());
            }
        }
        Pattern::As { inner, name, .. } => {
            pattern_bound_names_into(inner, out);
            out.insert(name.clone());
        }
    }
}

/// `e` 中の自由変数 (裸の unqualified `Var`) を `out` に追加する。
/// `bound` は呼び出し側で初期化したスコープ。lambda / let / case でローカルに
/// 拡張・復元される。
pub fn free_var_names_in(e: &Expr, bound: &mut HashSet<String>, out: &mut HashSet<String>) {
    match e {
        Expr::Var { module, name, .. } => {
            let is_unqualified = match module {
                None => true,
                Some(m) => m.segments.is_empty(),
            };
            if is_unqualified && !bound.contains(name) {
                out.insert(name.clone());
            }
        }
        Expr::Ctor { .. } | Expr::Lit(_, _) | Expr::Error(_) => {}
        Expr::List(items, _) => {
            for it in items {
                free_var_names_in(it, bound, out);
            }
        }
        Expr::Record(fields, _) => {
            for f in fields {
                free_var_names_in(&f.value, bound, out);
            }
        }
        Expr::RecordUpdate { base, updates, .. } => {
            free_var_names_in(base, bound, out);
            for u in updates {
                free_var_names_in(&u.value, bound, out);
            }
        }
        Expr::Field { receiver, .. } => free_var_names_in(receiver, bound, out),
        Expr::App { func, arg, .. } => {
            free_var_names_in(func, bound, out);
            free_var_names_in(arg, bound, out);
        }
        Expr::Lambda { params, body, .. } => {
            let saved: HashSet<String> = bound.clone();
            for p in params {
                pattern_bound_names_into(p, bound);
            }
            free_var_names_in(body, bound, out);
            *bound = saved;
        }
        Expr::Let { bindings, body, .. } => {
            // 相互再帰として扱うので、全 binding 名を bound に入れてから辿る。
            let saved: HashSet<String> = bound.clone();
            for b in bindings {
                bound.insert(b.name.clone());
            }
            for b in bindings {
                let body_saved: HashSet<String> = bound.clone();
                for p in &b.params {
                    pattern_bound_names_into(p, bound);
                }
                free_var_names_in(&b.body, bound, out);
                *bound = body_saved;
            }
            free_var_names_in(body, bound, out);
            *bound = saved;
        }
        Expr::Sketch { bindings, body, .. } => {
            // 逐次スコープ (前方参照不可): binding 本体を見てから名前を bound に足す。
            let saved: HashSet<String> = bound.clone();
            for b in bindings {
                free_var_names_in(&b.body, bound, out);
                bound.insert(b.name.clone());
            }
            free_var_names_in(body, bound, out);
            *bound = saved;
        }
        Expr::If {
            cond,
            then_branch,
            else_branch,
            ..
        } => {
            free_var_names_in(cond, bound, out);
            free_var_names_in(then_branch, bound, out);
            free_var_names_in(else_branch, bound, out);
        }
        Expr::Case {
            scrutinee, arms, ..
        } => {
            free_var_names_in(scrutinee, bound, out);
            for arm in arms {
                let saved: HashSet<String> = bound.clone();
                pattern_bound_names_into(&arm.pattern, bound);
                if let Some(g) = &arm.guard {
                    free_var_names_in(g, bound, out);
                }
                free_var_names_in(&arm.body, bound, out);
                *bound = saved;
            }
        }
        Expr::BinOp { left, right, .. } => {
            free_var_names_in(left, bound, out);
            free_var_names_in(right, bound, out);
        }
        Expr::Negate(inner, _) => free_var_names_in(inner, bound, out),
        Expr::Range { lo, hi, .. } => {
            free_var_names_in(lo, bound, out);
            free_var_names_in(hi, bound, out);
        }
    }
}

/// `ValueDecl` の body が参照する自由変数を返す (params で束縛されるものは除外)。
pub fn value_decl_free_names(v: &ValueDecl) -> HashSet<String> {
    let mut bound: HashSet<String> = HashSet::new();
    for p in &v.params {
        pattern_bound_names_into(p, &mut bound);
    }
    let mut out: HashSet<String> = HashSet::new();
    free_var_names_in(&v.body, &mut bound, &mut out);
    out
}
