use std::cmp::Ordering;
use std::fmt;

use crate::constraint::{ArithEq, ArithExpr, solve_constraints};
use crate::manifold_bridge::{BuiltinFunctor, is_builtin_functor};
use crate::parse::{ArithOp, Bound, Clause, FixedPoint, Term, annotated_var, list, number, struc, var};
use strum::IntoEnumIterator;

/// Check if a term is a built-in primitive that should not be rewritten
fn is_builtin_primitive(term: &Term) -> bool {
    match term {
        Term::Struct { functor, .. } => is_builtin_functor(functor),
        // InfixExpr (+, -, *) with CAD primitives as operands is also builtin
        Term::InfixExpr { left, right, .. } => {
            is_builtin_primitive(left) && is_builtin_primitive(right)
        }
        _ => false,
    }
}

fn builtin_fact(functor: &str, arity: usize) -> Clause {
    let args = (0..arity)
        .map(|idx| var(format!("__builtin_arg_{}", idx)))
        .collect();
    Clause::Fact(struc(functor.to_string(), args))
}

fn builtin_functor_arities(functor: BuiltinFunctor) -> &'static [usize] {
    match functor {
        BuiltinFunctor::Cube => &[3],
        BuiltinFunctor::Sphere => &[1, 2],
        BuiltinFunctor::Cylinder => &[2, 3],
        BuiltinFunctor::Tetrahedron => &[0],
        BuiltinFunctor::Union => &[2],
        BuiltinFunctor::Difference => &[2],
        BuiltinFunctor::Intersection => &[2],
        BuiltinFunctor::Translate => &[4],
        BuiltinFunctor::Scale => &[4],
        BuiltinFunctor::Rotate => &[4],
        BuiltinFunctor::Polygon => &[1],
        BuiltinFunctor::Circle => &[1, 2],
        BuiltinFunctor::Extrude => &[2],
        BuiltinFunctor::Revolve => &[2, 3],
        BuiltinFunctor::Polyhedron => &[2],
    }
}

fn builtin_cad_facts() -> Vec<Clause> {
    BuiltinFunctor::iter()
        .flat_map(|functor| {
            let functor_name: &'static str = functor.into();
            builtin_functor_arities(functor)
                .iter()
                .copied()
                .map(move |arity| builtin_fact(functor_name, arity))
        })
        .collect()
}

/// 単一化エラー
#[derive(Debug, Clone)]
pub struct UnifyError {
    pub message: String,
    pub term1: Term,
    pub term2: Term,
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
    pub goal: Term,
}

impl fmt::Display for RewriteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {:?}", self.message, self.goal)
    }
}

impl std::error::Error for RewriteError {}

/// 単一変数の置換をインプレースで適用
fn substitute_in_place(term: &mut Term, var_name: &str, replacement: &Term) {
    match term {
        Term::Var { name } if name == var_name => {
            *term = replacement.clone();
        }
        Term::AnnotatedVar { name, .. } if name == var_name => {
            // AnnotatedVarを置換する場合、replacementがNumberならspanを保持したままdefault_valueだけ更新
            // replacementが他の構造なら通常通り置換
            match replacement {
                Term::Number { value: new_val } => {
                    if let Term::AnnotatedVar { default_value, .. } = term {
                        *default_value = Some(*new_val);
                    }
                }
                _ => {
                    *term = replacement.clone();
                }
            }
        }
        Term::Struct { args, .. } => {
            for arg in args.iter_mut() {
                substitute_in_place(arg, var_name, replacement);
            }
        }
        Term::List { items, tail } => {
            for item in items.iter_mut() {
                substitute_in_place(item, var_name, replacement);
            }
            if let Some(t) = tail {
                substitute_in_place(t.as_mut(), var_name, replacement);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            substitute_in_place(left.as_mut(), var_name, replacement);
            substitute_in_place(right.as_mut(), var_name, replacement);
        }
        Term::Constraint { left, right } => {
            substitute_in_place(left.as_mut(), var_name, replacement);
            substitute_in_place(right.as_mut(), var_name, replacement);
        }
        _ => {}
    }
}

/// unify 内のスタックに対して変数置換を適用
fn substitute_in_stack(stack: &mut Vec<(Term, Term)>, var_name: &str, replacement: &Term) {
    for (t1, t2) in stack.iter_mut() {
        substitute_in_place(t1, var_name, replacement);
        substitute_in_place(t2, var_name, replacement);
    }
}

/// goals に対して変数置換を適用
fn substitute_in_goals(goals: &mut Vec<Term>, var_name: &str, replacement: &Term) {
    for goal in goals.iter_mut() {
        substitute_in_place(goal, var_name, replacement);
    }
}

fn collect_default_var_bindings(term: &Term, bindings: &mut Vec<(String, FixedPoint)>) {
    match term {
        Term::AnnotatedVar {
            name,
            default_value: Some(value),
            ..
        } if name != "_" => {
            bindings.push((name.clone(), *value));
        }
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
        _ => {}
    }
}

fn apply_default_var_bindings(term: &mut Term, goals: &mut Vec<Term>) {
    let mut bindings = Vec::new();
    collect_default_var_bindings(term, &mut bindings);
    for (name, value) in bindings {
        let replacement = number(value);
        substitute_in_place(term, &name, &replacement);
        substitute_in_goals(goals, &name, &replacement);
    }

    eval_arith_in_place(term);
    for goal in goals.iter_mut() {
        eval_arith_in_place(goal);
    }
}

/// 算術式を評価する。評価できない場合（未束縛変数を含む場合）はNoneを返す
/// 注: AnnotatedVar は eval_arith では処理しない。unify の明示的な AnnotatedVar ハンドラが
/// 名前ベースの置換を伴って処理するため、ここで Number に変換すると置換が抜け落ちる。
fn eval_arith(term: &Term) -> Option<FixedPoint> {
    match term {
        Term::Number { value } => Some(*value),
        Term::InfixExpr { op, left, right } => {
            let l = eval_arith(left)?;
            let r = eval_arith(right)?;
            Some(match op {
                ArithOp::Add => l + r,
                ArithOp::Sub => l - r,
                ArithOp::Mul => l * r,
                ArithOp::Div => l / r,
            })
        }
        _ => None,
    }
}

/// 算術式をインプレースで評価し、可能なら数値に置き換える
fn eval_arith_in_place(term: &mut Term) {
    if let Some(val) = eval_arith(term) {
        *term = number(val);
    } else {
        // 再帰的に子要素も評価
        match term {
            Term::InfixExpr { left, right, .. } => {
                eval_arith_in_place(left.as_mut());
                eval_arith_in_place(right.as_mut());
            }
            Term::Struct { args, .. } => {
                for arg in args.iter_mut() {
                    eval_arith_in_place(arg);
                }
            }
            Term::List { items, tail } => {
                for item in items.iter_mut() {
                    eval_arith_in_place(item);
                }
                if let Some(t) = tail {
                    eval_arith_in_place(t.as_mut());
                }
            }
            Term::Constraint { left, right } => {
                eval_arith_in_place(left.as_mut());
                eval_arith_in_place(right.as_mut());
            }
            _ => {}
        }
    }
}

/// occurs check: 変数varが項term内に出現するか
fn occurs_check(var_name: &str, term: &Term) -> bool {
    match term {
        Term::Var { name } => name == var_name,
        Term::AnnotatedVar { name, .. } => name == var_name,
        Term::Struct { args, .. } => args.iter().any(|arg| occurs_check(var_name, arg)),
        Term::List { items, tail } => {
            items.iter().any(|item| occurs_check(var_name, item))
                || tail.as_ref().map_or(false, |t| occurs_check(var_name, t))
        }
        Term::InfixExpr { left, right, .. } => {
            occurs_check(var_name, left) || occurs_check(var_name, right)
        }
        Term::Constraint { left, right } => {
            occurs_check(var_name, left) || occurs_check(var_name, right)
        }
        Term::Number { .. } => false,
    }
}

/// 2つの下限境界から、より厳しい方を選択
fn intersect_min(a: Option<Bound>, b: Option<Bound>) -> Option<Bound> {
    match (a, b) {
        (None, None) => None,
        (Some(x), None) | (None, Some(x)) => Some(x),
        (Some(a), Some(b)) => {
            if a.value > b.value {
                Some(a)
            } else if b.value > a.value {
                Some(b)
            } else {
                // 同じ値の場合、exclusiveの方が厳しい
                Some(Bound {
                    value: a.value,
                    inclusive: a.inclusive && b.inclusive,
                })
            }
        }
    }
}

/// 2つの上限境界から、より厳しい方を選択
fn intersect_max(a: Option<Bound>, b: Option<Bound>) -> Option<Bound> {
    match (a, b) {
        (None, None) => None,
        (Some(x), None) | (None, Some(x)) => Some(x),
        (Some(a), Some(b)) => {
            if a.value < b.value {
                Some(a)
            } else if b.value < a.value {
                Some(b)
            } else {
                // 同じ値の場合、exclusiveの方が厳しい
                Some(Bound {
                    value: a.value,
                    inclusive: a.inclusive && b.inclusive,
                })
            }
        }
    }
}

/// 範囲が空でないかチェック（少なくとも1つの整数値が含まれるか）
fn range_is_valid(min: Option<Bound>, max: Option<Bound>) -> bool {
    match (min, max) {
        (None, _) | (_, None) => true,
        (Some(min_b), Some(max_b)) => {
            if min_b.value > max_b.value {
                false
            } else if min_b.value < max_b.value {
                true
            } else {
                // min_b.value == max_b.value: 両方がinclusiveでないと空
                min_b.inclusive && max_b.inclusive
            }
        }
    }
}

/// 値が範囲内にあるかチェック
fn value_in_range(value: FixedPoint, min: Option<Bound>, max: Option<Bound>) -> bool {
    let min_ok = match min {
        None => true,
        Some(b) if b.inclusive => value >= b.value,
        Some(b) => value > b.value,
    };
    let max_ok = match max {
        None => true,
        Some(b) if b.inclusive => value <= b.value,
        Some(b) => value < b.value,
    };
    min_ok && max_ok
}

/// deferred リスト内の全Termに対して変数を置換する
fn substitute_in_deferred(deferred: &mut Vec<(Term, Term)>, var_name: &str, replacement: &Term) {
    for (t1, t2) in deferred.iter_mut() {
        substitute_in_place(t1, var_name, replacement);
        substitute_in_place(t2, var_name, replacement);
    }
}

/// 項が算術式として評価可能な形か（変数と数値とInfixExprのみで構成されているか）
fn is_potentially_arithmetic(term: &Term) -> bool {
    match term {
        Term::Number { .. } => true,
        Term::Var { .. } | Term::AnnotatedVar { .. } => true,
        Term::InfixExpr { left, right, .. } => {
            is_potentially_arithmetic(left) && is_potentially_arithmetic(right)
        }
        _ => false,
    }
}

/// 2つの項を単一化し、成功すれば goals をインプレースで書き換える
pub fn unify(term1: Term, term2: Term, goals: &mut Vec<Term>) -> Result<(), UnifyError> {
    let mut stack = vec![(term1, term2)];
    // 未評価の算術式制約を保持（変数が束縛されたら再評価）
    let mut deferred: Vec<(Term, Term)> = Vec::new();

    while let Some((mut t1, mut t2)) = stack.pop() {
        // X@25 のような既定値付き変数は、単一化時に X=25 を伝播する。
        // AnnotatedVarでdefault_valueありの場合のみ（rangeのみの場合はRangeVar的に扱う）
        if let Term::AnnotatedVar {
            name,
            default_value: Some(value),
            ..
        } = &t1
        {
            let name = name.clone();
            let replacement = number(*value);
            if name != "_" {
                substitute_in_stack(&mut stack, &name, &replacement);
                substitute_in_goals(goals, &name, &replacement);
                substitute_in_deferred(&mut deferred, &name, &replacement);
            }
            t1 = replacement;
        }
        if let Term::AnnotatedVar {
            name,
            default_value: Some(value),
            ..
        } = &t2
        {
            let name = name.clone();
            let replacement = number(*value);
            if name != "_" {
                substitute_in_stack(&mut stack, &name, &replacement);
                substitute_in_goals(goals, &name, &replacement);
                substitute_in_deferred(&mut deferred, &name, &replacement);
            }
            t2 = replacement;
        }

        if let Some(val) = eval_arith(&t1) {
            t1 = number(val);
        }
        if let Some(val) = eval_arith(&t2) {
            t2 = number(val);
        }

        // 算術式がまだ評価できない場合は遅延
        let has_unbound_var = |term: &Term| -> bool { matches!(term, Term::InfixExpr { .. }) };

        if has_unbound_var(&t1) || has_unbound_var(&t2) {
            deferred.push((t1, t2));
            continue;
        }

        match (&t1, &t2) {
            // 同じ変数
            (Term::Var { name: n1 }, Term::Var { name: n2 }) if n1 == n2 => {}
            // AnnotatedVar(rangeのみ)同士: 範囲の交差を計算
            (
                Term::AnnotatedVar {
                    name: n1,
                    min: min1,
                    max: max1,
                    ..
                },
                Term::AnnotatedVar {
                    name: n2,
                    min: min2,
                    max: max2,
                    ..
                },
            ) => {
                let new_min = intersect_min(*min1, *min2);
                let new_max = intersect_max(*max1, *max2);

                if !range_is_valid(new_min, new_max) {
                    return Err(UnifyError {
                        message: format!("range intersection is empty"),
                        term1: t1,
                        term2: t2,
                    });
                }

                let intersected = annotated_var(n1.clone(), None, new_min, new_max, None);
                if n1 != "_" {
                    substitute_in_stack(&mut stack, n1, &intersected);
                    substitute_in_goals(goals, n1, &intersected);
                    substitute_in_deferred(&mut deferred, n1, &intersected);
                }
                if n2 != "_" && n2 != n1 {
                    substitute_in_stack(&mut stack, n2, &intersected);
                    substitute_in_goals(goals, n2, &intersected);
                    substitute_in_deferred(&mut deferred, n2, &intersected);
                }
            }
            // AnnotatedVar(rangeあり)とNumber: 範囲内かチェック
            (Term::AnnotatedVar { name, min, max, .. }, Term::Number { value }) => {
                if !value_in_range(*value, *min, *max) {
                    return Err(UnifyError {
                        message: format!("value {} is out of range {:?}", value, t1),
                        term1: t1,
                        term2: t2,
                    });
                }
                if name != "_" {
                    substitute_in_stack(&mut stack, name, &t2);
                    substitute_in_goals(goals, name, &t2);
                    substitute_in_deferred(&mut deferred, name, &t2);
                }
            }
            // swap して再処理
            (Term::Number { .. }, Term::AnnotatedVar { .. }) => {
                stack.push((t2, t1));
            }
            // AnnotatedVarと他 (Varと同様に扱う)
            (Term::AnnotatedVar { name, .. }, _) if name != "_" => {
                if occurs_check(name, &t2) {
                    return Err(UnifyError {
                        message: format!("occurs check failed: {} occurs in {:?}", name, t2),
                        term1: t1,
                        term2: t2,
                    });
                }
                substitute_in_stack(&mut stack, name, &t2);
                substitute_in_goals(goals, name, &t2);
                substitute_in_deferred(&mut deferred, name, &t2);
            }
            (_, Term::AnnotatedVar { name, .. }) if name != "_" => {
                stack.push((t2, t1));
            }
            // 変数と何か（anonymous変数 "_" は束縛しない）
            (Term::Var { name }, _) if name != "_" => {
                if occurs_check(name, &t2) {
                    return Err(UnifyError {
                        message: format!("occurs check failed: {} occurs in {:?}", name, t2),
                        term1: t1,
                        term2: t2,
                    });
                }
                substitute_in_stack(&mut stack, name, &t2);
                substitute_in_goals(goals, name, &t2);
                substitute_in_deferred(&mut deferred, name, &t2);
            }
            (_, Term::Var { name }) if name != "_" => {
                stack.push((t2, t1));
            }
            // anonymous変数はどんな項とも単一化成功（束縛なし）
            (Term::Var { name }, _) | (Term::AnnotatedVar { name, .. }, _) if name == "_" => {}
            (_, Term::Var { name }) | (_, Term::AnnotatedVar { name, .. }) if name == "_" => {}
            // 数値
            (Term::Number { value: v1 }, Term::Number { value: v2 }) => {
                if v1 != v2 {
                    return Err(UnifyError {
                        message: format!("number mismatch: {} != {}", v1, v2),
                        term1: t1,
                        term2: t2,
                    });
                }
            }
            // 構造体
            (
                Term::Struct {
                    functor: f1,
                    args: args1,
                },
                Term::Struct {
                    functor: f2,
                    args: args2,
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
            // リスト
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
                // 要素を逆順でpushしてLIFOでも左から右の順序で処理されるようにする
                for i in (0..min_len).rev() {
                    stack.push((items1[i].clone(), items2[i].clone()));
                }

                match (items1.len().cmp(&items2.len()), tail1, tail2) {
                    (Ordering::Equal, Some(t1), Some(t2)) => {
                        stack.push((t1.as_ref().clone(), t2.as_ref().clone()));
                    }
                    (Ordering::Equal, None, None) => {}
                    (Ordering::Equal, Some(t1), None) => {
                        stack.push((t1.as_ref().clone(), list(vec![], None)));
                    }
                    (Ordering::Equal, None, Some(t2)) => {
                        stack.push((list(vec![], None), t2.as_ref().clone()));
                    }
                    (Ordering::Greater, _, Some(t2_tail)) => {
                        let remaining: Vec<Term> = items1[min_len..].to_vec();
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
                        let remaining: Vec<Term> = items2[min_len..].to_vec();
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
    for (t1, t2) in deferred {
        let v1 = eval_arith(&t1);
        let v2 = eval_arith(&t2);
        match (v1, v2) {
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
                // 両辺が算術式として解決可能な形（変数+数値+演算子のみ）なら
                // Constraint として goals に押し出す（変数束縛後に再評価される）
                if is_potentially_arithmetic(&t1) && is_potentially_arithmetic(&t2) {
                    goals.push(Term::Constraint {
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

    // goals 内の算術式を評価
    for goal in goals.iter_mut() {
        eval_arith_in_place(goal);
    }

    Ok(())
}

/// goals 内の Constraint を評価し、解けたものは除去、解けないものは残す
/// 全 Constraint をまとめて SolverState に渡し、連立方程式として解く
fn try_resolve_constraints(goals: &mut Vec<Term>) -> Result<(), RewriteError> {
    // Constraint を ArithEq に変換して SolverState に渡す
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
        for &idx in constraint_indices.iter().rev() {
            goals.remove(idx);
        }
        for (var_name, value) in &result.bindings {
            let replacement = number(*value);
            substitute_in_goals(goals, var_name, &replacement);
        }
        for goal in goals.iter_mut() {
            eval_arith_in_place(goal);
        }
    }

    Ok(())
}

/// 節の変数をリネームして衝突を避ける
fn rename_clause_vars(clause: &mut Clause, suffix: &str) {
    match clause {
        Clause::Fact(term) => rename_term_vars(term, suffix),
        Clause::Rule { head, body } => {
            rename_term_vars(head, suffix);
            for t in body.iter_mut() {
                rename_term_vars(t, suffix);
            }
        }
    }
}

fn rename_term_vars(term: &mut Term, suffix: &str) {
    match term {
        Term::Var { name } => {
            if name != "_" {
                *name = format!("{}_{}", name, suffix);
            }
        }
        Term::AnnotatedVar { name, .. } => {
            if name != "_" {
                *name = format!("{}_{}", name, suffix);
            }
        }
        Term::Struct { args, .. } => {
            for arg in args.iter_mut() {
                rename_term_vars(arg, suffix);
            }
        }
        Term::List { items, tail } => {
            for item in items.iter_mut() {
                rename_term_vars(item, suffix);
            }
            if let Some(t) = tail {
                rename_term_vars(t.as_mut(), suffix);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            rename_term_vars(left.as_mut(), suffix);
            rename_term_vars(right.as_mut(), suffix);
        }
        Term::Constraint { left, right } => {
            rename_term_vars(left.as_mut(), suffix);
            rename_term_vars(right.as_mut(), suffix);
        }
        Term::Number { .. } => {}
    }
}

/// 単一の項をルールとマッチさせ、マッチすれば(書き換え後の項, 置換適用済みbody)を返す
/// マッチしなければNoneを返す
fn try_rewrite_single_with_result(
    db: &[Clause],
    clause_counter: &mut usize,
    term: &Term,
    other_goals: &mut Vec<Term>,
) -> Option<(Term, Vec<Term>)> {
    for clause in db.iter() {
        *clause_counter += 1;
        let mut renamed = clause.clone();
        rename_clause_vars(&mut renamed, &clause_counter.to_string());

        let (head, body) = match &renamed {
            Clause::Fact(t) => (t.clone(), vec![]),
            Clause::Rule { head, body } => (head.clone(), body.clone()),
        };

        // term, body, other_goals を全て trial_goals に入れてunifyし、置換を適用
        let mut trial_goals = vec![term.clone()];
        trial_goals.extend(body.clone());
        trial_goals.extend(other_goals.clone());

        if unify(term.clone(), head, &mut trial_goals).is_ok() {
            // trial_goals[0] が置換適用済みの term
            let resolved_term = trial_goals.remove(0);
            // 次の body.len() 個が置換適用済みの body
            let resolved_body: Vec<Term> = trial_goals.drain(0..body.len()).collect();
            // 残りが other_goals
            *other_goals = trial_goals;
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
    term: Term,
    other_goals: &mut Vec<Term>,
) -> Result<Vec<Term>, RewriteError> {
    let mut term = term;
    apply_default_var_bindings(&mut term, other_goals);

    // まず、この項自体がルールにマッチするか試す
    if let Some((resolved_term, body)) =
        try_rewrite_single_with_result(db, clause_counter, &term, other_goals)
    {
        if body.is_empty() {
            // CADビルトインfactにマッチしたときだけ、引数内の節参照を解決する
            let resolved_term =
                resolve_builtin_fact_args(db, clause_counter, resolved_term, other_goals)?;
            return Ok(vec![resolved_term]);
        } else {
            // Ruleにマッチ: bodyの各項を再帰的に解決
            let mut remaining_body: Vec<Term> = body;
            let mut all_resolved = Vec::new();

            // body 解決前に制約を解き、変数束縛を body と other_goals に伝播
            {
                let body_len = remaining_body.len();
                let mut combined = remaining_body;
                combined.extend(other_goals.drain(..));
                try_resolve_constraints(&mut combined)?;
                remaining_body = combined.drain(0..body_len).collect();
                *other_goals = combined;
            }

            while let Some(b) = remaining_body.first().cloned() {
                remaining_body.remove(0);

                // remaining_body を other_goals の先頭に追加
                let mut temp_other_goals = remaining_body.clone();
                temp_other_goals.extend(other_goals.clone());

                let resolved =
                    rewrite_term_recursive(db, clause_counter, b, &mut temp_other_goals)?;
                all_resolved.extend(resolved);

                // 置換が適用された remaining_body と other_goals を復元
                remaining_body = temp_other_goals.drain(0..remaining_body.len()).collect();
                *other_goals = temp_other_goals;
            }

            try_resolve_constraints(other_goals)?;

            return Ok(all_resolved);
        }
    }

    // ルールにマッチしない場合、サブタームを再帰的に書き換える
    match term {
        Term::InfixExpr { op, left, right } => {
            let new_left_terms = rewrite_term_recursive(db, clause_counter, *left, other_goals)?;
            let new_right_terms = rewrite_term_recursive(db, clause_counter, *right, other_goals)?;

            // InfixExpr の各オペランドは1つの項に解決されるべき
            if new_left_terms.len() != 1 || new_right_terms.len() != 1 {
                return Err(RewriteError {
                    message: "InfixExpr operand resolved to multiple terms".to_string(),
                    goal: Term::InfixExpr {
                        op,
                        left: Box::new(new_left_terms.into_iter().next().unwrap_or(Term::Number {
                            value: FixedPoint::from_int(0),
                        })),
                        right: Box::new(new_right_terms.into_iter().next().unwrap_or(
                            Term::Number {
                                value: FixedPoint::from_int(0),
                            },
                        )),
                    },
                });
            }

            let new_left = new_left_terms.into_iter().next().unwrap();
            let new_right = new_right_terms.into_iter().next().unwrap();

            let new_term = Term::InfixExpr {
                op,
                left: Box::new(new_left),
                right: Box::new(new_right),
            };
            // 書き換え後の項がビルトインプリミティブならOK
            if is_builtin_primitive(&new_term) {
                Ok(vec![new_term])
            } else {
                Err(RewriteError {
                    message: "InfixExpr contains non-builtin terms after rewriting".to_string(),
                    goal: new_term,
                })
            }
        }
        Term::Struct { functor, args } => {
            // ビルトインファンクターの場合はそのまま返す
            if is_builtin_functor(&functor) {
                return Ok(vec![Term::Struct { functor, args }]);
            }
            // 非ビルトインでルールにもマッチしない場合はエラー
            Err(RewriteError {
                message: "no clause matches goal".to_string(),
                goal: Term::Struct { functor, args },
            })
        }
        // その他の項（Number, Var, List など）はそのまま
        other => {
            if is_builtin_primitive(&other) {
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

fn resolve_builtin_fact_args(
    db: &[Clause],
    clause_counter: &mut usize,
    term: Term,
    other_goals: &mut Vec<Term>,
) -> Result<Term, RewriteError> {
    let (functor, args) = match term {
        Term::Struct { functor, args } if is_builtin_functor(&functor) => (functor, args),
        other => return Ok(other),
    };

    let mut resolved_args = Vec::with_capacity(args.len());
    for arg in args {
        match arg {
            // リテラル/未束縛変数はそのまま。節参照っぽい項のみ再解決する。
            Term::Number { .. }
            | Term::Var { .. }
            | Term::AnnotatedVar { .. } => {
                resolved_args.push(arg);
            }
            arg_term => {
                let mut resolved =
                    rewrite_term_recursive(db, clause_counter, arg_term, other_goals)?;
                if resolved.len() != 1 {
                    return Err(RewriteError {
                        message: "builtin argument resolved to multiple terms".to_string(),
                        goal: Term::Struct {
                            functor,
                            args: resolved_args,
                        },
                    });
                }
                resolved_args.push(resolved.remove(0));
            }
        }
    }

    Ok(Term::Struct {
        functor,
        args: resolved_args,
    })
}

pub fn execute(db: &mut [Clause], query: Vec<Term>) -> Result<Vec<Term>, RewriteError> {
    let mut clause_counter = 0;
    let mut results = Vec::new();
    let mut db_with_builtins = db.to_vec();
    db_with_builtins.extend(builtin_cad_facts());

    for term in query {
        let mut other_goals = Vec::new();
        let resolved = rewrite_term_recursive(
            &db_with_builtins,
            &mut clause_counter,
            term,
            &mut other_goals,
        )?;
        results.extend(resolved);
        results.extend(other_goals);
    }

    // 最終的に残った Constraint を検証
    try_resolve_constraints(&mut results)?;

    // 解決済み Constraint を結果から除去
    results.retain(|t| !matches!(t, Term::Constraint { .. }));

    Ok(results)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{FixedPoint, database, query, struc, var};

    fn run_success(db_src: &str, query_src: &str) -> Vec<String> {
        let mut db = database(db_src).expect("failed to parse db");
        let q = query(query_src).expect("failed to parse query").1;
        let resolved = execute(&mut db, q).expect("Expected success");
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
        let x = var("X".to_string());
        let a = struc("a".to_string(), vec![]);
        let mut goals = vec![var("X".to_string())];
        unify(x, a.clone(), &mut goals).unwrap();
        assert_eq!(goals[0], a);
    }

    #[test]
    fn test_unify_structs() {
        let t1 = struc("f".to_string(), vec![var("X".to_string())]);
        let t2 = struc("f".to_string(), vec![struc("a".to_string(), vec![])]);
        let mut goals = vec![var("X".to_string())];
        unify(t1, t2, &mut goals).unwrap();
        assert_eq!(goals[0], struc("a".to_string(), vec![]));
    }

    #[test]
    fn test_unify_fail() {
        let t1 = struc("f".to_string(), vec![]);
        let t2 = struc("g".to_string(), vec![]);
        assert!(unify(t1, t2, &mut vec![]).is_err());
    }

    // ===== RangeVar unify tests =====

    #[test]
    fn test_rangevar_number_in_range() {
        use crate::parse::{Bound, number_int, range_var};
        let rv = range_var(
            "X".to_string(),
            Some(Bound {
                value: FixedPoint::from_int(0),
                inclusive: false,
            }),
            Some(Bound {
                value: FixedPoint::from_int(10),
                inclusive: false,
            }),
        );
        let n = number_int(5);
        let mut goals = vec![var("X".to_string())];
        unify(rv, n.clone(), &mut goals).unwrap();
        assert_eq!(goals[0], n);
    }

    #[test]
    fn test_rangevar_number_out_of_range() {
        use crate::parse::{Bound, number_int, range_var};
        let rv = range_var(
            "X".to_string(),
            Some(Bound {
                value: FixedPoint::from_int(0),
                inclusive: false,
            }),
            Some(Bound {
                value: FixedPoint::from_int(10),
                inclusive: false,
            }),
        );
        let n = number_int(15);
        assert!(unify(rv, n, &mut vec![]).is_err());
    }

    #[test]
    fn test_rangevar_number_boundary_exclusive() {
        use crate::parse::{Bound, number_int, range_var};
        let make_rv = || {
            range_var(
                "X".to_string(),
                Some(Bound {
                    value: FixedPoint::from_int(0),
                    inclusive: false,
                }),
                Some(Bound {
                    value: FixedPoint::from_int(10),
                    inclusive: false,
                }),
            )
        };
        assert!(unify(make_rv(), number_int(0), &mut vec![]).is_err());
        assert!(unify(make_rv(), number_int(10), &mut vec![]).is_err());
        assert!(unify(make_rv(), number_int(1), &mut vec![]).is_ok());
        assert!(unify(make_rv(), number_int(9), &mut vec![]).is_ok());
    }

    #[test]
    fn test_rangevar_number_boundary_inclusive() {
        use crate::parse::{Bound, number_int, range_var};
        let make_rv = || {
            range_var(
                "X".to_string(),
                Some(Bound {
                    value: FixedPoint::from_int(0),
                    inclusive: true,
                }),
                Some(Bound {
                    value: FixedPoint::from_int(10),
                    inclusive: true,
                }),
            )
        };
        assert!(unify(make_rv(), number_int(0), &mut vec![]).is_ok());
        assert!(unify(make_rv(), number_int(10), &mut vec![]).is_ok());
    }

    #[test]
    fn test_rangevar_intersection() {
        use crate::parse::{Bound, range_var};
        let rv1 = range_var(
            "X".to_string(),
            Some(Bound {
                value: FixedPoint::from_int(0),
                inclusive: false,
            }),
            Some(Bound {
                value: FixedPoint::from_int(10),
                inclusive: false,
            }),
        );
        let rv2 = range_var(
            "Y".to_string(),
            Some(Bound {
                value: FixedPoint::from_int(5),
                inclusive: false,
            }),
            Some(Bound {
                value: FixedPoint::from_int(15),
                inclusive: false,
            }),
        );
        let mut goals = vec![var("X".to_string())];
        unify(rv1, rv2, &mut goals).unwrap();
        match &goals[0] {
            Term::AnnotatedVar { min, max, .. } => {
                assert_eq!(
                    *min,
                    Some(Bound {
                        value: FixedPoint::from_int(5),
                        inclusive: false
                    })
                );
                assert_eq!(
                    *max,
                    Some(Bound {
                        value: FixedPoint::from_int(10),
                        inclusive: false
                    })
                );
            }
            _ => panic!("Expected AnnotatedVar"),
        }
    }

    #[test]
    fn test_rangevar_intersection_empty() {
        use crate::parse::{Bound, range_var};
        let rv1 = range_var(
            "X".to_string(),
            Some(Bound {
                value: FixedPoint::from_int(0),
                inclusive: false,
            }),
            Some(Bound {
                value: FixedPoint::from_int(5),
                inclusive: false,
            }),
        );
        let rv2 = range_var(
            "Y".to_string(),
            Some(Bound {
                value: FixedPoint::from_int(10),
                inclusive: false,
            }),
            Some(Bound {
                value: FixedPoint::from_int(15),
                inclusive: false,
            }),
        );
        assert!(unify(rv1, rv2, &mut vec![]).is_err());
    }

    #[test]
    fn test_rangevar_intersection_inclusive_exclusive() {
        use crate::parse::{Bound, range_var};
        let rv1 = range_var(
            "X".to_string(),
            Some(Bound {
                value: FixedPoint::from_int(0),
                inclusive: true,
            }),
            Some(Bound {
                value: FixedPoint::from_int(5),
                inclusive: true,
            }),
        );
        let rv2 = range_var(
            "Y".to_string(),
            Some(Bound {
                value: FixedPoint::from_int(5),
                inclusive: false,
            }),
            Some(Bound {
                value: FixedPoint::from_int(10),
                inclusive: false,
            }),
        );
        assert!(unify(rv1, rv2, &mut vec![]).is_err());
    }

    // ===== arithmetic tests =====

    #[test]
    fn test_arith_simple_add() {
        use crate::parse::number_int;
        let expr =
            crate::parse::arith_expr(crate::parse::ArithOp::Add, number_int(3), number_int(5));
        let n = number_int(8);
        assert!(unify(expr, n, &mut vec![]).is_ok());
    }

    #[test]
    fn test_arith_simple_sub() {
        use crate::parse::number_int;
        let expr =
            crate::parse::arith_expr(crate::parse::ArithOp::Sub, number_int(10), number_int(3));
        let n = number_int(7);
        assert!(unify(expr, n, &mut vec![]).is_ok());
    }

    #[test]
    fn test_arith_simple_mul() {
        use crate::parse::number_int;
        let expr =
            crate::parse::arith_expr(crate::parse::ArithOp::Mul, number_int(4), number_int(5));
        let n = number_int(20);
        assert!(unify(expr, n, &mut vec![]).is_ok());
    }

    #[test]
    fn test_arith_simple_div() {
        use crate::parse::{number, number_int};
        let expr =
            crate::parse::arith_expr(crate::parse::ArithOp::Div, number_int(10), number_int(3));
        let n = number(FixedPoint::from_hundredths(333)); // 10/3 = 3.33 in fixed point
        assert!(unify(expr, n, &mut vec![]).is_ok());
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
            "cut(W) :- cube(W, 50, 260). main :- cube(X@25, 50, 300) - (cut(W@5) |> translate(X / 2 - W, 0, 0)).",
            "main.",
        );
        assert_eq!(
            resolved,
            vec!["(cube(X_2@25, 50, 300) - translate(cube(5, 50, 260), 7.5, 0, 0))"]
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
    fn sample_rule() {
        let resolved = run_success("p(X,Y) :- q(X, Z), r(Z, Y). q(a, b). r(b, c).", "p(A, B).");
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

    // ===== backtracking required (ignored for now) =====

    #[test]
    #[ignore]
    fn rule_multiple_goals() {
        let resolved = run_success(
            "grandparent(X, Z) :- parent(X, Y), parent(Y, Z). parent(a, b). parent(b, c).",
            "grandparent(a, c).",
        );
        assert_eq!(
            resolved,
            vec!["grandparent(a, c)", "parent(a, b)", "parent(b, c)"]
        );
    }

    #[test]
    #[ignore]
    fn rule_multiple_goals_with_var() {
        let resolved = run_success(
            "grandparent(X, Z) :- parent(X, Y), parent(Y, Z). parent(a, b). parent(b, c).",
            "grandparent(a, W).",
        );
        assert_eq!(
            resolved,
            vec!["grandparent(a, c)", "parent(a, b)", "parent(b, c)"]
        );
    }

    #[test]
    #[ignore]
    fn rule_shared_variable_propagation() {
        let resolved = run_success(
            "connect(X, Z) :- link(X, Y), link(Y, Z). link(a, b). link(b, c).",
            "connect(a, Z).",
        );
        assert_eq!(resolved, vec!["connect(a, c)", "link(a, b)", "link(b, c)"]);
    }

    #[test]
    #[ignore]
    fn rule_mixed_with_facts() {
        run_success(
            "animal(dog). animal(cat). is_pet(X) :- animal(X).",
            "is_pet(dog).",
        );
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
        // ob :- cube(1,1,1) |> translate(10,0,0). main :- ob + cube(2,2,2).
        let resolved = run_success(
            "ob :- cube(1,1,1) |> translate(10,0,0). main :- ob + cube(2,2,2).",
            "main.",
        );
        assert_eq!(
            resolved,
            vec!["(translate(cube(1, 1, 1), 10, 0, 0) + cube(2, 2, 2))"]
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
    fn constraint_propagation_across_body() {
        let resolved = run_success("f(X+Y, Y) :- h(X), g(Y). h(4). g(3).", "f(7, 3).");
        assert_eq!(resolved, vec!["h(4)", "g(3)"]);
    }
}
