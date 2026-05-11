use std::collections::HashMap;

use crate::parse::{ArithOp, Bound, FixedPoint, Term};

/// 制約ソルバーの結果
#[derive(Debug, Clone, PartialEq)]
pub struct SolveResult {
    /// 解けた変数束縛
    pub bindings: HashMap<String, FixedPoint>,
    /// 全制約が解消されたか（未解決の制約が残っていない）
    pub fully_resolved: bool,
}

/// 算術制約の連立方程式を解く
/// 成功時は解けた変数束縛を返し、矛盾があればエラーメッセージを返す
pub fn solve_constraints(eqs: Vec<ArithEq>) -> Result<SolveResult, String> {
    let mut bindings: HashMap<String, FixedPoint> = HashMap::new();
    let mut constraints: Vec<ArithEq> = Vec::new();

    for eq in eqs {
        process_eq(eq, &mut bindings, &mut constraints)?;
    }

    // 不動点ループ: 既知の値を代入して再試行
    loop {
        if constraints.is_empty() {
            break;
        }
        let old_count = bindings.len();
        let pending = std::mem::take(&mut constraints);
        for eq in pending {
            let left = substitute_in_expr(&eq.left, &bindings);
            let right = substitute_in_expr(&eq.right, &bindings);
            process_eq(ArithEq::new(left, right), &mut bindings, &mut constraints)?;
        }
        if bindings.len() == old_count {
            break;
        }
    }

    Ok(SolveResult {
        bindings,
        fully_resolved: constraints.is_empty(),
    })
}

fn process_eq(
    eq: ArithEq,
    bindings: &mut HashMap<String, FixedPoint>,
    constraints: &mut Vec<ArithEq>,
) -> Result<(), String> {
    let left_val = try_eval(&eq.left);
    let right_val = try_eval(&eq.right);
    match (left_val, right_val) {
        (Some(l), Some(r)) => {
            if l != r {
                return Err(format!("{} != {}", l, r));
            }
        }
        (Some(target), None) => {
            if let Some((var, val)) = try_solve_for_var(&eq.right, target) {
                put_binding(bindings, var, val)?;
            } else {
                constraints.push(eq);
            }
        }
        (None, Some(target)) => {
            if let Some((var, val)) = try_solve_for_var(&eq.left, target) {
                put_binding(bindings, var, val)?;
            } else {
                constraints.push(eq);
            }
        }
        (None, None) => {
            constraints.push(eq);
        }
    }
    Ok(())
}

fn put_binding(
    bindings: &mut HashMap<String, FixedPoint>,
    var: String,
    value: FixedPoint,
) -> Result<(), String> {
    if let Some(&existing) = bindings.get(&var) {
        if existing != value {
            return Err(format!(
                "contradiction: {} already has value {}, cannot assign {}",
                var, existing, value
            ));
        }
    } else {
        bindings.insert(var, value);
    }
    Ok(())
}

fn try_eval(expr: &ArithExpr) -> Option<FixedPoint> {
    match expr {
        ArithExpr::Num(v) => Some(*v),
        ArithExpr::BinOp { op, left, right } => {
            let l = try_eval(left)?;
            let r = try_eval(right)?;
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

/// expr = target の形の方程式を解き、変数が1つなら (変数名, 値) を返す。
///
/// 除算で完全整除にならないケースでも FixedPoint の丸めを許容する。
/// 例えば `30 * T2 = 400` で T2 = 13.33 (実際の値 13.333... を 0.01 単位に丸めた値)。
/// 厳密整除を要求するとこの種の連立方程式が解けなくなり、
/// 歯車比のような実用的なケースで未束縛変数が下流に流れてしまう。
///
/// 矛盾検出は逆算で `candidate * l_val` と `target` の差が `DIVISION_TOLERANCE`
/// (= 1 単位の小数, つまり 0.01 から l_val を掛けたもの) を超える場合のみ失敗を返す。
/// これにより本物の矛盾 (例えば `2*X = 7, 3*X = 11`) は依然として捕捉される。
fn try_solve_for_var(expr: &ArithExpr, target: FixedPoint) -> Option<(String, FixedPoint)> {
    match expr {
        ArithExpr::Var(name) => Some((name.clone(), target)),
        ArithExpr::BinOp { op, left, right } => {
            let zero = FixedPoint::from_int(0);
            match (try_eval(left), try_eval(right)) {
                // c OP right = target → right を解く
                (Some(l_val), None) => {
                    let new_target = match op {
                        ArithOp::Add => target - l_val,
                        ArithOp::Sub => l_val - target,
                        ArithOp::Mul => {
                            if l_val == zero {
                                return None;
                            }
                            let candidate = target / l_val;
                            if !within_division_tolerance(candidate * l_val, target, l_val) {
                                return None;
                            }
                            candidate
                        }
                        ArithOp::Div => {
                            if target == zero {
                                return None;
                            }
                            let candidate = l_val / target;
                            if candidate == zero {
                                return None;
                            }
                            if !within_division_tolerance(l_val / candidate, target, target) {
                                return None;
                            }
                            candidate
                        }
                    };
                    try_solve_for_var(right, new_target)
                }
                // left OP c = target → left を解く
                (None, Some(r_val)) => {
                    let new_target = match op {
                        ArithOp::Add => target - r_val,
                        ArithOp::Sub => target + r_val,
                        ArithOp::Mul => {
                            if r_val == zero {
                                return None;
                            }
                            let candidate = target / r_val;
                            if !within_division_tolerance(candidate * r_val, target, r_val) {
                                return None;
                            }
                            candidate
                        }
                        ArithOp::Div => target * r_val,
                    };
                    try_solve_for_var(left, new_target)
                }
                _ => None,
            }
        }
        _ => None,
    }
}

/// 除算の丸めによる誤差を許容するかどうかを判定する。
///
/// `back` は逆算で得た値、`target` は本来の目標値、`scale` は係数。
/// FixedPoint::div は `(self.0 * 100) / rhs.0` の整数除算で 1 単位以内に切り捨てるため、
/// candidate * scale の逆算では最大 `|scale| / 100` raw 単位 + 切り捨て分 1 の誤差が発生する。
/// この上限以内の差は丸め誤差として許容し、それを超える差は本物の矛盾とみなす。
fn within_division_tolerance(back: FixedPoint, target: FixedPoint, scale: FixedPoint) -> bool {
    let diff_raw = (back.raw() - target.raw()).abs();
    let scale_raw = scale.raw().abs();
    let tol_raw = scale_raw / 100 + 1;
    diff_raw <= tol_raw
}

fn substitute_in_expr(expr: &ArithExpr, bindings: &HashMap<String, FixedPoint>) -> ArithExpr {
    match expr {
        ArithExpr::Var(name) => {
            if let Some(&value) = bindings.get(name) {
                ArithExpr::Num(value)
            } else {
                expr.clone()
            }
        }
        ArithExpr::BinOp { op, left, right } => ArithExpr::BinOp {
            op: *op,
            left: Box::new(substitute_in_expr(left, bindings)),
            right: Box::new(substitute_in_expr(right, bindings)),
        },
        other => other.clone(),
    }
}

/// Termの算術式サブセット
#[derive(Debug, Clone, PartialEq)]
pub enum ArithExpr {
    /// 変数
    Var(String),
    /// 範囲制約付き変数
    RangeVar {
        name: String,
        min: Option<Bound>,
        max: Option<Bound>,
    },
    /// 数値リテラル
    Num(FixedPoint),
    /// 二項演算
    BinOp {
        op: ArithOp,
        left: Box<ArithExpr>,
        right: Box<ArithExpr>,
    },
}

impl ArithExpr {
    pub fn var(name: impl Into<String>) -> Self {
        ArithExpr::Var(name.into())
    }

    pub fn num(value: FixedPoint) -> Self {
        ArithExpr::Num(value)
    }

    pub fn num_int(value: i64) -> Self {
        ArithExpr::Num(FixedPoint::from_int(value))
    }
}

impl std::ops::Add for ArithExpr {
    type Output = ArithExpr;
    fn add(self, rhs: Self) -> Self::Output {
        ArithExpr::BinOp {
            op: ArithOp::Add,
            left: Box::new(self),
            right: Box::new(rhs),
        }
    }
}

impl std::ops::Add<i64> for ArithExpr {
    type Output = ArithExpr;
    fn add(self, rhs: i64) -> Self::Output {
        self + ArithExpr::Num(FixedPoint::from_int(rhs))
    }
}

impl std::ops::Sub for ArithExpr {
    type Output = ArithExpr;
    fn sub(self, rhs: Self) -> Self::Output {
        ArithExpr::BinOp {
            op: ArithOp::Sub,
            left: Box::new(self),
            right: Box::new(rhs),
        }
    }
}

impl std::ops::Sub<i64> for ArithExpr {
    type Output = ArithExpr;
    fn sub(self, rhs: i64) -> Self::Output {
        self - ArithExpr::Num(FixedPoint::from_int(rhs))
    }
}

impl std::ops::Mul for ArithExpr {
    type Output = ArithExpr;
    fn mul(self, rhs: Self) -> Self::Output {
        ArithExpr::BinOp {
            op: ArithOp::Mul,
            left: Box::new(self),
            right: Box::new(rhs),
        }
    }
}

impl std::ops::Mul<i64> for ArithExpr {
    type Output = ArithExpr;
    fn mul(self, rhs: i64) -> Self::Output {
        self * ArithExpr::Num(FixedPoint::from_int(rhs))
    }
}

impl std::ops::Div for ArithExpr {
    type Output = ArithExpr;
    fn div(self, rhs: Self) -> Self::Output {
        ArithExpr::BinOp {
            op: ArithOp::Div,
            left: Box::new(self),
            right: Box::new(rhs),
        }
    }
}

impl std::ops::Div<i64> for ArithExpr {
    type Output = ArithExpr;
    fn div(self, rhs: i64) -> Self::Output {
        self / ArithExpr::Num(FixedPoint::from_int(rhs))
    }
}

impl From<i64> for ArithExpr {
    fn from(value: i64) -> Self {
        ArithExpr::Num(FixedPoint::from_int(value))
    }
}

/// 算術制約: left = right
#[derive(Debug, Clone, PartialEq)]
pub struct ArithEq {
    pub left: ArithExpr,
    pub right: ArithExpr,
}

impl ArithEq {
    pub fn new(left: ArithExpr, right: ArithExpr) -> Self {
        Self { left, right }
    }

    pub fn eq(left: impl Into<ArithExpr>, right: impl Into<ArithExpr>) -> Self {
        Self {
            left: left.into(),
            right: right.into(),
        }
    }
}

/// Term から ArithExpr への変換エラー
#[derive(Debug, Clone, PartialEq)]
pub struct ConversionError {
    pub message: String,
}

impl ArithExpr {
    /// Term から ArithExpr への変換を試みる
    /// Struct や List など算術式でないものは Err を返す
    pub fn try_from_term<S>(term: &Term<S>) -> Result<Self, ConversionError> {
        match term {
            Term::Var { default_value: Some(value), .. } => Ok(ArithExpr::Num(*value)),
            Term::Var { name, min, max, .. } if min.is_some() || max.is_some() => {
                Ok(ArithExpr::RangeVar {
                    name: name.clone(),
                    min: *min,
                    max: *max,
                })
            }
            Term::Var { name, .. } => Ok(ArithExpr::Var(name.clone())),
            Term::Number { value } => Ok(ArithExpr::Num(*value)),
            Term::InfixExpr { op, left, right } => {
                let left = ArithExpr::try_from_term(left)?;
                let right = ArithExpr::try_from_term(right)?;
                Ok(ArithExpr::BinOp {
                    op: *op,
                    left: Box::new(left),
                    right: Box::new(right),
                })
            }
            Term::Struct { functor, .. } => Err(ConversionError {
                message: format!(
                    "cannot convert struct '{}' to arithmetic expression",
                    functor
                ),
            }),
            Term::List { .. } => Err(ConversionError {
                message: "cannot convert list to arithmetic expression".to_string(),
            }),
            Term::Constraint { .. } => Err(ConversionError {
                message: "cannot convert constraint to arithmetic expression".to_string(),
            }),
            Term::StringLit { .. } => Err(ConversionError {
                message: "cannot convert string literal to arithmetic expression".to_string(),
            }),
        }
    }

    /// ArithExpr を Term に変換
    pub fn to_term(&self) -> Term {
        use crate::parse::{arith_expr, number, range_var, var};
        match self {
            ArithExpr::Var(name) => var(name.clone()),
            ArithExpr::RangeVar { name, min, max } => range_var(name.clone(), *min, *max),
            ArithExpr::Num(value) => number(*value),
            ArithExpr::BinOp { op, left, right } => {
                arith_expr(*op, left.to_term(), right.to_term())
            }
        }
    }

    /// 式中の未束縛変数を収集
    pub fn collect_vars(&self) -> Vec<String> {
        let mut vars = Vec::new();
        self.collect_vars_rec(&mut vars);
        vars
    }

    fn collect_vars_rec(&self, vars: &mut Vec<String>) {
        match self {
            ArithExpr::Var(name) if name != "_" => {
                if !vars.contains(name) {
                    vars.push(name.clone());
                }
            }
            ArithExpr::RangeVar { name, .. } => {
                if !vars.contains(name) {
                    vars.push(name.clone());
                }
            }
            ArithExpr::BinOp { left, right, .. } => {
                left.collect_vars_rec(vars);
                right.collect_vars_rec(vars);
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{ArithOp, FixedPoint, arith_expr, number_int, var};

    fn x() -> ArithExpr {
        ArithExpr::var("X")
    }
    fn y() -> ArithExpr {
        ArithExpr::var("Y")
    }

    fn v(name: &str) -> Term {
        var(name.to_string())
    }

    fn solve(constraint: ArithEq) -> Result<SolveResult, String> {
        solve_constraints(vec![constraint])
    }

    // ===== ArithExpr operator tests =====

    #[test]
    fn test_arith_expr_operators() {
        assert_eq!(
            x() + 1,
            ArithExpr::BinOp {
                op: ArithOp::Add,
                left: Box::new(ArithExpr::Var("X".to_string())),
                right: Box::new(ArithExpr::Num(FixedPoint::from_int(1))),
            }
        );

        let vars = (x() + y()).collect_vars();
        assert_eq!(vars, vec!["X".to_string(), "Y".to_string()]);
    }

    #[test]
    fn test_arith_expr_from_term() {
        let term = arith_expr(ArithOp::Add, v("X"), number_int(1));
        let expr = ArithExpr::try_from_term(&term).unwrap();
        assert_eq!(expr, x() + 1);
    }

    // ===== solver tests =====

    #[test]
    fn test_linear_simple_addition() {
        // X + 1 = 6 -> X = 5
        let r = solve(ArithEq::eq(x() + 1, 6)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(5)));
    }

    #[test]
    fn test_linear_simple_subtraction() {
        // X - 3 = 7 -> X = 10
        let r = solve(ArithEq::eq(x() - 3, 7)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(10)));
    }

    #[test]
    fn test_linear_variable_on_right() {
        // 6 = X + 1 -> X = 5
        let r = solve(ArithEq::eq(6, x() + 1)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(5)));
    }

    #[test]
    fn test_linear_multiplication() {
        // X * 2 = 10 -> X = 5
        let r = solve(ArithEq::eq(x() * 2, 10)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(5)));
    }

    #[test]
    fn test_linear_complex_expression() {
        // 2 * X + 3 = 11 -> X = 4
        let r = solve(ArithEq::eq(ArithExpr::num_int(2) * x() + 3, 11)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(4)));
    }

    #[test]
    fn test_linear_nested_expression() {
        // (X + 1) * 3 = 12 -> X = 3
        let r = solve(ArithEq::eq((x() + 1) * 3, 12)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(3)));
    }

    #[test]
    fn test_linear_negative_result() {
        // X + 10 = 3 -> X = -7
        let r = solve(ArithEq::eq(x() + 10, 3)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(-7)));
    }

    // ===== division tests =====

    #[test]
    fn test_division_simple() {
        // X / 2 = 5 -> X = 10
        let r = solve(ArithEq::eq(x() / 2, 5)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(10)));
    }

    #[test]
    fn test_division_with_offset() {
        // (X + 1) / 3 = 4 -> X = 11
        let r = solve(ArithEq::eq((x() + 1) / 3, 4)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(11)));
    }

    #[test]
    fn test_division_negative_divisor() {
        // X / -2 = 5 -> X = -10
        let r = solve(ArithEq::eq(x() / ArithExpr::num_int(-2), 5)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(-10)));
    }

    // ===== general tests =====

    #[test]
    fn test_contradiction() {
        // 5 = 6 -> error
        assert!(solve(ArithEq::eq(5, 6)).is_err());
    }

    #[test]
    fn test_two_variables_unsolvable() {
        // X + Y = 10 -> unsolvable (残る)
        let r = solve(ArithEq::eq(x() + y(), 10)).unwrap();
        assert!(!r.fully_resolved);
    }

    #[test]
    fn test_fractional_solution() {
        // X * 2 = 5 -> X = 2.50
        let r = solve(ArithEq::eq(x() * 2, 5)).unwrap();
        assert_eq!(
            r.bindings.get("X"),
            Some(&FixedPoint::from_hundredths(250))
        );
    }

    #[test]
    fn test_both_sides_constant_equal() {
        // 5 = 5 -> ok
        let r = solve(ArithEq::eq(5, 5)).unwrap();
        assert!(r.fully_resolved);
    }

    #[test]
    fn test_division_by_variable() {
        // 6 / X = 2 -> X = 3
        let r = solve(ArithEq::eq(ArithExpr::num_int(6) / x(), 2)).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(3)));
    }

    #[test]
    fn test_from_multiple_constraints() {
        // X + 1 = 6
        let r = solve_constraints(vec![ArithEq::eq(x() + 1, 6)]).unwrap();
        assert_eq!(r.bindings.get("X"), Some(&FixedPoint::from_int(5)));
    }
}
