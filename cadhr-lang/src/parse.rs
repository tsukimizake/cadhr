use nom::{
    IResult, Parser,
    branch::alt,
    bytes::complete::{is_not, tag, take_until, take_while, take_while1},
    character::complete::{char, digit1, multispace1},
    combinator::{cut, map, map_res, opt, recognize, value},
    multi::{many0, separated_list0, separated_list1},
    sequence::{delimited, pair, preceded, separated_pair, terminated},
};
use std::fmt;

// ============================================================
// SrcSpan: ソースコード上のバイトオフセット範囲
// ============================================================

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SrcSpan {
    pub start: usize,
    pub end: usize,
}

// ============================================================
// FixedPoint: 2桁固定小数点数 (hundredths)
// ============================================================

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct FixedPoint(i64);

impl FixedPoint {
    pub fn from_hundredths(h: i64) -> Self {
        Self(h)
    }
    pub fn from_int(v: i64) -> Self {
        Self(v * 100)
    }
    pub fn to_f64(self) -> f64 {
        self.0 as f64 / 100.0
    }
    pub fn to_i64_checked(self) -> Option<i64> {
        (self.0 % 100 == 0).then(|| self.0 / 100)
    }
    pub fn raw(self) -> i64 {
        self.0
    }
}

impl fmt::Debug for FixedPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self)
    }
}

impl fmt::Display for FixedPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0 % 100 == 0 {
            write!(f, "{}", self.0 / 100)
        } else {
            let abs = self.0.unsigned_abs();
            let sign = if self.0 < 0 { "-" } else { "" };
            let whole = abs / 100;
            let frac = abs % 100;
            if frac % 10 == 0 {
                write!(f, "{}{}.{}", sign, whole, frac / 10)
            } else {
                write!(f, "{}{}.{:02}", sign, whole, frac)
            }
        }
    }
}

impl std::ops::Add for FixedPoint {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl std::ops::Sub for FixedPoint {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl std::ops::Mul for FixedPoint {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self(self.0 * rhs.0 / 100)
    }
}

impl std::ops::Div for FixedPoint {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        Self(self.0 * 100 / rhs.0)
    }
}

impl std::ops::Neg for FixedPoint {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

impl PartialOrd for FixedPoint {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for FixedPoint {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.cmp(&other.0)
    }
}

impl From<i64> for FixedPoint {
    fn from(v: i64) -> Self {
        Self::from_int(v)
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Bound {
    pub value: FixedPoint,
    pub inclusive: bool,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ArithOp {
    Add,
    Sub,
    Mul,
    Div,
}

#[derive(Clone)]
pub enum Term {
    Var {
        name: String,
    },
    /// 既定値付き変数: X@25
    DefaultVar {
        name: String,
        value: FixedPoint,
        span: Option<SrcSpan>,
    },
    /// 範囲制約付き変数: min < X < max など
    RangeVar {
        name: String,
        min: Option<Bound>,
        max: Option<Bound>,
    },
    Number {
        value: FixedPoint,
    },
    /// 中置演算子式: left op right (算術演算 or CSG演算)
    InfixExpr {
        op: ArithOp,
        left: Box<Term>,
        right: Box<Term>,
    },
    Struct {
        functor: String,
        args: Vec<Term>,
    },
    List {
        items: Vec<Term>,
        tail: Option<Box<Term>>,
    },
    /// 遅延された算術制約: left = right を後で検証
    Constraint {
        left: Box<Term>,
        right: Box<Term>,
    },
}

impl PartialEq for Term {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Term::Var { name: n1 }, Term::Var { name: n2 }) => n1 == n2,
            (
                Term::DefaultVar {
                    name: n1,
                    value: v1,
                    ..
                },
                Term::DefaultVar {
                    name: n2,
                    value: v2,
                    ..
                },
            ) => n1 == n2 && v1 == v2,
            (
                Term::RangeVar {
                    name: n1,
                    min: min1,
                    max: max1,
                },
                Term::RangeVar {
                    name: n2,
                    min: min2,
                    max: max2,
                },
            ) => n1 == n2 && min1 == min2 && max1 == max2,
            (Term::Number { value: v1 }, Term::Number { value: v2 }) => v1 == v2,
            (
                Term::InfixExpr {
                    op: o1,
                    left: l1,
                    right: r1,
                },
                Term::InfixExpr {
                    op: o2,
                    left: l2,
                    right: r2,
                },
            ) => o1 == o2 && l1 == l2 && r1 == r2,
            (
                Term::Struct {
                    functor: f1,
                    args: a1,
                },
                Term::Struct {
                    functor: f2,
                    args: a2,
                },
            ) => f1 == f2 && a1 == a2,
            (
                Term::List {
                    items: i1,
                    tail: t1,
                },
                Term::List {
                    items: i2,
                    tail: t2,
                },
            ) => i1 == i2 && t1 == t2,
            (
                Term::Constraint {
                    left: l1,
                    right: r1,
                },
                Term::Constraint {
                    left: l2,
                    right: r2,
                },
            ) => l1 == l2 && r1 == r2,
            _ => false,
        }
    }
}

#[derive(Clone, PartialEq)]
pub enum Clause {
    Fact(Term),
    Rule { head: Term, body: Vec<Term> },
}

impl fmt::Debug for Term {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Term::Var { name } => write!(f, "{}", name),
            Term::DefaultVar { name, value, .. } => write!(f, "{}@{}", name, value),
            Term::RangeVar { name, min, max } => {
                if let Some(b) = min {
                    write!(f, "{} {} ", b.value, if b.inclusive { "<=" } else { "<" })?;
                }
                write!(f, "{}", name)?;
                if let Some(b) = max {
                    write!(f, " {} {}", if b.inclusive { "<=" } else { "<" }, b.value)?;
                }
                Ok(())
            }
            Term::Number { value } => write!(f, "{}", value),
            Term::InfixExpr { op, left, right } => {
                let op_str = match op {
                    ArithOp::Add => "+",
                    ArithOp::Sub => "-",
                    ArithOp::Mul => "*",
                    ArithOp::Div => "/",
                };
                write!(f, "({:?} {} {:?})", left, op_str, right)
            }
            Term::Struct { functor, args } => {
                write!(f, "{}", functor)?;
                if !args.is_empty() {
                    write!(f, "(")?;
                    for (idx, arg) in args.iter().enumerate() {
                        if idx > 0 {
                            write!(f, ", ")?;
                        }
                        write!(f, "{:?}", arg)?;
                    }
                    write!(f, ")")?;
                }
                Ok(())
            }
            Term::List { items, tail } => {
                write!(f, "[")?;
                for (idx, item) in items.iter().enumerate() {
                    if idx > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{:?}", item)?;
                }
                if let Some(tail) = tail {
                    if !items.is_empty() {
                        write!(f, " | ")?;
                    }
                    write!(f, "{:?}", tail)?;
                }
                write!(f, "]")
            }
            Term::Constraint { left, right } => {
                write!(f, "constraint({:?} = {:?})", left, right)
            }
        }
    }
}

impl fmt::Debug for Clause {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Clause::Fact(term) => write!(f, "{:?}.", term),
            Clause::Rule { head, body } => {
                write!(f, "{:?} :- ", head)?;
                for (idx, term) in body.iter().enumerate() {
                    if idx > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{:?}", term)?;
                }
                write!(f, ".")
            }
        }
    }
}

/// Termコンストラクタ
pub fn var(name: String) -> Term {
    Term::Var { name }
}

pub fn default_var(name: String, value: FixedPoint) -> Term {
    Term::DefaultVar {
        name,
        value,
        span: None,
    }
}

pub fn default_var_with_span(name: String, value: FixedPoint, span: SrcSpan) -> Term {
    Term::DefaultVar {
        name,
        value,
        span: Some(span),
    }
}

pub fn number(value: FixedPoint) -> Term {
    Term::Number { value }
}

pub fn number_int(value: i64) -> Term {
    Term::Number {
        value: FixedPoint::from_int(value),
    }
}

pub fn struc(functor: String, args: Vec<Term>) -> Term {
    Term::Struct { functor, args }
}

pub fn list(items: Vec<Term>, tail: Option<Term>) -> Term {
    Term::List {
        items,
        tail: tail.map(Box::new),
    }
}

pub fn range_var(name: String, min: Option<Bound>, max: Option<Bound>) -> Term {
    Term::RangeVar { name, min, max }
}

pub fn arith_expr(op: ArithOp, left: Term, right: Term) -> Term {
    Term::InfixExpr {
        op,
        left: Box::new(left),
        right: Box::new(right),
    }
}

#[allow(unused)]
pub(super) fn v(name: impl Into<String>) -> Term {
    var(name.into())
}

#[allow(unused)]
pub(super) fn a(name: impl Into<String>) -> Term {
    struc(name.into(), vec![])
}

pub(super) type PResult<'a, T> = IResult<&'a str, T>;

// Whitespace and comments
fn line_comment(input: &str) -> PResult<'_, ()> {
    map(pair(tag("%"), opt(is_not("\n"))), |_| ()).parse(input)
}

fn block_comment(input: &str) -> PResult<'_, ()> {
    map((tag("/*"), cut(take_until("*/")), tag("*/")), |_| ()).parse(input)
}

fn space_or_comment1(input: &str) -> PResult<'_, ()> {
    value(
        (),
        many0(alt((value((), multispace1), line_comment, block_comment))),
    )
    .parse(input)
}

fn space_or_comment0(input: &str) -> PResult<'_, ()> {
    value(
        (),
        many0(alt((value((), multispace1), line_comment, block_comment))),
    )
    .parse(input)
}

fn ws<'a, F, O>(inner: F) -> impl Parser<&'a str, Output = O, Error = nom::error::Error<&'a str>>
where
    F: Parser<&'a str, Output = O, Error = nom::error::Error<&'a str>>,
{
    delimited(space_or_comment0, inner, space_or_comment0)
}

// Identifiers and atoms
fn is_atom_start(c: char) -> bool {
    c.is_ascii_lowercase()
}

fn is_id_continue(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

fn is_var_start(c: char) -> bool {
    c.is_ascii_uppercase() || c == '_'
}

fn unquoted_atom(input: &str) -> PResult<'_, String> {
    map(
        recognize(pair(take_while1(is_atom_start), take_while(is_id_continue))),
        |s: &str| s.to_string(),
    )
    .parse(input)
}

fn quoted_atom(input: &str) -> PResult<'_, String> {
    let escaped_char = preceded(
        char('\\'),
        alt((
            value('\'', char('\'')),
            value('\\', char('\\')),
            value('\n', char('n')),
            value('\t', char('t')),
        )),
    );
    let normal_char = map(is_not("'\\"), |s: &str| s.chars().collect::<Vec<_>>());

    map(
        delimited(
            char('\''),
            map(
                many0(alt((map(escaped_char, |c| vec![c]), normal_char))),
                |v: Vec<Vec<char>>| v.into_iter().flatten().collect::<String>(),
            ),
            cut(char('\'')),
        ),
        |s| s,
    )
    .parse(input)
}

fn atom(input: &str) -> PResult<'_, String> {
    alt((quoted_atom, unquoted_atom)).parse(input)
}

fn variable(input: &str) -> PResult<'_, String> {
    map(
        recognize(pair(take_while1(is_var_start), take_while(is_id_continue))),
        |s: &str| s.to_string(),
    )
    .parse(input)
}

fn fixed_number(input: &str) -> PResult<'_, FixedPoint> {
    map_res(
        recognize((opt(char('-')), digit1, opt(pair(char('.'), digit1)))),
        |s: &str| -> Result<FixedPoint, String> {
            if let Some(dot_pos) = s.find('.') {
                let int_part: i64 = s[..dot_pos].parse().map_err(|e: std::num::ParseIntError| e.to_string())?;
                let frac_str = &s[dot_pos + 1..];
                let frac_val: i64 = frac_str.parse().map_err(|e: std::num::ParseIntError| e.to_string())?;
                let frac = match frac_str.len() {
                    1 => frac_val * 10,
                    2 => frac_val,
                    _ => return Err("fractional part must be 1-2 digits".to_string()),
                };
                let sign = if s.starts_with('-') { -1 } else { 1 };
                Ok(FixedPoint::from_hundredths(sign * (int_part.abs() * 100 + frac)))
            } else {
                let v: i64 = s.parse().map_err(|e: std::num::ParseIntError| e.to_string())?;
                Ok(FixedPoint::from_int(v))
            }
        },
    )
    .parse(input)
}

// Terms
fn list_term(input: &str) -> PResult<'_, Term> {
    ws(delimited(
        char('['),
        map(
            (
                separated_list0(ws(char(',')), term),
                opt(preceded(ws(char('|')), term)),
            ),
            |(items, tail)| list(items, tail),
        ),
        cut(ws(char(']'))),
    ))
    .parse(input)
}

fn paren_term(input: &str) -> PResult<'_, Term> {
    delimited(ws(char('(')), term, cut(ws(char(')')))).parse(input)
}

fn number_term(input: &str) -> PResult<'_, Term> {
    map(ws(fixed_number), number).parse(input)
}

fn default_var_term(input: &str) -> PResult<'_, Term> {
    let (input, name) = ws(variable).parse(input)?;
    let (input, _) = ws(char('@')).parse(input)?;
    let (input, _) = space_or_comment0(input)?;
    let value_start = input.as_ptr() as usize;
    let (input, value) = fixed_number(input)?;
    let value_end = input.as_ptr() as usize;
    // Store raw pointer addresses; database()/query() will convert to byte offsets
    Ok((
        input,
        Term::DefaultVar {
            name,
            value,
            span: Some(SrcSpan {
                start: value_start,
                end: value_end,
            }),
        },
    ))
}

/// 比較演算子 (<, <=, >, >=)
#[derive(Clone, Copy)]
enum CompOp {
    Lt,
    Le,
    Gt,
    Ge,
}

fn comp_op(input: &str) -> PResult<'_, CompOp> {
    ws(alt((
        map(tag("<="), |_| CompOp::Le),
        map(tag(">="), |_| CompOp::Ge),
        map(char('<'), |_| CompOp::Lt),
        map(char('>'), |_| CompOp::Gt),
    )))
    .parse(input)
}

/// range_var: `0 < X < 10`, `X < 10`, `0 < X`, `X > 0` など
fn range_var_term(input: &str) -> PResult<'_, Term> {
    // 左側: (num op)?
    let left_bound = opt((ws(fixed_number), comp_op));
    // 変数名
    let var_name = ws(variable);
    // 右側: (op num)?
    let right_bound = opt((comp_op, ws(fixed_number)));

    map(
        (left_bound, var_name, right_bound),
        |(left, name, right)| {
            let min = match left {
                Some((val, CompOp::Lt)) => Some(Bound {
                    value: val,
                    inclusive: false,
                }),
                Some((val, CompOp::Le)) => Some(Bound {
                    value: val,
                    inclusive: true,
                }),
                Some((_, CompOp::Gt | CompOp::Ge)) => return var(name),
                None => None,
            };

            let max = match right {
                Some((CompOp::Lt, val)) => Some(Bound {
                    value: val,
                    inclusive: false,
                }),
                Some((CompOp::Le, val)) => Some(Bound {
                    value: val,
                    inclusive: true,
                }),
                Some((CompOp::Gt | CompOp::Ge, _)) => return var(name),
                None => None,
            };

            if min.is_none() && max.is_none() {
                var(name)
            } else {
                range_var(name, min, max)
            }
        },
    )
    .parse(input)
}

fn atom_term(input: &str) -> PResult<'_, Term> {
    ws(map(
        pair(
            atom,
            opt(ws(delimited(
                char('('),
                separated_list0(ws(char(',')), term),
                cut(ws(char(')'))),
            ))),
        ),
        |(name, maybe_args)| match maybe_args {
            Some(args) => struc(name, args),
            None => struc(name, vec![]),
        },
    ))
    .parse(input)
}

fn primary_term(input: &str) -> PResult<'_, Term> {
    // range_var_term は number_term より先に試行（0 < X のような形式を正しくパースするため）
    alt((
        list_term,
        paren_term,
        default_var_term,
        range_var_term,
        number_term,
        atom_term,
    ))
    .parse(input)
}

fn mul_op(input: &str) -> PResult<'_, ArithOp> {
    ws(alt((
        map(char('*'), |_| ArithOp::Mul),
        map(char('/'), |_| ArithOp::Div),
    )))
    .parse(input)
}

fn add_op(input: &str) -> PResult<'_, ArithOp> {
    ws(alt((
        map(char('+'), |_| ArithOp::Add),
        map(char('-'), |_| ArithOp::Sub),
    )))
    .parse(input)
}

fn mul_expr(input: &str) -> PResult<'_, Term> {
    let (input, first) = primary_term(input)?;
    let (input, rest) = many0(pair(mul_op, primary_term)).parse(input)?;

    let result = rest
        .into_iter()
        .fold(first, |left, (op, right)| arith_expr(op, left, right));
    Ok((input, result))
}

fn add_expr(input: &str) -> PResult<'_, Term> {
    let (input, first) = mul_expr(input)?;
    let (input, rest) = many0(pair(add_op, mul_expr)).parse(input)?;

    let result = rest
        .into_iter()
        .fold(first, |left, (op, right)| arith_expr(op, left, right));
    Ok((input, result))
}

fn simple_term(input: &str) -> PResult<'_, Term> {
    add_expr(input)
}

/// Pipe operator: `a |> f(b, c)` becomes `f(a, b, c)`
fn pipe_expr(input: &str) -> PResult<'_, Term> {
    let (input, first) = simple_term(input)?;
    let (input, rest) = many0(preceded(ws(tag("|>")), simple_term)).parse(input)?;

    let result = rest.into_iter().fold(first, |acc, rhs| {
        // rhs should be a Struct; prepend acc as first argument
        match rhs {
            Term::Struct { functor, args } => {
                let mut new_args = vec![acc];
                new_args.extend(args);
                struc(functor, new_args)
            }
            // If rhs is not a struct, wrap it as a unary call
            other => struc("apply".to_string(), vec![other, acc]),
        }
    });
    Ok((input, result))
}

pub(super) fn term(input: &str) -> PResult<'_, Term> {
    pipe_expr(input)
}

fn goals(input: &str) -> PResult<'_, Vec<Term>> {
    separated_list1(ws(char(',')), term).parse(input)
}

pub(super) fn clause_parser(input: &str) -> PResult<'_, Clause> {
    ws(terminated(
        alt((
            map(
                separated_pair(term, ws(tag(":-")), goals),
                |(head, body)| Clause::Rule { head, body },
            ),
            map(term, Clause::Fact),
        )),
        cut(ws(char('.'))),
    ))
    .parse(input)
}

pub fn program(input: &str) -> PResult<'_, Vec<Clause>> {
    ws(terminated(many0(clause_parser), opt(space_or_comment1))).parse(input)
}

fn fix_spans_in_term(term: &mut Term, base: usize) {
    match term {
        Term::DefaultVar { span, .. } => {
            if let Some(s) = span {
                s.start = s.start.wrapping_sub(base);
                s.end = s.end.wrapping_sub(base);
            }
        }
        Term::Struct { args, .. } => {
            for arg in args.iter_mut() {
                fix_spans_in_term(arg, base);
            }
        }
        Term::List { items, tail } => {
            for item in items.iter_mut() {
                fix_spans_in_term(item, base);
            }
            if let Some(t) = tail {
                fix_spans_in_term(t, base);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            fix_spans_in_term(left, base);
            fix_spans_in_term(right, base);
        }
        Term::Constraint { left, right } => {
            fix_spans_in_term(left, base);
            fix_spans_in_term(right, base);
        }
        _ => {}
    }
}

fn fix_spans_in_clause(clause: &mut Clause, base: usize) {
    match clause {
        Clause::Fact(term) => fix_spans_in_term(term, base),
        Clause::Rule { head, body } => {
            fix_spans_in_term(head, base);
            for t in body.iter_mut() {
                fix_spans_in_term(t, base);
            }
        }
    }
}

pub fn database(input: &str) -> Result<Vec<Clause>, nom::Err<nom::error::Error<&str>>> {
    let base = input.as_ptr() as usize;
    match program(input) {
        Ok((rest, mut clauses)) if rest.is_empty() => {
            for clause in clauses.iter_mut() {
                fix_spans_in_clause(clause, base);
            }
            Ok(clauses)
        }
        Ok((rest, _)) => Err(nom::Err::Error(nom::error::Error {
            input: rest,
            code: nom::error::ErrorKind::Fail,
        })),
        Err(e) => Err(e),
    }
}

pub fn query(input: &str) -> PResult<'_, Vec<Term>> {
    let base = input.as_ptr() as usize;
    let (rest, mut terms) = ws(terminated(goals, cut(ws(char('.'))))).parse(input)?;
    for term in terms.iter_mut() {
        fix_spans_in_term(term, base);
    }
    Ok((rest, terms))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_clause(src: &str, expected: Clause) {
        let (_, parsed) = clause_parser(src).unwrap();
        assert_eq!(parsed, expected);
    }

    #[test]
    fn parse_fact() {
        assert_clause(
            "parent(alice, bob).",
            Clause::Fact(struc("parent".to_string(), vec![a("alice"), a("bob")])),
        );
    }

    #[test]
    fn parse_rule() {
        assert_clause(
            "grandparent(X, Y) :- parent(X, Z), parent(Z, Y).",
            Clause::Rule {
                head: struc("grandparent".to_string(), vec![v("X"), v("Y")]),
                body: vec![
                    struc("parent".to_string(), vec![v("X"), v("Z")]),
                    struc("parent".to_string(), vec![v("Z"), v("Y")]),
                ],
            },
        );
    }

    #[test]
    fn parse_list() {
        assert_clause(
            "member(X, [X|_]).",
            Clause::Fact(struc(
                "member".to_string(),
                vec![v("X"), list(vec![v("X")], Some(v("_")))],
            )),
        );
    }

    #[test]
    fn parse_query_simple() {
        let src = "member(X, [1,2,3]).";
        let (_, qs) = query(src).unwrap();
        assert_eq!(
            qs,
            vec![struc(
                "member".to_string(),
                vec![v("X"), list(vec![number_int(1), number_int(2), number_int(3)], None),],
            )]
        );
    }

    #[test]
    fn parse_database() {
        let src = r#"
            % facts
            parent(alice, bob).
            parent(bob, carol).

            /* rule */
            grandparent(X, Y) :- parent(X, Z), parent(Z, Y).
        "#;
        let db = database(src).unwrap();
        assert_eq!(db.len(), 3);
    }

    #[test]
    fn test_struct() {
        let src = "parent(alice, f(nested)).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { functor, args } => {
                    assert_eq!(functor, "parent");
                    assert_eq!(args.len(), 2);
                }
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn test_atom() {
        let src = "hello.";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { functor, args } => {
                    assert_eq!(functor, "hello");
                    assert_eq!(args.len(), 0);
                }
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_range_var_both_bounds() {
        let src = "hoge(0<X<10).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { functor, args } => {
                    assert_eq!(functor, "hoge");
                    assert_eq!(args.len(), 1);
                    match &args[0] {
                        Term::RangeVar { name, min, max } => {
                            assert_eq!(name, "X");
                            assert_eq!(
                                *min,
                                Some(Bound {
                                    value: FixedPoint::from_int(0),
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
                        _ => panic!("Expected RangeVar, got {:?}", args[0]),
                    }
                }
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_range_var_inclusive() {
        let src = "hoge(0<=X<=10).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { args, .. } => match &args[0] {
                    Term::RangeVar { name, min, max } => {
                        assert_eq!(name, "X");
                        assert_eq!(
                            *min,
                            Some(Bound {
                                value: FixedPoint::from_int(0),
                                inclusive: true
                            })
                        );
                        assert_eq!(
                            *max,
                            Some(Bound {
                                value: FixedPoint::from_int(10),
                                inclusive: true
                            })
                        );
                    }
                    _ => panic!("Expected RangeVar"),
                },
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_range_var_left_only() {
        let src = "hoge(0<X).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { args, .. } => match &args[0] {
                    Term::RangeVar { name, min, max } => {
                        assert_eq!(name, "X");
                        assert_eq!(
                            *min,
                            Some(Bound {
                                value: FixedPoint::from_int(0),
                                inclusive: false
                            })
                        );
                        assert_eq!(*max, None);
                    }
                    _ => panic!("Expected RangeVar"),
                },
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_range_var_right_only() {
        let src = "hoge(X<10).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { args, .. } => match &args[0] {
                    Term::RangeVar { name, min, max } => {
                        assert_eq!(name, "X");
                        assert_eq!(*min, None);
                        assert_eq!(
                            *max,
                            Some(Bound {
                                value: FixedPoint::from_int(10),
                                inclusive: false
                            })
                        );
                    }
                    _ => panic!("Expected RangeVar"),
                },
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_range_var_in_rule() {
        let src = "hoge(0<X<10) :- cube(X, X, X).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Rule { head, body } => {
                match &head {
                    Term::Struct { functor, args } => {
                        assert_eq!(functor, "hoge");
                        assert!(matches!(&args[0], Term::RangeVar { .. }));
                    }
                    _ => panic!("Expected Struct"),
                }
                assert_eq!(body.len(), 1);
            }
            _ => panic!("Expected Rule"),
        }
    }

    #[test]
    fn parse_default_var() {
        let src = "hoge(X@25).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { args, .. } => match &args[0] {
                    Term::DefaultVar { name, value, .. } => {
                        assert_eq!(name, "X");
                        assert_eq!(*value, FixedPoint::from_int(25));
                    }
                    _ => panic!("Expected DefaultVar"),
                },
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_fixed_point_integer() {
        let src = "f(42).";
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Fact(Term::Struct { args, .. }) => match &args[0] {
                Term::Number { value } => assert_eq!(*value, FixedPoint::from_int(42)),
                _ => panic!("Expected Number"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_fixed_point_decimal() {
        let src = "f(100.01).";
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Fact(Term::Struct { args, .. }) => match &args[0] {
                Term::Number { value } => assert_eq!(*value, FixedPoint::from_hundredths(10001)),
                _ => panic!("Expected Number"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_fixed_point_negative_decimal() {
        let src = "f(-3.5).";
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Fact(Term::Struct { args, .. }) => match &args[0] {
                Term::Number { value } => assert_eq!(*value, FixedPoint::from_hundredths(-350)),
                _ => panic!("Expected Number"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_fixed_point_display() {
        assert_eq!(format!("{}", FixedPoint::from_int(100)), "100");
        assert_eq!(format!("{}", FixedPoint::from_hundredths(10001)), "100.01");
        assert_eq!(format!("{}", FixedPoint::from_hundredths(350)), "3.5");
        assert_eq!(format!("{}", FixedPoint::from_hundredths(-350)), "-3.5");
    }

    #[test]
    fn parse_default_var_decimal() {
        let src = "hoge(X@2.5).";
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Fact(Term::Struct { args, .. }) => match &args[0] {
                Term::DefaultVar { name, value, .. } => {
                    assert_eq!(name, "X");
                    assert_eq!(*value, FixedPoint::from_hundredths(250));
                }
                _ => panic!("Expected DefaultVar"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_pipe_operator_chain() {
        // a |> b |> c should become c(b(a))
        let src = "cube(1,1,1) |> scale(2) |> translate(5,5,5).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { functor, args } => {
                    assert_eq!(functor, "translate");
                    assert_eq!(args.len(), 4);
                    // First arg should be scale(cube(1,1,1), 2)
                    match &args[0] {
                        Term::Struct { functor, args } => {
                            assert_eq!(functor, "scale");
                            assert_eq!(args.len(), 2);
                            // First arg of scale should be cube(1,1,1)
                            match &args[0] {
                                Term::Struct { functor, .. } => {
                                    assert_eq!(functor, "cube");
                                }
                                _ => panic!("Expected cube"),
                            }
                        }
                        _ => panic!("Expected scale"),
                    }
                }
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_pipe_with_parentheses() {
        // (cube(10,20,30) |> translate(10,0,0)) + cube(100,1,1)
        // should become: union(translate(cube(10,20,30), 10,0,0), cube(100,1,1))
        let src = "(cube(10,20,30) |> translate(10,0,0)) + cube(100,1,1).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::InfixExpr { op, left, right } => {
                    assert_eq!(*op, ArithOp::Add);
                    // left should be translate(cube(10,20,30), 10, 0, 0)
                    match left.as_ref() {
                        Term::Struct { functor, args } => {
                            assert_eq!(functor, "translate");
                            assert_eq!(args.len(), 4);
                            match &args[0] {
                                Term::Struct { functor, .. } => {
                                    assert_eq!(functor, "cube");
                                }
                                _ => panic!("Expected cube"),
                            }
                        }
                        _ => panic!("Expected Struct for left, got {:?}", left),
                    }
                    // right should be cube(100,1,1)
                    match right.as_ref() {
                        Term::Struct { functor, args } => {
                            assert_eq!(functor, "cube");
                            assert_eq!(args.len(), 3);
                        }
                        _ => panic!("Expected Struct for right"),
                    }
                }
                _ => panic!("Expected ArithExpr, got {:?}", term),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_pipe_without_parentheses() {
        // cube(10,20,30) |> translate(10,0,0) + cube(100,1,1)
        // Without parentheses, + binds tighter, so this becomes:
        // cube(10,20,30) |> (translate(10,0,0) + cube(100,1,1))
        // which is apply(translate(10,0,0) + cube(100,1,1), cube(10,20,30))
        // But since translate + cube is ArithExpr not Struct, it wraps with "apply"
        let src = "cube(10,20,30) |> translate(10,0,0) + cube(100,1,1).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => {
                // This should be apply(ArithExpr, cube)
                match &term {
                    Term::Struct { functor, .. } => {
                        assert_eq!(functor, "apply");
                    }
                    _ => panic!("Expected apply Struct, got {:?}", term),
                }
            }
            _ => panic!("Expected Fact"),
        }
    }
}
