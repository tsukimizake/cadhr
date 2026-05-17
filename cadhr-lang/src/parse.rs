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

/// パーサーでは file_id=0 で生成し、モジュール読み込み時に正しい file_id に書き換える
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SrcSpan {
    pub start: usize,
    pub end: usize,
    pub file_id: u16,
}

pub struct LineCol {
    pub line: usize,
    pub col: usize,
}

impl SrcSpan {
    pub fn start_line_col(&self, src: &str) -> LineCol {
        let offset = self.start.min(src.len());
        let before = &src[..offset];
        let line = before.chars().filter(|&c| c == '\n').count() + 1;
        let col = before.rfind('\n').map_or(offset, |pos| offset - pos - 1) + 1;
        LineCol { line, col }
    }
}

#[derive(Default, Clone)]
pub struct FileRegistry {
    files: Vec<(String, String)>,
}

impl FileRegistry {
    pub fn new() -> Self {
        Self { files: Vec::new() }
    }

    pub fn register_main(&mut self, name: String, source: String) -> u16 {
        assert!(self.files.is_empty());
        self.files.push((name, source));
        0
    }

    pub fn register(&mut self, name: String, source: String) -> u16 {
        let id = self.files.len() as u16;
        self.files.push((name, source));
        id
    }

    pub fn file_name(&self, file_id: u16) -> Option<&str> {
        self.files.get(file_id as usize).map(|(name, _)| name.as_str())
    }

    pub fn source(&self, file_id: u16) -> Option<&str> {
        self.files.get(file_id as usize).map(|(_, src)| src.as_str())
    }

    pub fn format_span(&self, span: &SrcSpan) -> String {
        let name = self.file_name(span.file_id).unwrap_or("<unknown>");
        if let Some(src) = self.source(span.file_id) {
            let lc = span.start_line_col(src);
            format!("{}:{}:{}", name, lc.line, lc.col)
        } else {
            format!("{}:?:?", name)
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct Bound {
    pub value: crate::rational::Rational,
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
pub enum Term<Scope = ()> {
    Var {
        name: String,
        scope: Scope,
        default_value: Option<crate::rational::Rational>,
        min: Option<Bound>,
        max: Option<Bound>,
        span: Option<SrcSpan>,
    },
    Number {
        value: crate::rational::Rational,
    },
    /// 中置演算子式: left op right (算術演算 or CSG演算)
    InfixExpr {
        op: ArithOp,
        left: Box<Term<Scope>>,
        right: Box<Term<Scope>>,
    },
    Struct {
        functor: String,
        args: Vec<Term<Scope>>,
        span: Option<SrcSpan>,
    },
    List {
        items: Vec<Term<Scope>>,
        tail: Option<Box<Term<Scope>>>,
    },
    /// 文字列リテラル: "hello" など
    StringLit {
        value: String,
    },
    /// 等値ゴール `left = right` を表す AST ノード。パース時には常にこの形で生成され、
    /// 実行時に `resolve_equality_goals` (term_rewrite.rs) が両辺を見て:
    ///   - 両側が ArithExpr に変換可能 → 算術連立方程式として線形ソルバへ
    ///   - それ以外 (Struct/List を含む) → unify() で構造的単一化
    /// のいずれかにディスパッチする。Prolog の `=/2` 相当だが算術ソルバも兼ねる点が拡張。
    Eq {
        left: Box<Term<Scope>>,
        right: Box<Term<Scope>>,
    },
}

pub type ScopeId = usize;

pub type ScopedTerm = Term<ScopeId>;

/// Var に紐づく注釈 (default / min / max / source span)。同じ論理変数の全 Term::Var インスタンスは
/// `ScopedEnv` の Union-Find で正規化された 1 つの値を共有する。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct VarAnnotation {
    pub default_value: Option<crate::rational::Rational>,
    pub min: Option<Bound>,
    pub max: Option<Bound>,
    pub span: Option<SrcSpan>,
}

impl VarAnnotation {
    /// `other` の各フィールドを self が未設定の場合のみ受け入れる。self を優先。
    pub fn absorb(&mut self, other: &VarAnnotation) {
        if self.default_value.is_none() {
            self.default_value = other.default_value.clone();
        }
        if self.min.is_none() {
            self.min = other.min.clone();
        }
        if self.max.is_none() {
            self.max = other.max.clone();
        }
        if self.span.is_none() {
            self.span = other.span;
        }
    }

    pub fn has_any(&self) -> bool {
        self.default_value.is_some() || self.min.is_some() || self.max.is_some()
    }
}

/// Number または Var(default_value付き) から Rational と SrcSpan を取り出す。
pub fn term_as_number<S>(term: &Term<S>) -> Option<(crate::rational::Rational, Option<SrcSpan>)> {
    match term {
        Term::Number { value } => Some((value.clone(), None)),
        Term::Var {
            default_value: Some(value),
            span,
            ..
        } => Some((value.clone(), *span)),
        _ => None,
    }
}

pub fn first_span<S>(term: &Term<S>) -> Option<SrcSpan> {
    match term {
        Term::Var { span, .. } | Term::Struct { span, .. } => {
            if span.is_some() {
                return *span;
            }
            if let Term::Struct { args, .. } = term {
                for arg in args {
                    if let Some(s) = first_span(arg) {
                        return Some(s);
                    }
                }
            }
            None
        }
        Term::InfixExpr { left, right, .. } | Term::Eq { left, right } => {
            first_span(left).or_else(|| first_span(right))
        }
        Term::List { items, tail } => {
            for item in items {
                if let Some(s) = first_span(item) {
                    return Some(s);
                }
            }
            tail.as_ref().and_then(|t| first_span(t))
        }
        _ => None,
    }
}

impl<Scope: PartialEq> PartialEq for Term<Scope> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (
                Term::Var {
                    name: n1,
                    scope: s1,
                    default_value: dv1,
                    min: min1,
                    max: max1,
                    ..
                },
                Term::Var {
                    name: n2,
                    scope: s2,
                    default_value: dv2,
                    min: min2,
                    max: max2,
                    ..
                },
            ) => n1 == n2 && s1 == s2 && dv1 == dv2 && min1 == min2 && max1 == max2,
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
                    ..
                },
                Term::Struct {
                    functor: f2,
                    args: a2,
                    ..
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
            (Term::StringLit { value: v1 }, Term::StringLit { value: v2 }) => v1 == v2,
            (
                Term::Eq {
                    left: l1,
                    right: r1,
                },
                Term::Eq {
                    left: l2,
                    right: r2,
                },
            ) => l1 == l2 && r1 == r2,
            _ => false,
        }
    }
}

#[derive(Clone, PartialEq)]
pub struct RecordField<Scope = ()> {
    pub name: String,
    pub default: Option<Term<Scope>>,
}

impl<S> fmt::Debug for RecordField<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.default {
            Some(d) => write!(f, "{}={:?}", self.name, d),
            None => write!(f, "{}", self.name),
        }
    }
}

#[derive(Clone, PartialEq)]
pub enum Clause<Scope = ()> {
    Fact(Term<Scope>),
    Rule {
        head: Term<Scope>,
        body: Vec<Term<Scope>>,
    },
    Use {
        path: String,
        expose: Vec<String>,
        span: Option<SrcSpan>,
    },
    /// `:- record NAME(FIELD, FIELD=DEFAULT, ...).` ディレクティブ。
    /// SWI-Prolog の library(record) を参考に、名前付き field をもつレコードを宣言する。
    /// resolve_modules 時に decl テーブルに登録され、`NAME_FIELD/2` (getter) と
    /// `set_FIELD_of_NAME/3` (setter) が Rule として自動 inject される。
    /// `make_NAME/2` は decl テーブルを参照する builtin として処理される。
    RecordDecl {
        name: String,
        fields: Vec<RecordField<Scope>>,
        span: Option<SrcSpan>,
    },
}

pub type ScopedClause = Clause<ScopeId>;

impl<Scope> fmt::Debug for Term<Scope> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Term::Var {
                name,
                default_value,
                min,
                max,
                ..
            } => {
                if let Some(b) = min {
                    write!(f, "{} {} ", b.value, if b.inclusive { "<=" } else { "<" })?;
                }
                write!(f, "{}", name)?;
                if let Some(dv) = default_value {
                    write!(f, "@{}", dv)?;
                }
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
            Term::Struct { functor, args, .. } => {
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
            Term::StringLit { value } => write!(f, "\"{}\"", value),
            Term::Eq { left, right } => {
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
            Clause::Use { path, expose, .. } => {
                write!(f, "#use(\"{}\"", path)?;
                if !expose.is_empty() {
                    write!(f, ", expose({:?})", expose)?;
                }
                write!(f, ").")
            }
            Clause::RecordDecl { name, fields, .. } => {
                write!(f, ":- record {}(", name)?;
                for (idx, field) in fields.iter().enumerate() {
                    if idx > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{}", field.name)?;
                    if let Some(default) = &field.default {
                        write!(f, "={:?}", default)?;
                    }
                }
                write!(f, ").")
            }
        }
    }
}

/// Termコンストラクタ
pub fn var(name: String) -> Term {
    Term::Var {
        name,
        scope: (),
        default_value: None,
        min: None,
        max: None,
        span: None,
    }
}

pub fn var_with_span(name: String, span: SrcSpan) -> Term {
    Term::Var {
        name,
        scope: (),
        default_value: None,
        min: None,
        max: None,
        span: Some(span),
    }
}

pub fn default_var(name: String, value: crate::rational::Rational) -> Term {
    Term::Var {
        name,
        scope: (),
        default_value: Some(value),
        min: None,
        max: None,
        span: None,
    }
}

pub fn default_var_with_span(name: String, value: crate::rational::Rational, span: SrcSpan) -> Term {
    Term::Var {
        name,
        scope: (),
        default_value: Some(value),
        min: None,
        max: None,
        span: Some(span),
    }
}

pub fn annotated_var(
    name: String,
    default_value: Option<crate::rational::Rational>,
    min: Option<Bound>,
    max: Option<Bound>,
    span: Option<SrcSpan>,
) -> Term {
    Term::Var {
        name,
        scope: (),
        default_value,
        min,
        max,
        span,
    }
}

pub fn number<S>(value: crate::rational::Rational) -> Term<S> {
    Term::Number { value }
}

pub fn number_int<S>(value: i64) -> Term<S> {
    Term::Number {
        value: crate::rational::Rational::from_integer(value),
    }
}

pub fn struc<S>(functor: String, args: Vec<Term<S>>) -> Term<S> {
    Term::Struct {
        functor,
        args,
        span: None,
    }
}

pub fn struc_with_span<S>(functor: String, args: Vec<Term<S>>, span: SrcSpan) -> Term<S> {
    Term::Struct {
        functor,
        args,
        span: Some(span),
    }
}

pub fn list<S>(items: Vec<Term<S>>, tail: Option<Term<S>>) -> Term<S> {
    Term::List {
        items,
        tail: tail.map(Box::new),
    }
}

pub fn range_var(name: String, min: Option<Bound>, max: Option<Bound>) -> Term {
    Term::Var {
        name,
        scope: (),
        default_value: None,
        min,
        max,
        span: None,
    }
}

pub fn string_lit<S>(value: String) -> Term<S> {
    Term::StringLit { value }
}

pub fn arith_expr<S>(op: ArithOp, left: Term<S>, right: Term<S>) -> Term<S> {
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

/// 数値リテラル parser。`"123"`, `"-3.5"`, `"1.234567"` などを受け入れ、
/// 厳密な有理数 (`Rational`) を返す。整数 / 小数の桁数に制限はない。
/// 内部的には「整数部 * 10^n + 小数部」を分子、`10^n` を分母として構築する。
fn fixed_number(input: &str) -> PResult<'_, crate::rational::Rational> {
    use num_bigint::BigInt;
    use num_rational::BigRational;
    use std::str::FromStr;

    map_res(
        recognize((opt(char('-')), digit1, opt(pair(char('.'), digit1)))),
        |s: &str| -> Result<crate::rational::Rational, String> {
            if let Some(dot_pos) = s.find('.') {
                let int_part_str = &s[..dot_pos];
                let frac_str = &s[dot_pos + 1..];
                let sign_is_neg = int_part_str.starts_with('-');
                let int_abs_str = if sign_is_neg { &int_part_str[1..] } else { int_part_str };
                let int_abs = BigInt::from_str(int_abs_str).map_err(|e| e.to_string())?;
                let frac_val = BigInt::from_str(frac_str).map_err(|e| e.to_string())?;
                let frac_digits = frac_str.len() as u32;
                let scale = BigInt::from(10).pow(frac_digits);
                let numer_abs = &int_abs * &scale + &frac_val;
                let numer = if sign_is_neg { -numer_abs } else { numer_abs };
                let r = BigRational::new(numer, scale);
                Ok(crate::rational::Rational::from_big_rational(r))
            } else {
                let v = BigInt::from_str(s).map_err(|e| e.to_string())?;
                let r = BigRational::from(v);
                Ok(crate::rational::Rational::from_big_rational(r))
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

fn default_value_suffix(input: &str) -> PResult<'_, (crate::rational::Rational, SrcSpan)> {
    let (input, _) = ws(char('@')).parse(input)?;
    let (input, _) = space_or_comment0(input)?;
    let value_start = input.as_ptr() as usize;
    let (input, value) = fixed_number(input)?;
    let value_end = input.as_ptr() as usize;
    Ok((
        input,
        (
            value,
            SrcSpan {
                start: value_start,
                end: value_end,
                file_id: 0,
            },
        ),
    ))
}

/// annotated_var: `X@25`, `0<X@20<50`, `0<X<10`, `X<10`, `0<X` など
/// left_bound? Variable (@value)? right_bound?
fn annotated_var_term(input: &str) -> PResult<'_, Term> {
    // 左側: (num op)?
    let (input, left) = opt((ws(fixed_number), comp_op)).parse(input)?;
    // 変数名
    let var_name_start = input.as_ptr() as usize;
    let (input, name) = ws(variable).parse(input)?;
    let var_name_end = input.as_ptr() as usize;
    let var_name_span = SrcSpan {
        start: var_name_start,
        end: var_name_end,
        file_id: 0,
    };
    // =value (optional)
    let (input, default_with_span) = opt(default_value_suffix).parse(input)?;
    // 右側: (op num)?
    let (input, right) = opt((comp_op, ws(fixed_number))).parse(input)?;

    let min = match left {
        Some((val, CompOp::Lt)) => Some(Bound {
            value: val,
            inclusive: false,
        }),
        Some((val, CompOp::Le)) => Some(Bound {
            value: val,
            inclusive: true,
        }),
        Some((_, CompOp::Gt | CompOp::Ge)) => {
            return Ok((input, var_with_span(name, var_name_span)));
        }
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
        Some((CompOp::Gt | CompOp::Ge, _)) => {
            return Ok((input, var_with_span(name, var_name_span)));
        }
        None => None,
    };

    let (default_value, span) = match default_with_span {
        Some((val, sp)) => (Some(val), Some(sp)),
        None => (
            None,
            Some(SrcSpan {
                start: var_name_span.end,
                end: var_name_span.end,
                file_id: 0,
            }),
        ),
    };

    if min.is_none() && max.is_none() && default_value.is_none() {
        Ok((input, var_with_span(name, var_name_span)))
    } else {
        Ok((
            input,
            Term::Var {
                name,
                scope: (),
                default_value,
                min,
                max,
                span,
            },
        ))
    }
}

fn string_literal(input: &str) -> PResult<'_, Term> {
    ws(map(
        delimited(
            char('"'),
            many0(alt((
                map(tag("\\\""), |_| '"'),
                map(tag("\\\\"), |_| '\\'),
                map(tag("\\n"), |_| '\n'),
                map(tag("\\t"), |_| '\t'),
                nom::character::complete::none_of("\"\\"),
            ))),
            cut(char('"')),
        ),
        |chars| string_lit(chars.into_iter().collect()),
    ))
    .parse(input)
}

fn atom_term(input: &str) -> PResult<'_, Term> {
    let (input, _) = space_or_comment0(input)?;
    let start = input.as_ptr() as usize;
    let (input, name) = atom(input)?;
    let (input, name) =
        if let Ok((input2, _)) = tag::<&str, &str, nom::error::Error<&str>>("::")(input) {
            if let Ok((input2, qualified_name)) = atom(input2) {
                (input2, format!("{}::{}", name, qualified_name))
            } else {
                (input, name)
            }
        } else {
            (input, name)
        };
    let (input, maybe_args) = opt(ws(delimited(
        char('('),
        separated_list0(ws(char(',')), term),
        cut(ws(char(')'))),
    )))
    .parse(input)?;
    let end = input.as_ptr() as usize;
    let (input, _) = space_or_comment0(input)?;
    let span = SrcSpan { start, end, file_id: 0 };
    let t = match maybe_args {
        Some(args) => struc_with_span(name, args, span),
        None => struc_with_span(name, vec![], span),
    };
    Ok((input, t))
}

fn primary_term(input: &str) -> PResult<'_, Term> {
    // annotated_var_term は number_term より先に試行（0 < X のような形式を正しくパースするため）
    alt((
        list_term,
        paren_term,
        string_literal,
        annotated_var_term,
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

    let result = rest.into_iter().fold(first, |acc, rhs| match rhs {
        Term::Struct {
            functor,
            args,
            span,
        } => {
            let mut new_args = vec![acc];
            new_args.extend(args);
            Term::Struct {
                functor,
                args: new_args,
                span,
            }
        }
        other => struc("apply".to_string(), vec![other, acc]),
    });
    Ok((input, result))
}

pub(super) fn term(input: &str) -> PResult<'_, Term> {
    pipe_expr(input)
}

/// goal内の等値: `term = term` → Term::Eq { left, right }
///
/// パース時には常に `Term::Eq` を生成する。実行時 (`resolve_equality_goals`) に
/// 左右の resolved 形を見て、両側が ArithExpr に変換できれば算術制約として、
/// 変換不能な Struct/List が含まれれば構造的単一化 (unify) として処理する。
/// `X = Y` の Var 同士は parse 時には判別不能 (後で Struct に束縛される可能性がある)
/// なので、判別を実行時まで遅延する。
fn eq_goal(input: &str) -> PResult<'_, Term> {
    let (input, left) = term(input)?;
    let (input, rhs) = opt(preceded(ws(char('=')), term)).parse(input)?;
    match rhs {
        Some(right) => Ok((
            input,
            Term::Eq {
                left: Box::new(left),
                right: Box::new(right),
            },
        )),
        None => Ok((input, left)),
    }
}

fn goals(input: &str) -> PResult<'_, Vec<Term>> {
    separated_list1(ws(char(',')), eq_goal).parse(input)
}

fn use_expose_list(input: &str) -> PResult<'_, Vec<String>> {
    preceded(
        ws(tag("expose")),
        ws(delimited(
            char('('),
            ws(delimited(
                char('['),
                separated_list0(ws(char(',')), atom),
                cut(ws(char(']'))),
            )),
            cut(ws(char(')'))),
        )),
    )
    .parse(input)
}

/// `:- record NAME(FIELD, FIELD=DEFAULT, ...).` を Clause::RecordDecl にパースする。
/// FIELD は atom、DEFAULT は任意の term (パイプ式まで)。
fn record_directive(input: &str) -> PResult<'_, Clause> {
    let (input, _) = space_or_comment0(input)?;
    let start = input.as_ptr() as usize;
    let (input, _) = tag(":-")(input)?;
    let (input, _) = ws(tag("record")).parse(input)?;
    let (input, name) = ws(atom).parse(input)?;
    let (input, _) = char('(').parse(input)?;
    let (input, fields) = separated_list0(
        ws(char(',')),
        ws(map(
            pair(atom, opt(preceded(ws(char('=')), term))),
            |(name, default)| RecordField { name, default },
        )),
    )
    .parse(input)?;
    let (input, _) = cut(ws(char(')'))).parse(input)?;
    let end = input.as_ptr() as usize;
    let (input, _) = cut(ws(char('.'))).parse(input)?;
    let span = SrcSpan {
        start,
        end,
        file_id: 0,
    };
    Ok((
        input,
        Clause::RecordDecl {
            name,
            fields,
            span: Some(span),
        },
    ))
}

fn use_directive(input: &str) -> PResult<'_, Clause> {
    let (input, _) = space_or_comment0(input)?;
    let start = input.as_ptr() as usize;
    let (input, _) = tag("#use")(input)?;
    let (input, _) = char('(').parse(input)?;
    let (input, path) = ws(delimited(
        char('"'),
        map(
            many0(alt((
                map(tag("\\\""), |_: &str| '"'),
                map(tag("\\\\"), |_: &str| '\\'),
                nom::character::complete::none_of("\"\\"),
            ))),
            |chars| chars.into_iter().collect::<String>(),
        ),
        cut(char('"')),
    ))
    .parse(input)?;
    let (input, expose) = opt(preceded(ws(char(',')), use_expose_list)).parse(input)?;
    let (input, _) = char(')').parse(input)?;
    let end = input.as_ptr() as usize;
    let (input, _) = char('.').parse(input)?;
    let span = SrcSpan { start, end, file_id: 0 };
    Ok((
        input,
        Clause::Use {
            path,
            expose: expose.unwrap_or_default(),
            span: Some(span),
        },
    ))
}

fn has_range_in_term(term: &Term) -> bool {
    match term {
        Term::Var { min, max, .. } => min.is_some() || max.is_some(),
        Term::Struct { args, .. } => args.iter().any(has_range_in_term),
        Term::List { items, tail } => {
            items.iter().any(has_range_in_term)
                || tail.as_ref().is_some_and(|t| has_range_in_term(t))
        }
        Term::InfixExpr { left, right, .. } => has_range_in_term(left) || has_range_in_term(right),
        _ => false,
    }
}

pub(super) fn clause_parser(input: &str) -> PResult<'_, Clause> {
    let (input, clause) = alt((
        use_directive,
        record_directive,
        ws(terminated(
            alt((
                map(
                    separated_pair(term, ws(tag(":-")), goals),
                    |(head, body)| Clause::Rule { head, body },
                ),
                map(term, Clause::Fact),
            )),
            cut(ws(char('.'))),
        )),
    ))
    .parse(input)?;

    let head = match &clause {
        Clause::Rule { head, .. } => Some(head),
        Clause::Fact(head) => Some(head),
        Clause::Use { .. } | Clause::RecordDecl { .. } => None,
    };
    if let Some(head) = head {
        if has_range_in_term(head) {
            return Err(nom::Err::Failure(nom::error::Error::new(
                input,
                nom::error::ErrorKind::Verify,
            )));
        }
    }

    Ok((input, clause))
}

pub fn program(input: &str) -> PResult<'_, Vec<Clause>> {
    ws(terminated(many0(clause_parser), opt(space_or_comment1))).parse(input)
}

fn fix_spans_in_term(term: &mut Term, base: usize) {
    match term {
        Term::Var { span, .. } => {
            if let Some(s) = span {
                s.start = s.start.wrapping_sub(base);
                s.end = s.end.wrapping_sub(base);
            }
        }
        Term::Struct { args, span, .. } => {
            if let Some(s) = span {
                s.start = s.start.wrapping_sub(base);
                s.end = s.end.wrapping_sub(base);
            }
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
        Term::Eq { left, right } => {
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
        Clause::Use { span, .. } => {
            if let Some(s) = span {
                s.start = s.start.wrapping_sub(base);
                s.end = s.end.wrapping_sub(base);
            }
        }
        Clause::RecordDecl { fields, span, .. } => {
            if let Some(s) = span {
                s.start = s.start.wrapping_sub(base);
                s.end = s.end.wrapping_sub(base);
            }
            for field in fields.iter_mut() {
                if let Some(default) = &mut field.default {
                    fix_spans_in_term(default, base);
                }
            }
        }
    }
}

pub fn parse_error_span(input: &str, err: &nom::Err<nom::error::Error<&str>>) -> Option<SrcSpan> {
    let base = input.as_ptr() as usize;
    let rest = match err {
        nom::Err::Error(e) | nom::Err::Failure(e) => e.input,
        nom::Err::Incomplete(_) => return None,
    };
    let start = rest.as_ptr() as usize - base;
    let end = (start + rest.len()).min(input.len());
    Some(SrcSpan { start, end, file_id: 0 })
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

/// query変数のパラメータ情報（UIスライダー用）
#[derive(Clone, Debug)]
pub struct QueryParam {
    pub name: String,
    pub min: Option<Bound>,
    pub max: Option<Bound>,
    pub default_value: Option<crate::rational::Rational>,
}

fn collect_query_params_from_term(term: &Term, params: &mut Vec<QueryParam>) {
    match term {
        Term::Var {
            name,
            default_value,
            min,
            max,
            ..
        } if name != "_" => {
            if !params.iter().any(|p| p.name == *name) {
                params.push(QueryParam {
                    name: name.clone(),
                    min: min.clone(),
                    max: max.clone(),
                    default_value: default_value.clone(),
                });
            }
        }
        Term::Struct { args, .. } => {
            for arg in args {
                collect_query_params_from_term(arg, params);
            }
        }
        Term::List { items, tail } => {
            for item in items {
                collect_query_params_from_term(item, params);
            }
            if let Some(t) = tail {
                collect_query_params_from_term(t, params);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            collect_query_params_from_term(left, params);
            collect_query_params_from_term(right, params);
        }
        _ => {}
    }
}

/// execute結果のtermからVar/Varを走査してQueryParamsを抽出する。
/// unificationでhead側rangeが伝播済みの状態を想定。
pub fn collect_query_params(terms: &[Term]) -> Vec<QueryParam> {
    let mut params = Vec::new();
    for term in terms {
        collect_query_params_from_term(term, &mut params);
    }
    params
}

/// term内のVar/Varを値で置換する
pub fn substitute_query_params(
    terms: &[Term],
    values: &std::collections::HashMap<String, f64>,
) -> Vec<Term> {
    terms.iter().map(|t| substitute_term(t, values)).collect()
}

fn substitute_term(term: &Term, values: &std::collections::HashMap<String, f64>) -> Term {
    match term {
        Term::Var {
            name, min, max, span, ..
        } if name != "_" => {
            if let Some(&val) = values.get(name) {
                Term::Var {
                    name: name.clone(),
                    scope: (),
                    default_value: Some(crate::rational::Rational::from_f64(val)),
                    min: min.clone(),
                    max: max.clone(),
                    span: *span,
                }
            } else {
                term.clone()
            }
        }
        Term::Struct {
            functor,
            args,
            span,
        } => Term::Struct {
            functor: functor.clone(),
            args: args.iter().map(|a| substitute_term(a, values)).collect(),
            span: *span,
        },
        Term::List { items, tail } => Term::List {
            items: items.iter().map(|i| substitute_term(i, values)).collect(),
            tail: tail.as_ref().map(|t| Box::new(substitute_term(t, values))),
        },
        Term::InfixExpr { op, left, right } => Term::InfixExpr {
            op: *op,
            left: Box::new(substitute_term(left, values)),
            right: Box::new(substitute_term(right, values)),
        },
        _ => term.clone(),
    }
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
    fn parse_record_decl_no_defaults() {
        let (_, parsed) = clause_parser(":- record point(x, y).").unwrap();
        match parsed {
            Clause::RecordDecl { name, fields, .. } => {
                assert_eq!(name, "point");
                assert_eq!(fields.len(), 2);
                assert_eq!(fields[0].name, "x");
                assert!(fields[0].default.is_none());
                assert_eq!(fields[1].name, "y");
                assert!(fields[1].default.is_none());
            }
            _ => panic!("expected RecordDecl, got {:?}", parsed),
        }
    }

    #[test]
    fn parse_record_decl_with_defaults() {
        let (_, parsed) =
            clause_parser(":- record output(models=[], bom=[], controls=[]).").unwrap();
        match parsed {
            Clause::RecordDecl { name, fields, .. } => {
                assert_eq!(name, "output");
                assert_eq!(fields.len(), 3);
                for field in &fields {
                    assert!(matches!(
                        &field.default,
                        Some(Term::List { items, tail }) if items.is_empty() && tail.is_none()
                    ));
                }
            }
            _ => panic!("expected RecordDecl, got {:?}", parsed),
        }
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
                vec![
                    v("X"),
                    list(vec![number_int(1), number_int(2), number_int(3)], None),
                ],
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
                Term::Struct { functor, args, .. } => {
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
                Term::Struct { functor, args, .. } => {
                    assert_eq!(functor, "hello");
                    assert_eq!(args.len(), 0);
                }
                _ => panic!("Expected Struct"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_range_var_in_body() {
        let src = "hoge(X) :- 0<X<10, cube(X, X, X).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Rule { head, body } => {
                match &head {
                    Term::Struct { functor, args, .. } => {
                        assert_eq!(functor, "hoge");
                        assert!(matches!(&args[0], Term::Var { .. }));
                    }
                    _ => panic!("Expected Struct"),
                }
                assert_eq!(body.len(), 2);
                match &body[0] {
                    Term::Var { name, min, max, .. } => {
                        assert_eq!(name, "X");
                        assert_eq!(
                            *min,
                            Some(Bound {
                                value: crate::rational::Rational::from_integer(0),
                                inclusive: false
                            })
                        );
                        assert_eq!(
                            *max,
                            Some(Bound {
                                value: crate::rational::Rational::from_integer(10),
                                inclusive: false
                            })
                        );
                    }
                    _ => panic!("Expected Var, got {:?}", body[0]),
                }
            }
            _ => panic!("Expected Rule"),
        }
    }

    #[test]
    fn parse_range_var_inclusive_in_body() {
        let src = "hoge(X) :- 0<=X<=10, cube(X).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Rule { body, .. } => match &body[0] {
                Term::Var { name, min, max, .. } => {
                    assert_eq!(name, "X");
                    assert!(min.as_ref().unwrap().inclusive);
                    assert!(max.as_ref().unwrap().inclusive);
                }
                _ => panic!("Expected Var"),
            },
            _ => panic!("Expected Rule"),
        }
    }

    #[test]
    fn parse_range_var_in_head_is_error() {
        let src = "hoge(0<X<10).";
        assert!(clause_parser(src).is_err());
    }

    #[test]
    fn parse_default_var() {
        let src = "hoge(X@25).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { args, .. } => match &args[0] {
                    Term::Var {
                        name,
                        default_value,
                        ..
                    } => {
                        assert_eq!(name, "X");
                        assert_eq!(*default_value, Some(crate::rational::Rational::from_integer(25)));
                    }
                    _ => panic!("Expected Var"),
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
                Term::Number { value } => assert_eq!(*value, crate::rational::Rational::from_integer(42)),
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
                Term::Number { value } => assert_eq!(*value, crate::rational::Rational::from_ratio(10001, 100)),
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
                Term::Number { value } => assert_eq!(*value, crate::rational::Rational::from_ratio(-350, 100)),
                _ => panic!("Expected Number"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_fixed_point_display() {
        assert_eq!(format!("{}", crate::rational::Rational::from_integer(100)), "100");
        assert_eq!(format!("{}", crate::rational::Rational::from_ratio(10001, 100)), "100.01");
        assert_eq!(format!("{}", crate::rational::Rational::from_ratio(350, 100)), "3.5");
        assert_eq!(format!("{}", crate::rational::Rational::from_ratio(-350, 100)), "-3.5");
    }

    #[test]
    fn parse_default_var_decimal() {
        let src = "hoge(X@2.5).";
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Fact(Term::Struct { args, .. }) => match &args[0] {
                Term::Var {
                    name,
                    default_value,
                    ..
                } => {
                    assert_eq!(name, "X");
                    assert_eq!(*default_value, Some(crate::rational::Rational::from_ratio(250, 100)));
                }
                _ => panic!("Expected Var"),
            },
            _ => panic!("Expected Fact"),
        }
    }

    #[test]
    fn parse_pipe_operator_chain() {
        // a |> b |> c should become c(b(a))
        let src = "cube(1,1,1) |> scale(2) |> translate(p(0,0,0), p(5,5,5)).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::Struct { functor, args, .. } => {
                    assert_eq!(functor, "translate");
                    assert_eq!(args.len(), 3);
                    // First arg should be scale(cube(1,1,1), 2)
                    match &args[0] {
                        Term::Struct { functor, args, .. } => {
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
        // (cube(10,20,30) |> translate(p(0,0,0), p(10,0,0))) + cube(100,1,1)
        // should become: union(translate(cube(10,20,30), p(0,0,0), p(10,0,0)), cube(100,1,1))
        let src = "(cube(10,20,30) |> translate(p(0,0,0), p(10,0,0))) + cube(100,1,1).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Fact(term) => match &term {
                Term::InfixExpr { op, left, right } => {
                    assert_eq!(*op, ArithOp::Add);
                    // left should be translate(cube(10,20,30), p(0,0,0), p(10,0,0))
                    match left.as_ref() {
                        Term::Struct { functor, args, .. } => {
                            assert_eq!(functor, "translate");
                            assert_eq!(args.len(), 3);
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
                        Term::Struct { functor, args, .. } => {
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
        // cube(10,20,30) |> translate(p(0,0,0), p(10,0,0)) + cube(100,1,1)
        // Without parentheses, + binds tighter, so this becomes:
        // cube(10,20,30) |> (translate(p(0,0,0), p(10,0,0)) + cube(100,1,1))
        // which is apply(translate(p(0,0,0), p(10,0,0)) + cube(100,1,1), cube(10,20,30))
        // But since translate + cube is ArithExpr not Struct, it wraps with "apply"
        let src = "cube(10,20,30) |> translate(p(0,0,0), p(10,0,0)) + cube(100,1,1).";
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

    #[test]
    fn parse_annotated_var_with_default_and_range_in_body() {
        let src = "hoge(X@20) :- 0<X<50, cube(X).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Rule { head, body } => {
                match &head {
                    Term::Struct { args, .. } => match &args[0] {
                        Term::Var {
                            name,
                            default_value,
                            ..
                        } => {
                            assert_eq!(name, "X");
                            assert_eq!(*default_value, Some(crate::rational::Rational::from_integer(20)));
                        }
                        _ => panic!("Expected Var"),
                    },
                    _ => panic!("Expected Struct"),
                }
                match &body[0] {
                    Term::Var {
                        name, min, max, ..
                    } => {
                        assert_eq!(name, "X");
                        assert_eq!(
                            *min,
                            Some(Bound {
                                value: crate::rational::Rational::from_integer(0),
                                inclusive: false
                            })
                        );
                        assert_eq!(
                            *max,
                            Some(Bound {
                                value: crate::rational::Rational::from_integer(50),
                                inclusive: false
                            })
                        );
                    }
                    _ => panic!("Expected Var"),
                }
            }
            _ => panic!("Expected Rule"),
        }
    }

    #[test]
    fn parse_annotated_var_inclusive_range_in_body() {
        let src = "hoge(X@20) :- 0<=X<=50, cube(X).";
        let (_, clause) = clause_parser(src).unwrap();

        match clause {
            Clause::Rule { body, .. } => match &body[0] {
                Term::Var {
                    name,
                    min,
                    max,
                    ..
                } => {
                    assert_eq!(name, "X");
                    assert!(min.as_ref().unwrap().inclusive);
                    assert!(max.as_ref().unwrap().inclusive);
                }
                _ => panic!("Expected Var"),
            },
            _ => panic!("Expected Rule"),
        }
    }

    #[test]
    fn test_collect_query_params() {
        let (_, terms) = query("main(X, 0<Y<50).").unwrap();
        let params = collect_query_params(&terms);
        assert_eq!(params.len(), 2);
        assert_eq!(params[0].name, "X");
        assert!(params[0].min.is_none());
        assert!(params[0].max.is_none());
        assert_eq!(params[1].name, "Y");
        assert!(params[1].min.is_some());
        assert!(params[1].max.is_some());
    }

    #[test]
    fn test_substitute_query_params() {
        let (_, terms) = query("main(X, Y).").unwrap();
        let mut values = std::collections::HashMap::new();
        values.insert("X".to_string(), 5.0);
        values.insert("Y".to_string(), 10.0);
        let substituted = substitute_query_params(&terms, &values);
        assert_eq!(format!("{:?}", substituted), "[main(X@5, Y@10)]");
    }

    #[test]
    fn parse_use_simple() {
        let src = r#"#use("bolts")."#;
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Use { path, expose, .. } => {
                assert_eq!(path, "bolts");
                assert!(expose.is_empty());
            }
            _ => panic!("Expected Use, got {:?}", clause),
        }
    }

    #[test]
    fn parse_use_with_expose() {
        let src = r#"#use("bolts", expose([m5, m6]))."#;
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Use { path, expose, .. } => {
                assert_eq!(path, "bolts");
                assert_eq!(expose, vec!["m5", "m6"]);
            }
            _ => panic!("Expected Use, got {:?}", clause),
        }
    }

    #[test]
    fn parse_use_with_subpath() {
        let src = r#"#use("sub/parts")."#;
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Use { path, .. } => assert_eq!(path, "sub/parts"),
            _ => panic!("Expected Use"),
        }
    }

    #[test]
    fn parse_qualified_functor() {
        let src = "bolts::m5(X).";
        let (_, clause) = clause_parser(src).unwrap();
        match clause {
            Clause::Fact(Term::Struct { functor, args, .. }) => {
                assert_eq!(functor, "bolts::m5");
                assert_eq!(args.len(), 1);
            }
            _ => panic!("Expected qualified Struct, got {:?}", clause),
        }
    }

    #[test]
    fn parse_database_with_use() {
        let src = r#"
            #use("bolts").
            main(X) :- bolts::m5(X).
        "#;
        let db = database(src).unwrap();
        assert_eq!(db.len(), 2);
        assert!(matches!(&db[0], Clause::Use { path, .. } if path == "bolts"));
    }
}
