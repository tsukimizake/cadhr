//! Elm-like cadhr-lang の字句解析。
//!
//! 入力ソース文字列 `&str` を `Vec<(Token, Span)>` に変換する。
//! parser はこのトークン列を受け取って AST を組み立てる。
//!
//! 字句構造:
//! - 小文字始まりの識別子 `LowerIdent` (関数 / 変数 / フィールド名)
//! - 大文字始まりの識別子 `UpperIdent` (型 / コンストラクタ / モジュール)
//! - 数値リテラル `Int` / `Float`
//! - 文字列リテラル `Str`
//! - 主要キーワード (`module`, `exposing`, `import`, `as`, `type`, `alias`,
//!   `let`, `in`, `if`, `then`, `else`, `case`, `of`, `True`, `False`,
//!   `slider`)
//! - 演算子 / 区切り記号
//!
//! コメントは `--` 行コメント、`{- ... -}` ブロックコメント (ネスト対応)。
//! 改行は `Token::Newline` として残し、後段の layout pass (`layout::apply_layout`) が
//! インデント (オフサイドルール) に基づいて BlockOpen/BlockSep/BlockClose に変換する。

use crate::diagnostic::Span;
use chumsky::prelude::*;

#[derive(Clone, Debug, PartialEq)]
pub enum Token<'src> {
    // 識別子
    LowerIdent(&'src str),
    UpperIdent(&'src str),

    // リテラル
    Int(i64),
    Float(f64),
    Str(String),

    // キーワード
    Module,
    Exposing,
    Import,
    As,
    Type,
    Alias,
    Let,
    In,
    If,
    Then,
    Else,
    Case,
    Of,
    True,
    False,
    Slider,

    // 演算子と記号
    Eq,       // =
    Arrow,    // ->
    Colon,    // :
    DotDot,   // ..
    Dot,      // .
    Pipe,     // |
    PipeFwd,  // |>
    PipeBwd,  // <|
    Compose,  // >>
    ComposeR, // <<
    Cons,     // ::
    EqEq,     // ==
    NotEq,    // /=
    Le,       // <=
    Ge,       // >=
    Lt,       // <
    Gt,       // >
    PlusPlus, // ++
    Plus,
    Minus,
    Star,
    Slash,
    AndAnd,    // &&
    OrOr,      // ||
    BackSlash, // \

    // 区切り
    LParen,
    RParen,
    LBrace,
    RBrace,
    LBracket,
    RBracket,
    Comma,
    Underscore,

    /// 改行 (1 個以上連続する `\n` を 1 つの Newline トークンに集約)。
    /// lexer が出力し、後段の layout pass (`layout::apply_layout`) が消費する。
    /// parser には届かない (layout pass で BlockOpen/BlockSep/BlockClose に変換される)。
    Newline,

    /// layout pass が挿入する仮想トークン。let / case-of ブロックの開始。
    BlockOpen,
    /// layout pass が挿入する仮想トークン。同一レイアウト列に揃った兄弟要素の区切り
    /// (top-level decl / let binding / case arm の間)。
    BlockSep,
    /// layout pass が挿入する仮想トークン。let / case-of ブロックの終了。
    BlockClose,
}

impl<'src> std::fmt::Display for Token<'src> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Token::LowerIdent(s) | Token::UpperIdent(s) => write!(f, "{s}"),
            Token::Int(n) => write!(f, "{n}"),
            Token::Float(n) => write!(f, "{n}"),
            Token::Str(s) => write!(f, "\"{s}\""),
            Token::Module => f.write_str("module"),
            Token::Exposing => f.write_str("exposing"),
            Token::Import => f.write_str("import"),
            Token::As => f.write_str("as"),
            Token::Type => f.write_str("type"),
            Token::Alias => f.write_str("alias"),
            Token::Let => f.write_str("let"),
            Token::In => f.write_str("in"),
            Token::If => f.write_str("if"),
            Token::Then => f.write_str("then"),
            Token::Else => f.write_str("else"),
            Token::Case => f.write_str("case"),
            Token::Of => f.write_str("of"),
            Token::True => f.write_str("True"),
            Token::False => f.write_str("False"),
            Token::Slider => f.write_str("slider"),
            Token::Eq => f.write_str("="),
            Token::Arrow => f.write_str("->"),
            Token::Colon => f.write_str(":"),
            Token::DotDot => f.write_str(".."),
            Token::Dot => f.write_str("."),
            Token::Pipe => f.write_str("|"),
            Token::PipeFwd => f.write_str("|>"),
            Token::PipeBwd => f.write_str("<|"),
            Token::Compose => f.write_str(">>"),
            Token::ComposeR => f.write_str("<<"),
            Token::Cons => f.write_str("::"),
            Token::EqEq => f.write_str("=="),
            Token::NotEq => f.write_str("/="),
            Token::Le => f.write_str("<="),
            Token::Ge => f.write_str(">="),
            Token::Lt => f.write_str("<"),
            Token::Gt => f.write_str(">"),
            Token::PlusPlus => f.write_str("++"),
            Token::Plus => f.write_str("+"),
            Token::Minus => f.write_str("-"),
            Token::Star => f.write_str("*"),
            Token::Slash => f.write_str("/"),
            Token::AndAnd => f.write_str("&&"),
            Token::OrOr => f.write_str("||"),
            Token::BackSlash => f.write_str("\\"),
            Token::LParen => f.write_str("("),
            Token::RParen => f.write_str(")"),
            Token::LBrace => f.write_str("{"),
            Token::RBrace => f.write_str("}"),
            Token::LBracket => f.write_str("["),
            Token::RBracket => f.write_str("]"),
            Token::Comma => f.write_str(","),
            Token::Underscore => f.write_str("_"),
            Token::Newline => f.write_str("<newline>"),
            Token::BlockOpen => f.write_str("<block-open>"),
            Token::BlockSep => f.write_str("<block-sep>"),
            Token::BlockClose => f.write_str("<block-close>"),
        }
    }
}

/// chumsky の `Spanned` struct を再 export。`split_spanned` などの I/F と整合させるため。
pub use chumsky::span::Spanned;

type LexError<'src> = extra::Err<Rich<'src, char>>;

/// 入力ソース全体を `Vec<Spanned<Token, SimpleSpan>>` にレックスする parser。
pub fn lexer<'src>() -> impl Parser<'src, &'src str, Vec<Spanned<Token<'src>>>, LexError<'src>> {
    // -- ... 改行 までの行コメント
    let line_comment = just("--")
        .then(any().and_is(just('\n').not()).repeated())
        .ignored();

    // {- ... -} ネスト対応のブロックコメント
    let block_comment = recursive(|block_comment| {
        let inner = choice((
            block_comment,
            any()
                .and_is(just("-}").not())
                .and_is(just("{-").not())
                .ignored(),
        ));
        just("{-").then(inner.repeated()).then(just("-}")).ignored()
    });

    let comment = choice((line_comment, block_comment));

    // 改行以外の whitespace のみを ws として扱う。改行は別 token として残す。
    let ws = choice((
        any()
            .filter(|c: &char| c.is_whitespace() && *c != '\n')
            .ignored(),
        comment,
    ))
    .repeated()
    .ignored();

    // 識別子: 先頭文字で大文字 / 小文字を区別。
    let raw_ident = any()
        .filter(|c: &char| c.is_ascii_alphabetic() || *c == '_')
        .then(
            any()
                .filter(|c: &char| c.is_ascii_alphanumeric() || *c == '_')
                .repeated(),
        )
        .to_slice();

    let ident_or_keyword = raw_ident.map(|s: &str| match s {
        "module" => Token::Module,
        "exposing" => Token::Exposing,
        "import" => Token::Import,
        "as" => Token::As,
        "type" => Token::Type,
        "alias" => Token::Alias,
        "let" => Token::Let,
        "in" => Token::In,
        "if" => Token::If,
        "then" => Token::Then,
        "else" => Token::Else,
        "case" => Token::Case,
        "of" => Token::Of,
        "True" => Token::True,
        "False" => Token::False,
        "slider" => Token::Slider,
        "_" => Token::Underscore,
        s if s.starts_with(|c: char| c.is_ascii_uppercase()) => Token::UpperIdent(s),
        s => Token::LowerIdent(s),
    });

    // 数値: 整数 or 浮動小数点。負号は単項演算子として parser 側で処理する。
    let number = text::int(10)
        .then(just('.').then(text::digits(10)).or_not())
        .to_slice()
        .map(|s: &str| {
            if s.contains('.') {
                Token::Float(s.parse().unwrap())
            } else {
                Token::Int(s.parse().unwrap())
            }
        });

    // 文字列: シンプルなエスケープ対応 (\n, \t, \\, \")
    let escape = just('\\').ignore_then(choice((
        just('n').to('\n'),
        just('t').to('\t'),
        just('"').to('"'),
        just('\\').to('\\'),
    )));
    let string = just('"')
        .ignore_then(
            choice((escape, any().and_is(just('"').not())))
                .repeated()
                .collect::<String>(),
        )
        .then_ignore(just('"'))
        .map(Token::Str);

    // 演算子は chumsky の `choice` がタプル展開なので 1 グループあたり要素数に
    // 上限がある。長い順に並べた上で 3 グループに分割する。
    let op_multi = choice((
        just("->").to(Token::Arrow),
        just("==").to(Token::EqEq),
        just("/=").to(Token::NotEq),
        just("<=").to(Token::Le),
        just(">=").to(Token::Ge),
        just("|>").to(Token::PipeFwd),
        just("<|").to(Token::PipeBwd),
        just(">>").to(Token::Compose),
        just("<<").to(Token::ComposeR),
        just("::").to(Token::Cons),
        just("..").to(Token::DotDot),
        just("++").to(Token::PlusPlus),
        just("&&").to(Token::AndAnd),
        just("||").to(Token::OrOr),
    ));
    let op_single = choice((
        just("=").to(Token::Eq),
        just(":").to(Token::Colon),
        just("|").to(Token::Pipe),
        just(".").to(Token::Dot),
        just("<").to(Token::Lt),
        just(">").to(Token::Gt),
        just("+").to(Token::Plus),
        just("-").to(Token::Minus),
        just("*").to(Token::Star),
        just("/").to(Token::Slash),
        just("\\").to(Token::BackSlash),
    ));
    let punct = choice((
        just("(").to(Token::LParen),
        just(")").to(Token::RParen),
        just("{").to(Token::LBrace),
        just("}").to(Token::RBrace),
        just("[").to(Token::LBracket),
        just("]").to(Token::RBracket),
        just(",").to(Token::Comma),
    ));
    let op = choice((op_multi, op_single, punct));

    let single = choice((ident_or_keyword, number, string, op));

    // 改行 `\n+` を 1 つの Newline トークンに集約。
    let newline_tok = just('\n')
        .then(
            just('\n')
                .or(any().filter(|c: &char| c.is_whitespace() && *c != '\n'))
                .repeated(),
        )
        .ignored()
        .map(|_| Token::Newline);

    let any_tok = choice((newline_tok, single));

    any_tok
        .map_with(|tok, e| Spanned {
            inner: tok,
            span: e.span(),
        })
        .padded_by(ws)
        .repeated()
        .collect::<Vec<_>>()
}

/// 中間表現の `Spanned<Token, SimpleSpan>` をそのまま返す (parser 用)。
pub fn lex_raw(src: &str) -> Result<Vec<Spanned<Token<'_>>>, Vec<Rich<'_, char>>> {
    lexer().parse(src).into_result()
}

/// 外部 API: 公開 `Span` に変換して返す。
pub fn lex(src: &str) -> Result<Vec<(Token<'_>, Span)>, Vec<Rich<'_, char>>> {
    lex_raw(src).map(|tokens| {
        tokens
            .into_iter()
            .map(|s| (s.inner, Span::from(s.span)))
            .collect()
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lex_strip(src: &str) -> Vec<Token<'_>> {
        lex(src)
            .expect("lex failed")
            .into_iter()
            .map(|(t, _)| t)
            .collect()
    }

    #[test]
    fn idents_and_keywords() {
        assert_eq!(
            lex_strip("module Foo exposing"),
            vec![Token::Module, Token::UpperIdent("Foo"), Token::Exposing]
        );
        assert_eq!(
            lex_strip("let x = 1 in x"),
            vec![
                Token::Let,
                Token::LowerIdent("x"),
                Token::Eq,
                Token::Int(1),
                Token::In,
                Token::LowerIdent("x"),
            ]
        );
    }

    #[test]
    fn operators() {
        assert_eq!(
            lex_strip("|> <| >> <<"),
            vec![
                Token::PipeFwd,
                Token::PipeBwd,
                Token::Compose,
                Token::ComposeR,
            ]
        );
        assert_eq!(
            lex_strip("1..10"),
            vec![Token::Int(1), Token::DotDot, Token::Int(10)]
        );
    }

    #[test]
    fn numbers_and_strings() {
        assert_eq!(
            lex_strip("3.14 42"),
            vec![Token::Float(3.14), Token::Int(42)]
        );
        assert_eq!(
            lex_strip(r#""hello\n""#),
            vec![Token::Str("hello\n".to_string())]
        );
    }

    #[test]
    fn comments_are_skipped() {
        // 行コメントは `\n` を残すので Newline が混じる
        assert_eq!(
            lex_strip("x -- a comment\n y"),
            vec![
                Token::LowerIdent("x"),
                Token::Newline,
                Token::LowerIdent("y")
            ]
        );
        // ブロックコメントは内部の `\n` を吸収する (同一行扱い)
        assert_eq!(
            lex_strip("x {- nested {- inner -} still -} y"),
            vec![Token::LowerIdent("x"), Token::LowerIdent("y")]
        );
    }

    #[test]
    fn newlines_are_tokens() {
        // `\n+` は 1 つの Newline に集約
        assert_eq!(
            lex_strip("a\nb\n\n\nc"),
            vec![
                Token::LowerIdent("a"),
                Token::Newline,
                Token::LowerIdent("b"),
                Token::Newline,
                Token::LowerIdent("c"),
            ]
        );
    }

    #[test]
    fn underscore_pattern() {
        assert_eq!(
            lex_strip("_ x"),
            vec![Token::Underscore, Token::LowerIdent("x")]
        );
    }
}
