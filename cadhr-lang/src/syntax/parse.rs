//! Elm-like cadhr-lang の構文解析。
//!
//! 入力は `lex.rs` が出力した `Vec<(Token, SimpleSpan)>` のスライス。
//! 出力は AST の `Module`。式の優先順位は chumsky の `pratt` で記述する。
//!
//! 設計:
//! - 関数適用 (`f x y`) は curried で App ノードを左結合に積む
//! - パイプ `|>` は `expr |> f x` → `App(f x, expr)` ではなく `BinOp::ApplyR` で
//!   AST 上は左辺 / 右辺の双方を保持する (pretty 表記の readability のため)
//! - 二項演算子はすべて Pratt で扱う (リスト `::`, `++`、関数合成 `<<` `>>` 等)
//! - 単項マイナス `-x` は atom レベルで処理 (Negate)
//! - Range リテラル `a..b` は二項演算ではなく専用 `Expr::Range` ノードに
//! - 識別子の `Std.Gears` は qualified module name 解析時にドットを連結
//!
//! エラー型は `Rich<'tokens, Token<'src>>`。tokens slice の寿命は 'tokens、
//! token 内の `&str` は元ソースの 'src を借りる。

use crate::diagnostic::Span as DSpan;
use crate::syntax::ast::*;
use crate::syntax::lex::{Spanned, Token};
use chumsky::input::{Input as _, MappedInput};
use chumsky::pratt::*;
use chumsky::prelude::*;

/// parser に渡す入力スライス型。`mini_ml.rs` と同じ構造。
pub type ParserInput<'tokens, 'src> =
    MappedInput<'tokens, Token<'src>, SimpleSpan, &'tokens [Spanned<Token<'src>, SimpleSpan>]>;

pub type ParserError<'tokens, 'src> = extra::Err<Rich<'tokens, Token<'src>>>;

fn dspan(s: SimpleSpan) -> DSpan {
    DSpan::from(s)
}

/// top-level 要素間の区切り `BlockSep` (0 個以上) を skip する parser。
/// layout pass が decl 境界に挿入する。
fn top_sep<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, (), ParserError<'tokens, 'src>> + Clone {
    just(Token::BlockSep).repeated().ignored()
}

/// 小文字始まりの識別子を取り出す。
fn lower_ident<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, String, ParserError<'tokens, 'src>> + Clone {
    select! { Token::LowerIdent(s) => s.to_string() }
}

/// 大文字始まりの識別子を取り出す (型 / コンストラクタ / モジュール名)。
fn upper_ident<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, String, ParserError<'tokens, 'src>> + Clone {
    select! { Token::UpperIdent(s) => s.to_string() }
}

/// `Foo.Bar.Baz` のような qualified モジュール名。空タプル `()` でドット無しの単純名も
/// 同じ AST で扱う。
fn module_name<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, ModuleName, ParserError<'tokens, 'src>> + Clone
{
    upper_ident()
        .separated_by(just(Token::Dot))
        .at_least(1)
        .collect::<Vec<_>>()
        .map_with(|segments, e| ModuleName {
            segments,
            span: dspan(e.span()),
        })
}

/// Pattern parser (再帰)。`expr_parser` から呼ばれる場合に備えて recursive 内で
/// 定義する。
fn pattern_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, Pattern, ParserError<'tokens, 'src>> + Clone {
    recursive(|pat| {
        let var = lower_ident().map_with(|name, e| Pattern::Var(name, dspan(e.span())));
        let wildcard = just(Token::Underscore).map_with(|_, e| Pattern::Wildcard(dspan(e.span())));
        let lit = lit_atom().map_with(|l, e| Pattern::Lit(l, dspan(e.span())));

        // `Cube x y z` のような ADT コンストラクタパターン
        let ctor = upper_ident()
            .then(pat.clone().repeated().collect::<Vec<_>>())
            .map_with(|(name, args), e| Pattern::Ctor {
                module: None,
                name,
                args,
                span: dspan(e.span()),
            });

        // `[a, b, c]` の list パターン (`a :: rest` は :: 演算子側で処理)
        let list = pat
            .clone()
            .separated_by(just(Token::Comma))
            .allow_trailing()
            .collect::<Vec<_>>()
            .delimited_by(just(Token::LBracket), just(Token::RBracket))
            .map_with(|items, e| Pattern::List(items, dspan(e.span())));

        // `{ a, b }` の record パターン
        let record = lower_ident()
            .separated_by(just(Token::Comma))
            .allow_trailing()
            .collect::<Vec<_>>()
            .delimited_by(just(Token::LBrace), just(Token::RBrace))
            .map_with(|fields, e| Pattern::Record(fields, dspan(e.span())));

        // `( p )` のグループ化、または ctor / atomic patterns
        let atom = choice((
            wildcard,
            var.clone(),
            lit,
            ctor.clone(),
            list,
            record,
            pat.clone()
                .delimited_by(just(Token::LParen), just(Token::RParen)),
        ));

        // `a :: rest` を右結合で扱う
        let cons = atom.clone().foldl_with(
            just(Token::Cons).ignore_then(atom.clone()).repeated(),
            |head, tail, e| Pattern::Cons {
                head: Box::new(head),
                tail: Box::new(tail),
                span: dspan(e.span()),
            },
        );

        // `inner as name` の alias パターン
        cons.then(just(Token::As).ignore_then(lower_ident()).or_not())
            .map_with(|(inner, alias), e| match alias {
                Some(name) => Pattern::As {
                    inner: Box::new(inner),
                    name,
                    span: dspan(e.span()),
                },
                None => inner,
            })
    })
}

/// 数値・文字列・真偽値のリテラル。
fn lit_atom<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, Lit, ParserError<'tokens, 'src>> + Clone {
    select! {
        Token::Int(n) => Lit::Int(n),
        Token::Float(f) => Lit::Float(f),
        Token::Str(ref s) => Lit::String(s.clone()),
        Token::True => Lit::Bool(true),
        Token::False => Lit::Bool(false),
    }
}

/// type expr の **atomic** 部分 (constructor 引数として直接書ける) のみを取り出す parser。
/// constructor application や arrow を含まない。`type_decl` の constructor 引数で使う。
fn type_atom_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, TypeExpr, ParserError<'tokens, 'src>> + Clone {
    let var = lower_ident().map_with(|name, e| TypeExpr::Var(name, dspan(e.span())));
    let con_atom = module_name().map_with(|mname, e| {
        let span = dspan(e.span());
        let (module, name) = split_qualified_upper(mname);
        TypeExpr::Con {
            module,
            name,
            args: vec![],
            span,
        }
    });
    let record_field = lower_ident()
        .then_ignore(just(Token::Colon))
        .then(type_expr_parser())
        .map_with(|(name, ty), e| RecordTypeField {
            name,
            ty,
            span: dspan(e.span()),
        });
    let record = record_field
        .separated_by(just(Token::Comma))
        .allow_trailing()
        .collect::<Vec<_>>()
        .delimited_by(just(Token::LBrace), just(Token::RBrace))
        .map_with(|fields, e| TypeExpr::Record(fields, dspan(e.span())));
    let paren = type_expr_parser().delimited_by(just(Token::LParen), just(Token::RParen));
    choice((var, record, con_atom, paren))
}

/// `Foo.Bar.Baz` のような qualified upper-cased 名前を、最後を name にして分割する
/// ヘルパ。constructor 名 / 型名で使う。
fn split_qualified_upper(mname: ModuleName) -> (Option<ModuleName>, String) {
    if mname.segments.len() == 1 {
        (None, mname.segments.into_iter().next().unwrap())
    } else {
        let mut segs = mname.segments;
        let last = segs.pop().unwrap();
        (
            Some(ModuleName {
                segments: segs,
                span: mname.span,
            }),
            last,
        )
    }
}

/// TypeExpr parser (再帰)。
/// 文法構造:
/// - atom: `Var`, `Con` (引数なし), `Record`, `( ty )`
/// - app : `Con atom+` (引数 1 個以上の constructor application) ∪ atom
/// - ty  : `app -> app -> ...` (右結合)
///
/// `Con` の引数を `atom` に限定しないと、`f : Int -> Int\nf x = ...` のような書き方で
/// 「Int の右に続く `f x` を Int の引数として食う」ような誤解析が起きる。
fn type_expr_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, TypeExpr, ParserError<'tokens, 'src>> + Clone {
    recursive(|ty| {
        let var = lower_ident().map_with(|name, e| TypeExpr::Var(name, dspan(e.span())));

        // 引数を取らない裸の Con 名 (atom 文脈)
        let con_atom = module_name().map_with(|mname, e| {
            let span = dspan(e.span());
            let (module, name) = split_qualified_upper(mname);
            TypeExpr::Con {
                module,
                name,
                args: vec![],
                span,
            }
        });

        // `{ field : Type, ... }` の record 型。括弧内は layout 停止なので改行は透過し、
        // カンマ区切りのみで扱う。
        let record_field = lower_ident()
            .then_ignore(just(Token::Colon))
            .then(ty.clone())
            .map_with(|(name, ty), e| RecordTypeField {
                name,
                ty,
                span: dspan(e.span()),
            });
        let record = record_field
            .separated_by(just(Token::Comma))
            .allow_trailing()
            .collect::<Vec<_>>()
            .delimited_by(just(Token::LBrace), just(Token::RBrace))
            .map_with(|fields, e| TypeExpr::Record(fields, dspan(e.span())));

        let atom = choice((
            var,
            record,
            con_atom,
            ty.clone()
                .delimited_by(just(Token::LParen), just(Token::RParen)),
        ));

        // 引数を 1 つ以上取る constructor application (`List Int`, `Dict Key Value` ...)
        let app_con = module_name()
            .then(atom.clone().repeated().at_least(1).collect::<Vec<_>>())
            .map_with(|(mname, args), e| {
                let span = dspan(e.span());
                let (module, name) = split_qualified_upper(mname);
                TypeExpr::Con {
                    module,
                    name,
                    args,
                    span,
                }
            });

        let app = app_con.or(atom);

        // `a -> b -> c` は右結合。`ty` 自体の再帰を使って右側を取る。
        app.clone()
            .then(just(Token::Arrow).ignore_then(ty.clone()).or_not())
            .map_with(|(from, to), e| match to {
                Some(t) => TypeExpr::Arrow {
                    from: Box::new(from),
                    to: Box::new(t),
                    span: dspan(e.span()),
                },
                None => from,
            })
    })
}

/// Expression parser (再帰)。
pub fn expr_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, Expr, ParserError<'tokens, 'src>> + Clone {
    recursive(|expr| {
        let pat = pattern_parser();

        let var = lower_ident().map_with(|name, e| Expr::Var {
            module: None,
            name,
            span: dspan(e.span()),
        });

        // `Foo.bar` のような qualified 変数。`Foo.Bar.baz` も同様。
        // 大文字始まり segment が並んだ後に最後にドット + 小文字始まりの場合のみ
        // qualified var として解釈する。
        let qualified_var = upper_ident()
            .then(
                just(Token::Dot)
                    .ignore_then(upper_ident())
                    .repeated()
                    .collect::<Vec<_>>(),
            )
            .then(just(Token::Dot).ignore_then(lower_ident()))
            .map_with(|((first, rest), last), e| {
                let mut segments = vec![first];
                segments.extend(rest);
                let mspan = dspan(e.span());
                Expr::Var {
                    module: Some(ModuleName {
                        segments,
                        span: mspan,
                    }),
                    name: last,
                    span: mspan,
                }
            });

        // Constructor は大文字始まり (qualified 可能)。
        let ctor = upper_ident()
            .then(
                just(Token::Dot)
                    .ignore_then(upper_ident())
                    .repeated()
                    .collect::<Vec<_>>(),
            )
            .map_with(|(first, rest), e| {
                let span = dspan(e.span());
                if rest.is_empty() {
                    Expr::Ctor {
                        module: None,
                        name: first,
                        span,
                    }
                } else {
                    let mut segments = vec![first];
                    let mut rest = rest;
                    let name = rest.pop().unwrap();
                    segments.extend(rest);
                    Expr::Ctor {
                        module: Some(ModuleName { segments, span }),
                        name,
                        span,
                    }
                }
            });

        let lit = lit_atom().map_with(|l, e| Expr::Lit(l, dspan(e.span())));

        let list = expr
            .clone()
            .separated_by(just(Token::Comma))
            .allow_trailing()
            .collect::<Vec<_>>()
            .delimited_by(just(Token::LBracket), just(Token::RBracket))
            .map_with(|items, e| Expr::List(items, dspan(e.span())));

        // `{ field = expr, ... }` の record literal。
        // record update `{ r | field = expr }` も同じ `{}` 内に `name |` プレフィクス
        // が来る形で扱う。
        let record_field = lower_ident()
            .then_ignore(just(Token::Eq))
            .then(expr.clone())
            .map_with(|(name, value), e| RecordField {
                name,
                value,
                span: dspan(e.span()),
            });

        let record_or_update = just(Token::LBrace)
            .ignore_then(lower_ident().then_ignore(just(Token::Pipe)).or_not())
            .then(
                record_field
                    .separated_by(just(Token::Comma))
                    .allow_trailing()
                    .collect::<Vec<_>>(),
            )
            .then_ignore(just(Token::RBrace))
            .map_with(|(base_name, fields), e| {
                let span = dspan(e.span());
                match base_name {
                    Some(name) => Expr::RecordUpdate {
                        base: Box::new(Expr::Var {
                            module: None,
                            name,
                            span,
                        }),
                        updates: fields,
                        span,
                    },
                    None => Expr::Record(fields, span),
                }
            });

        // `let <bindings> in body`。binding 列は layout pass が BlockOpen/BlockSep/BlockClose で
        // 区切る (Elm 流のインデント整列)。
        let value_binding = lower_ident()
            .then(pat.clone().repeated().collect::<Vec<_>>())
            .then_ignore(just(Token::Eq))
            .then(expr.clone())
            .map_with(|((name, params), body), e| ValueDecl {
                name,
                params,
                body,
                span: dspan(e.span()),
            });
        let let_expr = just(Token::Let)
            .ignore_then(just(Token::BlockOpen))
            .ignore_then(
                value_binding
                    .clone()
                    .separated_by(just(Token::BlockSep))
                    .allow_trailing()
                    .at_least(1)
                    .collect::<Vec<_>>(),
            )
            .then_ignore(just(Token::BlockClose))
            .then_ignore(just(Token::In))
            .then(expr.clone())
            .map_with(|(bindings, body), e| Expr::Let {
                bindings,
                body: Box::new(body),
                span: dspan(e.span()),
            });

        // `if cond then a else b`
        // 各分岐を次行に書く場合、layout pass が継続行 (深い字下げ) として透過する。
        let if_expr = just(Token::If)
            .ignore_then(expr.clone())
            .then_ignore(just(Token::Then))
            .then(expr.clone())
            .then_ignore(just(Token::Else))
            .then(expr.clone())
            .map_with(|((cond, then_b), else_b), e| Expr::If {
                cond: Box::new(cond),
                then_branch: Box::new(then_b),
                else_branch: Box::new(else_b),
                span: dspan(e.span()),
            });

        // `case scrutinee of` の後、arm をインデント整列で並べる (Elm 流, 先頭 `|` 無し)。
        // arm 列は layout pass が BlockOpen/BlockSep/BlockClose で区切る。
        let case_arm = pat
            .clone()
            .then_ignore(just(Token::Arrow))
            .then(expr.clone())
            .map_with(|(pattern, body), e| CaseArm {
                pattern,
                guard: None,
                body,
                span: dspan(e.span()),
            });
        let case_expr = just(Token::Case)
            .ignore_then(expr.clone())
            .then_ignore(just(Token::Of))
            .then_ignore(just(Token::BlockOpen))
            .then(
                case_arm
                    .clone()
                    .separated_by(just(Token::BlockSep))
                    .allow_trailing()
                    .at_least(1)
                    .collect::<Vec<_>>(),
            )
            .then_ignore(just(Token::BlockClose))
            .map_with(|(scrutinee, arms), e| Expr::Case {
                scrutinee: Box::new(scrutinee),
                arms,
                span: dspan(e.span()),
            });

        // `\x y -> body` (`->` の後で改行した場合 layout pass が継続行として透過する)
        let lambda = just(Token::BackSlash)
            .ignore_then(pat.clone().repeated().at_least(1).collect::<Vec<_>>())
            .then_ignore(just(Token::Arrow))
            .then(expr.clone())
            .map_with(|(params, body), e| Expr::Lambda {
                params,
                body: Box::new(body),
                span: dspan(e.span()),
            });

        // パース可能な単位 (atom)。括弧でグループ化された expr も含む。
        // 注意: 単項マイナスは atom に入れずに app の前 (prefix) でのみ拾う。
        // そうしないと `0.0 - gear_z g1` のような式で `-` が新 atom の prefix と
        // 認識されてしまい `0.0 (Negate(gear_z g1))` のように誤解析される。
        let atom = choice((
            let_expr,
            if_expr,
            case_expr,
            lambda,
            qualified_var,
            ctor,
            var,
            lit,
            list,
            record_or_update,
            expr.clone()
                .delimited_by(just(Token::LParen), just(Token::RParen)),
        ));

        // `expr.field` の field access (左結合)
        let field_access = atom.clone().foldl_with(
            just(Token::Dot).ignore_then(lower_ident()).repeated(),
            |receiver, name, e| Expr::Field {
                receiver: Box::new(receiver),
                name,
                span: dspan(e.span()),
            },
        );

        // 関数適用 (curried, 左結合)。`f x y z` → `App(App(App(f, x), y), z)`
        let app_inner =
            field_access
                .clone()
                .foldl_with(field_access.clone().repeated(), |func, arg, e| Expr::App {
                    func: Box::new(func),
                    arg: Box::new(arg),
                    span: dspan(e.span()),
                });

        // 単項マイナス: app 全体の prefix としてのみ許す。`-x` / `-(x + y)` など。
        let negate = just(Token::Minus)
            .ignore_then(app_inner.clone())
            .map_with(|inner, e| Expr::Negate(Box::new(inner), dspan(e.span())));
        let app = negate.or(app_inner);

        // 二項演算子を Pratt で組む。優先順位は Elm に概ね合わせる。
        // 低い順: || (2), && (3), 比較 (4), :: ++ (5), + - (6), * / (7),
        //         関数合成 << >> (9), パイプ <| |> (0/負)
        // Range `a..b` は最も低い優先度の演算子として扱う。
        // 複数行 pipe チェーン (`cube ..\n  |> rotate3d ..`) は、継続行が深い字下げのとき
        // layout pass が改行を透過する (区切りを挿入しない) ことで成立する。
        app.pratt((
            infix(left(0), just(Token::PipeFwd), |l, _, r, e| Expr::BinOp {
                op: BinOp::ApplyR,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(right(0), just(Token::PipeBwd), |l, _, r, e| Expr::BinOp {
                op: BinOp::ApplyL,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(2), just(Token::OrOr), |l, _, r, e| Expr::BinOp {
                op: BinOp::Or,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(3), just(Token::AndAnd), |l, _, r, e| Expr::BinOp {
                op: BinOp::And,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(4), just(Token::EqEq), |l, _, r, e| Expr::BinOp {
                op: BinOp::Eq,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(4), just(Token::NotEq), |l, _, r, e| Expr::BinOp {
                op: BinOp::NotEq,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(4), just(Token::Lt), |l, _, r, e| Expr::BinOp {
                op: BinOp::Lt,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(4), just(Token::Le), |l, _, r, e| Expr::BinOp {
                op: BinOp::Le,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(4), just(Token::Gt), |l, _, r, e| Expr::BinOp {
                op: BinOp::Gt,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(4), just(Token::Ge), |l, _, r, e| Expr::BinOp {
                op: BinOp::Ge,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(right(5), just(Token::Cons), |l, _, r, e| Expr::BinOp {
                op: BinOp::Cons,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(right(5), just(Token::PlusPlus), |l, _, r, e| Expr::BinOp {
                op: BinOp::Append,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(6), just(Token::Plus), |l, _, r, e| Expr::BinOp {
                op: BinOp::Add,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(6), just(Token::Minus), |l, _, r, e| Expr::BinOp {
                op: BinOp::Sub,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(7), just(Token::Star), |l, _, r, e| Expr::BinOp {
                op: BinOp::Mul,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(left(7), just(Token::Slash), |l, _, r, e| Expr::BinOp {
                op: BinOp::Div,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(right(9), just(Token::Compose), |l, _, r, e| Expr::BinOp {
                op: BinOp::Compose,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            infix(right(9), just(Token::ComposeR), |l, _, r, e| Expr::BinOp {
                op: BinOp::ComposeR,
                left: Box::new(l),
                right: Box::new(r),
                span: dspan(e.span()),
            }),
            // Range `lo..hi` は最も低い優先度。
            infix(left(1), just(Token::DotDot), |l, _, r, e| Expr::Range {
                lo: Box::new(l),
                hi: Box::new(r),
                span: dspan(e.span()),
            }),
        ))
    })
}

/// 1 つのトップレベル宣言。Signature と Value は同じ識別子で 2 行に分かれて書かれる
/// (Elm 流: `f : Int -> Int` と `f x = x + 1` を別 decl として保持し、resolve / typecheck
/// 時に紐付ける)。
pub fn decl_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, Decl, ParserError<'tokens, 'src>> + Clone {
    let pat = pattern_parser();
    let ty = type_expr_parser();
    let expr = expr_parser();

    // `name : Type` の signature 宣言
    let signature = lower_ident()
        .then_ignore(just(Token::Colon))
        .then(ty.clone())
        .map_with(|(name, ty), e| {
            Decl::Signature(SignatureDecl {
                name,
                ty,
                span: dspan(e.span()),
            })
        });

    // `name pat... = body` の value 宣言。signature と区別するため `=` の存在で確定。
    // `=` の直後で改行した body は layout pass が継続行として透過する (Elm 流の複数行 body)。
    let value = lower_ident()
        .then(pat.clone().repeated().collect::<Vec<_>>())
        .then_ignore(just(Token::Eq))
        .then(expr.clone())
        .map_with(|((name, params), body), e| {
            Decl::Value(ValueDecl {
                name,
                params,
                body,
                span: dspan(e.span()),
            })
        });

    // `type T a b = C1 a | C2 b`
    // constructor の引数は **atomic** な type expr のみ (パイプ `|` で次の constructor に
    // 進めるよう、空白区切りの繰り返しで type_expr を全部食べないようにする)。
    // 複数行の `= C1\n | C2` は継続行 (深い字下げ) として layout pass が改行を透過する。
    let type_decl = just(Token::Type)
        .ignore_then(upper_ident())
        .then(lower_ident().repeated().collect::<Vec<_>>())
        .then_ignore(just(Token::Eq))
        .then({
            let ctor = upper_ident()
                .then(type_atom_parser().repeated().collect::<Vec<_>>())
                .map_with(|(name, args), e| Constructor {
                    name,
                    args,
                    span: dspan(e.span()),
                });
            ctor.separated_by(just(Token::Pipe))
                .at_least(1)
                .collect::<Vec<_>>()
        })
        .map_with(|((name, params), constructors), e| {
            Decl::Type(TypeDecl {
                name,
                params,
                constructors,
                span: dspan(e.span()),
            })
        });

    // `type alias R a = { ... }`
    let type_alias = just(Token::Type)
        .then(just(Token::Alias))
        .ignore_then(upper_ident())
        .then(lower_ident().repeated().collect::<Vec<_>>())
        .then_ignore(just(Token::Eq))
        .then(ty.clone())
        .map_with(|((name, params), body), e| {
            Decl::TypeAlias(TypeAliasDecl {
                name,
                params,
                body,
                span: dspan(e.span()),
            })
        });

    // `slider name = expr`
    let slider = just(Token::Slider)
        .ignore_then(lower_ident())
        .then_ignore(just(Token::Eq))
        .then(expr.clone())
        .map_with(|(name, body), e| {
            Decl::Slider(SliderDecl {
                name,
                body,
                span: dspan(e.span()),
            })
        });

    // type_alias は `type alias` で始まるので type_decl より先にチェック。
    choice((type_alias, type_decl, slider, signature, value))
}

/// `exposing (..)` または `exposing (a, b, T(..), T(C1))`
fn exposing_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, Exposing, ParserError<'tokens, 'src>> + Clone {
    let all = just(Token::DotDot).map_with(|_, e| Exposing::All(dspan(e.span())));

    let ctor_list = upper_ident()
        .separated_by(just(Token::Comma))
        .at_least(1)
        .collect::<Vec<_>>();
    let item_ctors = just(Token::LParen)
        .ignore_then(choice((
            just(Token::DotDot).to(ExposingVariant::AllCtors),
            ctor_list.map(ExposingVariant::SomeCtors),
        )))
        .then_ignore(just(Token::RParen));
    let item_upper = upper_ident()
        .then(item_ctors.or_not())
        .map_with(|(name, variant), e| ExposingItem {
            name,
            variant: variant.unwrap_or(ExposingVariant::Bare),
            span: dspan(e.span()),
        });
    let item_lower = lower_ident().map_with(|name, e| ExposingItem {
        name,
        variant: ExposingVariant::Bare,
        span: dspan(e.span()),
    });
    let item = choice((item_upper, item_lower));

    let some = item
        .separated_by(just(Token::Comma))
        .at_least(1)
        .collect::<Vec<_>>()
        .map_with(|items, e| Exposing::Some(items, dspan(e.span())));

    choice((all, some)).delimited_by(just(Token::LParen), just(Token::RParen))
}

fn module_header_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, ModuleHeader, ParserError<'tokens, 'src>> + Clone
{
    just(Token::Module)
        .ignore_then(module_name())
        .then_ignore(just(Token::Exposing))
        .then(exposing_parser())
        .map_with(|(name, exposing), e| ModuleHeader {
            name,
            exposing,
            span: dspan(e.span()),
        })
}

fn import_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, Import, ParserError<'tokens, 'src>> + Clone {
    just(Token::Import)
        .ignore_then(module_name())
        .then(just(Token::As).ignore_then(upper_ident()).or_not())
        .then(
            just(Token::Exposing)
                .ignore_then(exposing_parser())
                .or_not(),
        )
        .map_with(|((module, alias), exposing), e| Import {
            module,
            alias,
            exposing,
            span: dspan(e.span()),
        })
}

pub fn module_parser<'tokens, 'src: 'tokens>()
-> impl Parser<'tokens, ParserInput<'tokens, 'src>, Module, ParserError<'tokens, 'src>> + Clone {
    // top-level 要素の境界は layout pass が `BlockSep` で区切る。各要素の後ろで
    // `BlockSep` (0 個以上) を吸収する。先頭の余分な `BlockSep` も leading で吸収する。
    let header = module_header_parser().then_ignore(top_sep());
    let import = import_parser().then_ignore(top_sep());
    let decl = decl_parser().then_ignore(top_sep());

    top_sep().ignore_then(
        header
            .or_not()
            .then(import.repeated().collect::<Vec<_>>())
            .then(decl.repeated().collect::<Vec<_>>())
            .map_with(|((header, imports), decls), e| Module {
                header,
                imports,
                decls,
                span: dspan(e.span()),
            }),
    )
}

/// 高レベル API: ソース文字列を直接 `Module` にパースする。
pub fn parse(src: &str) -> Result<Module, Vec<crate::diagnostic::Diagnostic>> {
    let tokens = match crate::syntax::lex::lex_raw(src) {
        Ok(t) => t,
        Err(errs) => {
            return Err(errs
                .into_iter()
                .map(|e| crate::diagnostic::Diagnostic::Lex {
                    span: DSpan::from(*e.span()),
                    message: format!("{e}"),
                })
                .collect());
        }
    };
    let tokens = crate::syntax::layout::apply_layout(src, tokens);
    let eoi: SimpleSpan = (src.len()..src.len()).into();
    let input = tokens[..].split_spanned(eoi);
    match module_parser().parse(input).into_result() {
        Ok(m) => Ok(m),
        Err(errs) => Err(errs
            .into_iter()
            .map(|e| crate::diagnostic::Diagnostic::Syntax {
                span: DSpan::from(*e.span()),
                message: format!("{e}"),
            })
            .collect()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_ok(src: &str) -> Module {
        match parse(src) {
            Ok(m) => m,
            Err(errs) => panic!("parse failed: {errs:?}"),
        }
    }

    #[test]
    fn empty_module() {
        let m = parse_ok("");
        assert!(m.header.is_none());
        assert!(m.imports.is_empty());
        assert!(m.decls.is_empty());
    }

    #[test]
    fn module_header() {
        let m = parse_ok("module Foo exposing (..)");
        let h = m.header.unwrap();
        assert_eq!(h.name.segments, vec!["Foo".to_string()]);
        assert!(matches!(h.exposing, Exposing::All(_)));
    }

    #[test]
    fn module_header_dotted() {
        let m = parse_ok("module Std.Gears exposing (Gear, mesh)");
        let h = m.header.unwrap();
        assert_eq!(
            h.name.segments,
            vec!["Std".to_string(), "Gears".to_string()]
        );
        match h.exposing {
            Exposing::Some(items, _) => {
                assert_eq!(items.len(), 2);
                assert_eq!(items[0].name, "Gear");
                assert_eq!(items[1].name, "mesh");
            }
            _ => panic!("expected Some"),
        }
    }

    #[test]
    fn imports() {
        let m = parse_ok(
            "module M exposing (..)\n\
             import Std exposing (cube)\n\
             import Std.Gears as G\n",
        );
        assert_eq!(m.imports.len(), 2);
        assert_eq!(m.imports[0].module.segments, vec!["Std"]);
        assert_eq!(m.imports[1].module.segments, vec!["Std", "Gears"]);
        assert_eq!(m.imports[1].alias.as_deref(), Some("G"));
    }

    #[test]
    fn value_decl_simple() {
        let m = parse_ok("answer = 42");
        assert_eq!(m.decls.len(), 1);
        match &m.decls[0] {
            Decl::Value(v) => {
                assert_eq!(v.name, "answer");
                assert!(v.params.is_empty());
                match &v.body {
                    Expr::Lit(Lit::Int(42), _) => {}
                    other => panic!("unexpected body: {other:?}"),
                }
            }
            other => panic!("unexpected decl: {other:?}"),
        }
    }

    #[test]
    fn value_decl_with_signature() {
        let m = parse_ok("f : Int -> Int\nf x = x + 1");
        assert_eq!(m.decls.len(), 2);
        assert!(matches!(m.decls[0], Decl::Signature(_)));
        assert!(matches!(m.decls[1], Decl::Value(_)));
    }

    #[test]
    fn type_decl_adt() {
        let m = parse_ok("type Shape = Cube Float Float Float | Sphere Float");
        match &m.decls[0] {
            Decl::Type(t) => {
                assert_eq!(t.name, "Shape");
                assert_eq!(t.constructors.len(), 2);
                assert_eq!(t.constructors[0].name, "Cube");
                assert_eq!(t.constructors[0].args.len(), 3);
            }
            _ => panic!("expected Type"),
        }
    }

    #[test]
    fn type_alias_record() {
        let m = parse_ok("type alias Output = { models : List Shape3D, bom : List BomEntry }");
        match &m.decls[0] {
            Decl::TypeAlias(a) => {
                assert_eq!(a.name, "Output");
                match &a.body {
                    TypeExpr::Record(fields, _) => assert_eq!(fields.len(), 2),
                    _ => panic!("expected Record"),
                }
            }
            _ => panic!("expected TypeAlias"),
        }
    }

    #[test]
    fn slider_decl() {
        let m = parse_ok("slider length = 6.0 .. 80.0");
        match &m.decls[0] {
            Decl::Slider(s) => {
                assert_eq!(s.name, "length");
                match &s.body {
                    Expr::Range { lo, hi, .. } => {
                        assert!(matches!(**lo, Expr::Lit(Lit::Float(_), _)));
                        assert!(matches!(**hi, Expr::Lit(Lit::Float(_), _)));
                    }
                    _ => panic!("expected Range"),
                }
            }
            _ => panic!("expected Slider"),
        }
    }

    #[test]
    fn expr_app_and_pipe() {
        let m = parse_ok("f = cube 10 10 length |> translate3d origin dest");
        let body = match &m.decls[0] {
            Decl::Value(v) => &v.body,
            _ => panic!(),
        };
        // パイプ右側に app があるはず
        match body {
            Expr::BinOp {
                op: BinOp::ApplyR, ..
            } => {}
            other => panic!("expected pipe, got {other:?}"),
        }
    }

    #[test]
    fn let_in_expr() {
        let m = parse_ok("f = let x = 1 in x + 2");
        match &m.decls[0] {
            Decl::Value(v) => match &v.body {
                Expr::Let { bindings, body, .. } => {
                    assert_eq!(bindings.len(), 1);
                    assert_eq!(bindings[0].name, "x");
                    assert!(matches!(**body, Expr::BinOp { op: BinOp::Add, .. }));
                }
                _ => panic!("expected Let"),
            },
            _ => panic!(),
        }
    }

    #[test]
    fn if_expr() {
        let m = parse_ok("f = if x then a else b");
        match &m.decls[0] {
            Decl::Value(v) => assert!(matches!(v.body, Expr::If { .. })),
            _ => panic!(),
        }
    }

    #[test]
    fn case_expr() {
        let m = parse_ok("f s = case s of\n    Cube x y z -> x\n    Sphere r -> r");
        match &m.decls[0] {
            Decl::Value(v) => match &v.body {
                Expr::Case { arms, .. } => assert_eq!(arms.len(), 2),
                _ => panic!("expected Case"),
            },
            _ => panic!(),
        }
    }

    #[test]
    fn lambda_expr() {
        let m = parse_ok("f = \\x y -> x + y");
        match &m.decls[0] {
            Decl::Value(v) => match &v.body {
                Expr::Lambda { params, .. } => assert_eq!(params.len(), 2),
                _ => panic!("expected Lambda"),
            },
            _ => panic!(),
        }
    }

    #[test]
    fn lambda_newline_after_arrow() {
        let m = parse_ok("f = \\x y ->\n    x + y");
        match &m.decls[0] {
            Decl::Value(v) => match &v.body {
                Expr::Lambda { params, .. } => assert_eq!(params.len(), 2),
                _ => panic!("expected Lambda"),
            },
            _ => panic!(),
        }
    }

    #[test]
    fn record_literal() {
        let m = parse_ok("f = { models = [1, 2], bom = [] }");
        match &m.decls[0] {
            Decl::Value(v) => match &v.body {
                Expr::Record(fields, _) => assert_eq!(fields.len(), 2),
                _ => panic!("expected Record"),
            },
            _ => panic!(),
        }
    }

    #[test]
    fn record_update() {
        let m = parse_ok("f = { r | x = 1 }");
        match &m.decls[0] {
            Decl::Value(v) => assert!(matches!(v.body, Expr::RecordUpdate { .. })),
            _ => panic!(),
        }
    }

    #[test]
    fn field_access() {
        let m = parse_ok("f = r.field.nested");
        match &m.decls[0] {
            Decl::Value(v) => match &v.body {
                Expr::Field { name, .. } => assert_eq!(name, "nested"),
                _ => panic!("expected Field"),
            },
            _ => panic!(),
        }
    }

    #[test]
    fn cons_pattern() {
        let m = parse_ok("f xs = case xs of\n    x :: rest -> x");
        match &m.decls[0] {
            Decl::Value(v) => match &v.body {
                Expr::Case { arms, .. } => {
                    assert!(matches!(arms[0].pattern, Pattern::Cons { .. }));
                }
                _ => panic!(),
            },
            _ => panic!(),
        }
    }

    #[test]
    fn bolt_example() {
        let src = "\
module Bolt exposing (main)

import Std exposing (Output)

main : Float -> Output
main length =
    let bolt = cube 10 10 length in
    { models = [bolt], bom = [], controls = [] }

slider length = 6.0 .. 80.0
";
        let m = parse_ok(src);
        assert!(m.header.is_some());
        assert_eq!(m.imports.len(), 1);
        assert_eq!(m.decls.len(), 3);
        assert!(matches!(m.decls[0], Decl::Signature(_)));
        assert!(matches!(m.decls[1], Decl::Value(_)));
        assert!(matches!(m.decls[2], Decl::Slider(_)));
    }
}
