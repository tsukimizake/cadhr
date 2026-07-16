//! オフサイドルール (layout rule)。
//!
//! lexer 出力 (`Token::Newline` を含む) を、インデントに基づいて
//! `BlockOpen` / `BlockSep` / `BlockClose` を挿入したトークン列に変換する。parser は
//! `Newline` を一切見ず、これら仮想トークンで let / case-of / top-level のブロック構造を解釈する。
//! Haskell report の L 関数を cadhr 向けに簡約したもの。
//!
//! ブロックを開くキーワードは `let` / `of` / `sketch`。`let` / `sketch` の終端は
//! `in` が明示的に閉じる。`case ... of` の arm 列は dedent で閉じる。top-level は
//! 暗黙のブロック (列の床) として扱い、`BlockSep` のみ生成し開閉トークンは出さない。
//! sketch ブロック直下の binding 先頭の `let` は DSL キーワードとして扱い、ブロックを
//! 開かない。`end` は行頭でも継続扱い (区切り/閉じを発生させない)。
//!
//! 列 (インデント) は文字数で数える (タブ非対応 = タブ 1 個 1 列扱い)。

use crate::syntax::lex::{Spanned, Token};
use chumsky::span::SimpleSpan;

#[derive(Clone, Copy, PartialEq)]
enum Opener {
    Top,
    Let,
    Of,
    /// `sketch .. in .. end`。binding 列のレイアウトは `let` と同じで `in` が閉じる。
    Sketch,
}

enum Ctx {
    Layout { col: usize, opener: Opener },
    Bracket,
}

fn virt(tok: Token<'_>, span: SimpleSpan) -> Spanned<Token<'_>> {
    Spanned { inner: tok, span }
}

/// 各行頭のバイトオフセット (昇順)。`col_of` の起点に使う。
fn compute_line_starts(src: &str) -> Vec<usize> {
    let mut starts = vec![0usize];
    for (i, b) in src.bytes().enumerate() {
        if b == b'\n' {
            starts.push(i + 1);
        }
    }
    starts
}

/// dedent: スタック先頭が `Of` レイアウトを閉じ続け、最初の `Let` / `Sketch`
/// レイアウトを閉じたら停止する。
/// `Top` / `Bracket` に当たった場合は何もせず停止 (= 既に dedent で閉じ済み)。
fn close_until_let<'src>(
    stack: &mut Vec<Ctx>,
    out: &mut Vec<Spanned<Token<'src>>>,
    span: SimpleSpan,
) {
    loop {
        match stack.last() {
            Some(Ctx::Layout {
                opener: Opener::Of,
                ..
            }) => {
                out.push(virt(Token::BlockClose, span));
                stack.pop();
            }
            Some(Ctx::Layout {
                opener: Opener::Let | Opener::Sketch,
                ..
            }) => {
                out.push(virt(Token::BlockClose, span));
                stack.pop();
                break;
            }
            _ => break,
        }
    }
}

/// 閉じ括弧: 括弧内で開いた `Let` / `Of` レイアウトを閉じてから `Bracket` を 1 つ pop する。
fn close_brackets<'src>(
    stack: &mut Vec<Ctx>,
    out: &mut Vec<Spanned<Token<'src>>>,
    span: SimpleSpan,
) {
    while let Some(Ctx::Layout { opener, .. }) = stack.last() {
        if *opener == Opener::Top {
            break;
        }
        out.push(virt(Token::BlockClose, span));
        stack.pop();
    }
    if matches!(stack.last(), Some(Ctx::Bracket)) {
        stack.pop();
    }
}

/// 行頭トークンの列 `col` をスタック先頭のレイアウト列と比較し、区切り / 閉じを挿入する。
/// 先頭が `Bracket` のときは括弧内の継続行とみなし何もしない。
fn resolve_line<'src>(
    col: usize,
    stack: &mut Vec<Ctx>,
    out: &mut Vec<Spanned<Token<'src>>>,
    span: SimpleSpan,
) {
    loop {
        match stack.last() {
            Some(Ctx::Layout {
                col: lc,
                opener,
            }) => {
                if col == *lc {
                    out.push(virt(Token::BlockSep, span));
                    break;
                } else if col < *lc {
                    // Top は床: それ以上閉じない。
                    if *opener == Opener::Top {
                        break;
                    }
                    out.push(virt(Token::BlockClose, span));
                    stack.pop();
                    // 新しい先頭に対して再評価
                } else {
                    break; // col > lc: 継続行
                }
            }
            _ => break, // Bracket / 空: 継続
        }
    }
}

/// lexer 出力に layout rule を適用する。
pub fn apply_layout<'src>(src: &str, tokens: Vec<Spanned<Token<'src>>>) -> Vec<Spanned<Token<'src>>> {
    let line_starts = compute_line_starts(src);
    let col_of = |offset: usize| -> usize {
        let idx = line_starts.partition_point(|&s| s <= offset);
        let line_start = line_starts[idx - 1];
        src[line_start..offset].chars().count()
    };

    let mut out: Vec<Spanned<Token<'src>>> = Vec::with_capacity(tokens.len() + 16);
    let mut stack: Vec<Ctx> = Vec::new();
    let mut pending: Option<Opener> = None;
    let mut at_line_start = false;
    let mut started = false;

    for Spanned { inner, span } in tokens {
        if matches!(inner, Token::Newline) {
            at_line_start = true;
            continue;
        }
        let pos = span.start;
        let col = col_of(pos);
        let vspan: SimpleSpan = (pos..pos).into();

        // --- prefix: ブロック開始 / in による閉じ / 行頭解決 ---
        if !started {
            stack.push(Ctx::Layout {
                col,
                opener: Opener::Top,
            });
            started = true;
        } else if matches!(inner, Token::In) {
            close_until_let(&mut stack, &mut out, vspan);
        } else if let Some(opener) = pending.take() {
            out.push(virt(Token::BlockOpen, vspan));
            stack.push(Ctx::Layout { col, opener });
        } else if at_line_start && !matches!(inner, Token::End) {
            // `end` は sketch ブロックの明示的な終端で、行頭に来ても区切り/閉じを
            // 発生させない (継続扱い)。外側の let/case レイアウトを誤って区切らないため。
            resolve_line(col, &mut stack, &mut out, vspan);
        }
        at_line_start = false;

        // --- トークン自身の構造効果 ---
        match &inner {
            Token::LParen | Token::LBracket | Token::LBrace => stack.push(Ctx::Bracket),
            Token::RParen | Token::RBracket | Token::RBrace => {
                close_brackets(&mut stack, &mut out, vspan)
            }
            Token::Let => {
                // sketch ブロック直下の binding 先頭に現れる `let` は DSL の
                // 読み取り専用束縛キーワードであり、レイアウトブロックを開かない。
                let at_sketch_binding_head = matches!(
                    stack.last(),
                    Some(Ctx::Layout {
                        opener: Opener::Sketch,
                        ..
                    })
                ) && matches!(
                    out.last().map(|s| &s.inner),
                    Some(Token::BlockOpen | Token::BlockSep)
                );
                if !at_sketch_binding_head {
                    pending = Some(Opener::Let);
                }
            }
            Token::Of => pending = Some(Opener::Of),
            Token::Sketch => pending = Some(Opener::Sketch),
            _ => {}
        }

        out.push(Spanned { inner, span });
    }

    // --- EOF: 残るレイアウトを閉じる (Top は閉じない) ---
    let end: SimpleSpan = (src.len()..src.len()).into();
    while let Some(ctx) = stack.pop() {
        if let Ctx::Layout {
            opener: Opener::Let | Opener::Of | Opener::Sketch,
            ..
        } = ctx
        {
            out.push(virt(Token::BlockClose, end));
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::lex::lex_raw;

    /// ソースを lex → layout し、トークンを表示用文字列の列にする。
    fn layout_toks(src: &str) -> Vec<String> {
        let toks = lex_raw(src).expect("lex failed");
        apply_layout(src, toks)
            .into_iter()
            .map(|s| s.inner.to_string())
            .collect()
    }

    fn count(src: &str, tok: &str) -> usize {
        layout_toks(src).iter().filter(|t| *t == tok).count()
    }

    #[test]
    fn no_newline_tokens_survive() {
        let src = "x = 1\ny = 2\n";
        assert!(!layout_toks(src).iter().any(|t| t == "<newline>"));
    }

    #[test]
    fn top_level_decls_separated() {
        // 2 decl の間に BlockSep が 1 つ。Top は BlockOpen/Close を出さない。
        let src = "x = 1\ny = 2\n";
        assert_eq!(count(src, "<block-sep>"), 1);
        assert_eq!(count(src, "<block-open>"), 0);
        assert_eq!(count(src, "<block-close>"), 0);
    }

    #[test]
    fn multiline_application_is_continuation() {
        // 深くインデントされた引数は継続行: 区切りトークンが入らず f arg1 arg2 になる。
        let src = "f =\n    g\n        arg1\n        arg2\n";
        let toks = layout_toks(src);
        assert!(!toks.iter().any(|t| t == "<block-sep>"));
        // f = g arg1 arg2 の並び (block-open/close 無し)
        assert_eq!(
            toks,
            vec!["f", "=", "g", "arg1", "arg2"]
                .into_iter()
                .map(String::from)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn let_block_opens_and_closes() {
        let src = "f =\n    let\n        x = 1\n        y = 2\n    in\n    x\n";
        assert_eq!(count(src, "<block-open>"), 1);
        assert_eq!(count(src, "<block-close>"), 1);
        // 2 binding の間に BlockSep 1 つ
        assert_eq!(count(src, "<block-sep>"), 1);
    }

    #[test]
    fn let_in_same_line() {
        // 同一行 let..in でも BlockOpen/Close が 1 組挿入され、in の前で閉じる。
        let src = "f = let x = 1 in x\n";
        let toks = layout_toks(src);
        let open = toks.iter().position(|t| t == "<block-open>").unwrap();
        let close = toks.iter().position(|t| t == "<block-close>").unwrap();
        let in_pos = toks.iter().position(|t| t == "in").unwrap();
        assert!(open < close && close < in_pos, "{toks:?}");
    }

    #[test]
    fn nested_case_arms_associate_correctly() {
        // 外側 case の arm が内側 case に吸われない (誤結合バグの回帰テスト)。
        let src = "\
f x =
    case x of
        A ->
            case x of
                B -> 1
                C -> 2
        D -> 3
";
        // case-of ブロックが 2 つ → BlockOpen 2 / BlockClose 2
        assert_eq!(count(src, "<block-open>"), 2);
        assert_eq!(count(src, "<block-close>"), 2);
        // 内側 (B|C) と外側 (A|D) で BlockSep がそれぞれ 1 つずつ → 計 2
        assert_eq!(count(src, "<block-sep>"), 2);
    }

    #[test]
    fn case_in_parens_closes_at_rparen() {
        let src = "f = (case x of A -> 1)\n";
        let toks = layout_toks(src);
        let close = toks.iter().position(|t| t == "<block-close>").unwrap();
        let rparen = toks.iter().position(|t| t == ")").unwrap();
        assert!(close < rparen, "{toks:?}");
    }

    #[test]
    fn sketch_block_opens_and_closes() {
        let src = "sk =\n    sketch\n        var x = 1.0\n        let y = 2.0\n        p = p2 x y\n    in\n    p\n    end\n";
        assert_eq!(count(src, "<block-open>"), 1);
        assert_eq!(count(src, "<block-close>"), 1);
        // 3 binding の間に BlockSep 2 つ。let は sketch 直下なのでブロックを開かない。
        assert_eq!(count(src, "<block-sep>"), 2);
    }

    #[test]
    fn end_on_outer_let_column_is_continuation() {
        // `end` が外側 let の binding 列に来ても BlockSep を入れず継続扱いする。
        let src = "f =\n    let\n        sk = sketch\n                var x = 1.0\n                p = p2 x x\n            in p\n        end\n    in\n    sk\n";
        let toks = layout_toks(src);
        // let / sketch で 2 ブロックずつ開閉。`end` の位置に区切りは入らない。
        assert_eq!(count(src, "<block-open>"), 2);
        assert_eq!(count(src, "<block-close>"), 2);
        let end_pos = toks.iter().position(|t| t == "end").unwrap();
        assert_ne!(toks[end_pos - 1], "<block-sep>", "{toks:?}");
    }

    #[test]
    fn nested_let_inside_sketch_binding_rhs_opens_block() {
        // binding 先頭でない `let` (RHS 内) は通常のレイアウトブロックを開く。
        let src = "sk =\n    sketch\n        var x = 1.0\n        p = p2 (let a = 1.0 in a) x\n    in\n    p\n    end\n";
        // sketch 1 + 内側 let 1
        assert_eq!(count(src, "<block-open>"), 2);
        assert_eq!(count(src, "<block-close>"), 2);
    }

    #[test]
    fn multiline_list_has_no_layout_tokens() {
        // 括弧内はレイアウト停止: カンマ区切りのみで BlockSep が入らない。
        let src = "xs =\n    [ 1\n    , 2\n    , 3\n    ]\n";
        let toks = layout_toks(src);
        assert!(!toks.iter().any(|t| t == "<block-sep>"));
        assert!(!toks.iter().any(|t| t == "<block-open>"));
    }
}
