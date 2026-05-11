//! 実行時アサーション組込。
//!
//! `assert_eq(Expected, Actual)` または `assert_eq(Expected, Actual, "ラベル")` をゴールとして
//! 書くと、両辺を数値に評価して比較する。一致すれば何も残さず成功、不一致なら `RewriteError` を返す。
//! 機構ライブラリで「中心距離整合」「歯車比整合」など、人間に読めるエラーメッセージで
//! 不整合を検出するために使う。

use crate::parse::{ScopedTerm, Term};
use crate::term_processor::BuiltinFunctorSet;

inventory::submit! {
    BuiltinFunctorSet {
        functors: &[("assert_eq", &[2_usize, 3_usize] as &[usize])],
        // 引数解決は `term_rewrite` 側のインターセプトで個別に行うため、
        // ここでは関与しない。
        resolve_args: false,
    }
}

/// `assert_eq` ゴールから (left, right, label) を取り出す。`Term::Struct` であることは呼び出し側で確認済みの想定。
pub fn split_assert_eq_args(args: &[ScopedTerm]) -> (ScopedTerm, ScopedTerm, Option<String>) {
    let left = args[0].clone();
    let right = args[1].clone();
    let label = if args.len() == 3 {
        match &args[2] {
            Term::StringLit { value } => Some(value.clone()),
            _ => None,
        }
    } else {
        None
    };
    (left, right, label)
}
