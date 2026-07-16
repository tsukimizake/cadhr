//! 評価器 (Elm-like cadhr-lang の正格関数型 evaluator)。
//!
//! - `value`: 実行時値 (`Value`)
//! - `eval`: 正格評価ルール
//! - `builtin`: builtin 関数の評価実装 (型 sig は `sema::builtin`、評価は `runtime::builtin`)

pub mod builtin;
pub mod eval;
pub mod value;

#[cfg(feature = "manifold")]
pub mod manifold_bridge;
