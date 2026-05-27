//! 意味解析層 (semantic analysis)。
//!
//! - `ty`: 型 (`Type`, `TyVar`, `Scheme`) の表現
//! - `subst`: 置換と単一化 (HM の基本ピース)
//! - `env`: 型環境 (識別子 → スキーム)
//! - `builtin`: builtin functor の registry
//! - `infer`: Algorithm W ベースの型推論

pub mod builtin;
pub mod env;
pub mod infer;
pub mod subst;
pub mod ty;
