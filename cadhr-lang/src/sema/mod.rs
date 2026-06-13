//! 意味解析層 (semantic analysis)。
//!
//! - `ty`: 保存・公開用の immutable な型 (`Type`, `TyVar`, `Scheme`)
//! - `infer_ty`: 推論専用の可変型 (`InferTy`, Rc<RefCell> + level) と in-place 単一化
//! - `env`: 型環境 (識別子 → `InferScheme`)
//! - `builtin`: builtin functor の registry
//! - `infer`: Algorithm J ベースの型推論

pub mod builtin;
pub mod class;
pub mod env;
pub mod exhaustive;
pub mod infer;
pub mod infer_ty;
pub mod slider;
pub mod ty;
