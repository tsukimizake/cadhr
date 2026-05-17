#[cfg(test)]
#[macro_use]
mod test_helpers;

pub mod assertions;
pub mod bezier;
pub mod bom;
pub mod collision;
pub mod constraint;
pub mod manifold_bridge;
pub mod module;
pub mod parse;
pub mod rational;
pub mod sweep;
pub mod term_processor;
pub mod term_rewrite;

/// 標準ライブラリ (`std/db.cadhr`) を探すための既定 include path。
/// 現時点では開発用に CARGO_MANIFEST_DIR を返す。将来的に配布時の解決を考える必要あり。
pub fn default_include_paths() -> Vec<std::path::PathBuf> {
    vec![std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))]
}
