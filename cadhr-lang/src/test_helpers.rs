//! テスト用のヘルパーマクロ。
//!
//! `assert_close!(a, b)` はデフォルト epsilon = 1e-6 で `a` と `b` の絶対差を比較する。
//! 第3引数で epsilon を上書き可能: `assert_close!(a, b, 1e-9)`。

macro_rules! assert_close {
    ($a:expr, $b:expr) => {
        approx::assert_abs_diff_eq!($a, $b, epsilon = 1e-6)
    };
    ($a:expr, $b:expr, $eps:expr) => {
        approx::assert_abs_diff_eq!($a, $b, epsilon = $eps)
    };
}
