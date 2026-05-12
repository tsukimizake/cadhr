//! cadhr-lang の数値型。`BigRational` をラップして、線形演算を厳密に扱う。
//!
//! 設計意図:
//! - 制約ソルバ・項書き換え・式評価のすべてを厳密な有理数で行うことで、
//!   `400 / 30 = 40/3` のような非整除も精度損失なしに伝播させる。
//! - 表示は基本的に小数 2 桁丸めで現状互換 (`Display`)。
//!   エラーメッセージなど厳密値を見せたい文脈では `fmt_exact()` を使い、
//!   `40/3 (≈ 13.33)` のように分数と近似値を併記する。
//! - manifold-rs / BOM / UI の境界では `to_f64()` で吸収する。

use num_bigint::BigInt;
use num_rational::BigRational;
use num_traits::{FromPrimitive, Signed, ToPrimitive, Zero};

/// 厳密な有理数。
#[derive(Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Rational(BigRational);

impl Rational {
    pub fn from_integer(n: i64) -> Self {
        Self(BigRational::from_integer(BigInt::from(n)))
    }

    pub fn from_ratio(num: i64, denom: i64) -> Self {
        debug_assert!(denom != 0, "denominator must be non-zero");
        Self(BigRational::new(BigInt::from(num), BigInt::from(denom)))
    }

    /// f64 から構築。NaN/Infinity は `Rational::zero()` にフォールバック。
    pub fn from_f64(v: f64) -> Self {
        BigRational::from_f64(v).map(Self).unwrap_or_else(Self::zero)
    }

    /// 既存の `BigRational` をラップする (内部ユーティリティ)。
    pub fn from_big_rational(r: BigRational) -> Self {
        Self(r)
    }

    pub fn to_f64(&self) -> f64 {
        self.0.to_f64().unwrap_or(0.0)
    }

    pub fn zero() -> Self {
        Self(BigRational::zero())
    }

    pub fn is_zero(&self) -> bool {
        self.0.is_zero()
    }

    /// 分母が 1 の場合 (整数値) true。
    pub fn is_integer(&self) -> bool {
        self.0.is_integer()
    }

    pub fn abs(&self) -> Self {
        Self(self.0.abs())
    }

    pub fn inner(&self) -> &BigRational {
        &self.0
    }

    /// 厳密な分数表記。整数なら "5"、有限 10 進で表せれば "1.5"、
    /// それ以外は "40/3 (≈ 13.33)" のように分数と近似 10 進を併記する。
    pub fn fmt_exact(&self) -> String {
        if self.is_integer() {
            return format!("{}", self.0.numer());
        }
        // 2 桁丸めで round-trip 一致するなら有限 10 進と判定し、丸め値をそのまま返す。
        // 一致しなければ非有限 10 進 (例 40/3) とみなして分数 + 近似 10 進を併記する。
        let approx = (self.to_f64() * 100.0).round() / 100.0;
        if (approx * 100.0).round() as i64
            == (self.to_f64() * 100.0).round() as i64
            && BigRational::from_f64(approx).as_ref() == Some(&self.0)
        {
            return format!("{}", self);
        }
        format!("{}/{} (≈ {})", self.0.numer(), self.0.denom(), self)
    }

}

// ─────── Display / Debug ───────

impl std::fmt::Display for Rational {
    /// 「小数 2 桁丸め」で表示する。厳密値が必要な場合は `fmt_exact()` を使う。
    /// 整数なら `5` のように、有限 10 進なら `1.5` のように、それ以外は四捨五入し
    /// `13.33` のように 2 桁で表示する。
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let v = self.to_f64();
        if self.is_integer() {
            return write!(f, "{}", self.0.numer());
        }
        // 2 桁四捨五入の文字列化。負値や末尾 0 の扱いを揃える。
        let rounded = (v * 100.0).round() / 100.0;
        let formatted = format!("{:.2}", rounded);
        // 末尾 0 を削る (`3.50` → `3.5`)
        let trimmed = if formatted.contains('.') {
            formatted.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            formatted
        };
        write!(f, "{}", trimmed)
    }
}

impl std::fmt::Debug for Rational {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self)
    }
}

// ─────── 算術演算子 (値・参照の両方) ───────

macro_rules! impl_binop {
    ($trait:ident, $method:ident) => {
        impl std::ops::$trait for Rational {
            type Output = Rational;
            fn $method(self, rhs: Self) -> Self::Output {
                Rational(self.0.$method(rhs.0))
            }
        }
        impl std::ops::$trait<&Rational> for Rational {
            type Output = Rational;
            fn $method(self, rhs: &Self) -> Self::Output {
                Rational(self.0.$method(&rhs.0))
            }
        }
        impl std::ops::$trait<Rational> for &Rational {
            type Output = Rational;
            fn $method(self, rhs: Rational) -> Self::Output {
                Rational((&self.0).$method(rhs.0))
            }
        }
        impl std::ops::$trait<&Rational> for &Rational {
            type Output = Rational;
            fn $method(self, rhs: &Rational) -> Self::Output {
                Rational((&self.0).$method(&rhs.0))
            }
        }
    };
}

impl_binop!(Add, add);
impl_binop!(Sub, sub);
impl_binop!(Mul, mul);
impl_binop!(Div, div);

impl std::ops::Neg for Rational {
    type Output = Rational;
    fn neg(self) -> Self::Output {
        Rational(-self.0)
    }
}

impl std::ops::Neg for &Rational {
    type Output = Rational;
    fn neg(self) -> Self::Output {
        Rational(-&self.0)
    }
}

// ─────── 型変換 ───────

impl From<i64> for Rational {
    fn from(v: i64) -> Self {
        Self::from_integer(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_integer_basic() {
        let r = Rational::from_integer(5);
        assert_eq!(r.to_f64(), 5.0);
        assert!(r.is_integer());
    }

    #[test]
    fn ratio_simplifies() {
        let r = Rational::from_ratio(400, 30);
        // 簡約後は 40/3
        assert_eq!(format!("{}", r.inner()), "40/3");
        // 表示は 2 桁丸め
        assert_eq!(format!("{}", r), "13.33");
    }

    #[test]
    fn arithmetic_exact() {
        let a = Rational::from_ratio(40, 3);
        let b = Rational::from_integer(3);
        // (40/3) * 3 = 40 (整除性が完全に保たれる)
        let c = a * b;
        assert!(c.is_integer());
        assert_eq!(c.to_f64(), 40.0);
    }

    #[test]
    fn fmt_exact_integer() {
        let r = Rational::from_integer(42);
        assert_eq!(r.fmt_exact(), "42");
    }

    #[test]
    fn fmt_exact_finite_decimal() {
        let r = Rational::from_ratio(3, 2);
        // 1.5 は有限 10 進で表せる
        assert_eq!(r.fmt_exact(), "1.5");
    }

    #[test]
    fn fmt_exact_non_finite_decimal() {
        let r = Rational::from_ratio(40, 3);
        assert_eq!(r.fmt_exact(), "40/3 (≈ 13.33)");
    }

    #[test]
    fn negation() {
        let r = Rational::from_ratio(40, 3);
        assert_eq!(-r.clone(), Rational::from_ratio(-40, 3));
    }
}
