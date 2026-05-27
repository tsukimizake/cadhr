//! cadhr-lang の値の型 (§5 型システム)。
//!
//! - シグネチャ (関数引数 / 戻り値、record field) では型必須
//! - 関数本体内のみ推論 (§5.2)
//! - List(T) などの多相は シグネチャに型変数を明示する rank-1 多相 (§5.3)
//! - 推論不能なら panic / Diagnostic、Any への silent fallback は禁止

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Type {
    Number,
    Atom,
    String,
    Bool,
    List(Box<Type>),
    /// record 型。`String` は record 名 (`output` 等)。
    /// 実体 (フィールド構成) は ModuleResolver の record table から引く。
    Record(String),

    Shape2D,
    Shape3D,
    /// `Shape2D` を `Plane` に貼り付けたもの。extrude 系の入力。
    PlacedShape2D,
    Path2D,
    Point2D,
    Point3D,
    Plane,

    /// 推論中の自由型変数 (本体内で生成される fresh α)。
    /// 推論完了時に具体型へ書き換えられる。残ったらエラー。
    Var(u32),

    /// シグネチャに登場する量化型変数 (`forall T.` 相当)。
    /// 関数呼び出し時に fresh `Var` に instantiate される。
    /// `String` は元の identifier (`"T"`, `"A"` 等)。
    Forall(String),
}

impl Type {
    pub fn list_of(t: Type) -> Self {
        Type::List(Box::new(t))
    }

    /// 自由 / 量化型変数を含まない場合 true。
    pub fn is_concrete(&self) -> bool {
        match self {
            Type::Var(_) | Type::Forall(_) => false,
            Type::List(t) => t.is_concrete(),
            _ => true,
        }
    }
}

impl std::fmt::Display for Type {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Type::Number => f.write_str("Number"),
            Type::Atom => f.write_str("Atom"),
            Type::String => f.write_str("String"),
            Type::Bool => f.write_str("Bool"),
            Type::List(t) => write!(f, "List({})", t),
            Type::Record(name) => write!(f, "{}", name),
            Type::Shape2D => f.write_str("Shape2D"),
            Type::Shape3D => f.write_str("Shape3D"),
            Type::PlacedShape2D => f.write_str("PlacedShape2D"),
            Type::Path2D => f.write_str("Path2D"),
            Type::Point2D => f.write_str("Point2D"),
            Type::Point3D => f.write_str("Point3D"),
            Type::Plane => f.write_str("Plane"),
            Type::Var(id) => write!(f, "α{}", id),
            Type::Forall(name) => write!(f, "{}", name),
        }
    }
}
