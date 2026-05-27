//! Builtin functor の registry。
//!
//! Phase 2 では型シグネチャだけを持ち、Phase 3 で evaluator (`runtime/builtin.rs`)
//! に評価実装を接続する。LSP の completion / hover もこのレジストリを参照する。

use crate::sema::ty::{Scheme, TyVar, TyVarGen, Type};
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct Builtin {
    pub name: &'static str,
    pub scheme: Scheme,
    pub doc: &'static str,
}

#[derive(Clone, Debug, Default)]
pub struct BuiltinRegistry {
    by_name: HashMap<&'static str, Builtin>,
}

impl BuiltinRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add(mut self, b: Builtin) -> Self {
        self.by_name.insert(b.name, b);
        self
    }

    pub fn get(&self, name: &str) -> Option<&Builtin> {
        self.by_name.get(name)
    }

    pub fn iter(&self) -> impl Iterator<Item = &Builtin> {
        self.by_name.values()
    }
}

/// 初期ビルトイン群。本仕様の最小セット (cube / sphere / cylinder / translate3d /
/// union / difference / intersect / linear_extrude / p3 / p2 / stl / intersect (Range))。
/// Phase 3 で evaluator 実装を `runtime/builtin.rs` に持たせて接続する。
pub fn registry() -> BuiltinRegistry {
    let mut g = TyVarGen::new();
    let r = BuiltinRegistry::new();

    // モノ型ビルトインを足すヘルパ。
    let mono = |name: &'static str, args: Vec<Type>, ret: Type, doc: &'static str| Builtin {
        name,
        scheme: Scheme::mono(Type::arrows(args, ret)),
        doc,
    };

    // 多相ビルトイン (Range, List 系)。fresh な TyVar を含む scheme を作る。
    let poly = |name: &'static str,
                vars: Vec<TyVar>,
                args: Vec<Type>,
                ret: Type,
                doc: &'static str|
     -> Builtin {
        Builtin {
            name,
            scheme: Scheme {
                vars,
                ty: Type::arrows(args, ret),
            },
            doc,
        }
    };

    let float = || Type::con("Float");
    let int = || Type::con("Int");
    let shape3d = || Type::con("Shape3D");
    let shape2d = || Type::con("Shape2D");
    let placed2d = || Type::con("PlacedShape2D");
    let point3d = || Type::con("Point3D");
    let point2d = || Type::con("Point2D");
    let plane = || Type::con("Plane");
    let string = || Type::con("String");
    let range_of = |t: Type| Type::app("Range", vec![t]);

    // -- Range 集合演算 (intersect : Range a -> Range a -> Range a)
    let a = g.fresh();
    let r = r.add(poly(
        "intersect",
        vec![a],
        vec![range_of(Type::Var(a)), range_of(Type::Var(a))],
        range_of(Type::Var(a)),
        "2 つの Range の積を返す (両方に含まれる区間)",
    ));

    // -- 3D primitives
    let r = r.add(mono(
        "cube",
        vec![float(), float(), float()],
        shape3d(),
        "原点中心の軸並行な箱",
    ));
    let r = r.add(mono("sphere", vec![float()], shape3d(), "原点中心の球"));
    let r = r.add(mono(
        "cylinder",
        vec![float(), float()],
        shape3d(),
        "Z 軸に沿った円柱",
    ));
    let r = r.add(mono("tetrahedron", vec![], shape3d(), "原点中心の正四面体"));

    // -- CSG 3D
    let r = r.add(mono(
        "union3d",
        vec![shape3d(), shape3d()],
        shape3d(),
        "3D 形状の和",
    ));
    let r = r.add(mono(
        "diff3d",
        vec![shape3d(), shape3d()],
        shape3d(),
        "3D 形状の差",
    ));
    let r = r.add(mono(
        "intersect3d",
        vec![shape3d(), shape3d()],
        shape3d(),
        "3D 形状の積",
    ));
    let r = r.add(mono(
        "hull3d",
        vec![shape3d(), shape3d()],
        shape3d(),
        "2 つの 3D 形状の凸包",
    ));

    // -- Transform 3D (Shape3D を最後の引数にしてパイプフレンドリーに)
    let r = r.add(mono(
        "translate3d",
        vec![point3d(), point3d(), shape3d()],
        shape3d(),
        "Shape3D の点 src を点 dst に運ぶ (`s |> translate3d src dst`)",
    ));
    let r = r.add(mono(
        "scale3d",
        vec![point3d(), shape3d()],
        shape3d(),
        "Shape3D を Point3D の各軸倍率で拡縮",
    ));
    let r = r.add(mono(
        "rotate3d",
        vec![point3d(), shape3d()],
        shape3d(),
        "Shape3D を Point3D の各軸回転角 (度) で回転",
    ));

    // -- 2D primitives
    let r = r.add(mono("circle", vec![float()], shape2d(), "原点中心の円"));
    let r = r.add(mono("empty_3d", vec![], shape3d(), "空の Shape3D"));
    let r = r.add(mono("empty_2d", vec![], shape2d(), "空の Shape2D"));

    // -- Place + extrude
    let r = r.add(mono(
        "place",
        vec![shape2d(), plane()],
        placed2d(),
        "2D 形状を 3D 平面に貼り付け",
    ));
    let r = r.add(mono(
        "linear_extrude",
        vec![placed2d(), float()],
        shape3d(),
        "貼り付け 2D 形状を平面法線方向に押し出し",
    ));

    // -- Points
    let r = r.add(mono(
        "p3",
        vec![float(), float(), float()],
        point3d(),
        "3D 点",
    ));
    let r = r.add(mono("p2", vec![float(), float()], point2d(), "2D 点"));

    // -- I/O
    let r = r.add(mono("stl", vec![string()], shape3d(), "STL ファイル読み込み"));

    // -- 数値変換
    let r = r.add(mono(
        "fromInt",
        vec![int()],
        float(),
        "Int を Float に変換",
    ));

    r
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_has_core_primitives() {
        let r = registry();
        assert!(r.get("cube").is_some());
        assert!(r.get("sphere").is_some());
        assert!(r.get("translate3d").is_some());
        assert!(r.get("intersect").is_some());
        assert!(r.get("p3").is_some());
    }

    #[test]
    fn cube_signature() {
        let r = registry();
        let cube = r.get("cube").unwrap();
        // 多相変数なし
        assert!(cube.scheme.vars.is_empty());
        // Float -> Float -> Float -> Shape3D
        assert_eq!(
            cube.scheme.ty.to_string(),
            "Float -> Float -> Float -> Shape3D"
        );
    }

    #[test]
    fn intersect_is_polymorphic() {
        let r = registry();
        let inter = r.get("intersect").unwrap();
        assert_eq!(inter.scheme.vars.len(), 1);
        // forall a. Range a -> Range a -> Range a
        let s = inter.scheme.to_string();
        assert!(s.contains("Range") && s.contains("->"));
    }
}
