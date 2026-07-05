//! Builtin functor の registry。
//!
//! 型シグネチャを持ち、評価実装は `runtime/builtin.rs` で接続する。
//! LSP の completion / hover もこのレジストリを参照する。

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

/// builtin の型シグネチャに登場する型。LSP の completion / hover が参照する。
#[derive(Clone, Copy, Debug)]
pub struct BuiltinType {
    pub name: &'static str,
    /// 型引数名 (`List a` の `["a"]`)。空ならモノ型。
    pub params: &'static [&'static str],
    /// record alias の展開結果 (`Point2D` の `{ x : Float, y : Float }`)。
    pub alias_body: Option<&'static str>,
    pub doc: &'static str,
}

impl BuiltinType {
    /// hover / completion 表示用の宣言形。
    /// `type Shape3D` / `type List a` / `type alias Point2D = { x : Float, y : Float }`
    pub fn decl(&self) -> String {
        let mut s = String::from("type ");
        if self.alias_body.is_some() {
            s.push_str("alias ");
        }
        s.push_str(self.name);
        for p in self.params {
            s.push(' ');
            s.push_str(p);
        }
        if let Some(body) = self.alias_body {
            s.push_str(" = ");
            s.push_str(body);
        }
        s
    }
}

pub fn type_registry() -> &'static [BuiltinType] {
    const NONE: &[&str] = &[];
    const A: &[&str] = &["a"];
    &[
        BuiltinType {
            name: "Int",
            params: NONE,
            alias_body: None,
            doc: "整数。`fromInt` で Float に変換できる",
        },
        BuiltinType {
            name: "Float",
            params: NONE,
            alias_body: None,
            doc: "浮動小数点数。寸法・座標・角度 (度) に使う",
        },
        BuiltinType {
            name: "Bool",
            params: NONE,
            alias_body: None,
            doc: "真偽値 (`True` / `False`)。比較演算子と `if`-`then`-`else` で使う",
        },
        BuiltinType {
            name: "String",
            params: NONE,
            alias_body: None,
            doc: "文字列。`stl` のパスや `control3d` の名前に使う",
        },
        BuiltinType {
            name: "Shape3D",
            params: NONE,
            alias_body: None,
            doc: "3D 形状。`cube` / `sphere` などのプリミティブと `union3d` / `diff3d` などの CSG で組み立て、`main` の `models` に入れる",
        },
        BuiltinType {
            name: "Shape2D",
            params: NONE,
            alias_body: None,
            doc: "2D 形状。`circle` / `polygon` で作り、`extrude_xy` / `revolve_xy` などで Shape3D にする",
        },
        BuiltinType {
            name: "PlacedShape2D",
            params: NONE,
            alias_body: None,
            doc: "`place` で 3D 平面に貼り付けた 2D 形状。`linear_extrude` で押し出す",
        },
        BuiltinType {
            name: "Plane",
            params: NONE,
            alias_body: None,
            doc: "3D 空間内の平面。`place` の貼り付け先",
        },
        BuiltinType {
            name: "Point2D",
            params: NONE,
            alias_body: Some("{ x : Float, y : Float }"),
            doc: "2D 点。`p2 x y` で作る構造的 record 型",
        },
        BuiltinType {
            name: "Point3D",
            params: NONE,
            alias_body: Some("{ x : Float, y : Float, z : Float }"),
            doc: "3D 点。`p3 x y z` で作る構造的 record 型",
        },
        BuiltinType {
            name: "List",
            params: A,
            alias_body: None,
            doc: "リスト。リテラル `[a, b, c]`、`::` (cons)、`++` (append) と case パターンで扱う",
        },
        BuiltinType {
            name: "Range",
            params: A,
            alias_body: None,
            doc: "区間。`intersect` で 2 つの Range の積を取れる",
        },
    ]
}

pub fn get_type(name: &str) -> Option<&'static BuiltinType> {
    type_registry().iter().find(|t| t.name == name)
}

/// 初期ビルトイン群 (cube / sphere / cylinder / translate3d / union / difference /
/// intersect / linear_extrude / p3 / p2 / stl / intersect (Range))。
/// 評価実装は `runtime/builtin.rs` に登録する。
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
                constraints: Vec::new(),
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
    // Point2D / Point3D は構造的 record 型 (Elm の `type alias Point2D = { x, y }` 相当)。
    // 型注釈の `Point2D` / `Point3D` も infer 側で同じ record 型に解決される。
    let point3d = || {
        Type::Record(vec![
            ("x".to_string(), Type::con("Float")),
            ("y".to_string(), Type::con("Float")),
            ("z".to_string(), Type::con("Float")),
        ])
    };
    let point2d = || {
        Type::Record(vec![
            ("x".to_string(), Type::con("Float")),
            ("y".to_string(), Type::con("Float")),
        ])
    };
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
        "3D 形状の差 (`base |> diff3d cut` = base - cut)",
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

    // -- Transform 3D
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

    // -- 2D ポリゴン + 平面別 extrude (簡略 API)
    let r = r.add(mono(
        "polygon",
        vec![Type::app("List", vec![point2d()])],
        shape2d(),
        "Point2D の閉路リストから 2D ポリゴンを作る",
    ));
    let r = r.add(mono(
        "extrude_xy",
        vec![float(), shape2d()],
        shape3d(),
        "Shape2D を XY 平面上で Z 方向に Float だけ押し出す (`s |> extrude_xy h`)",
    ));
    let r = r.add(mono(
        "extrude_yz",
        vec![float(), shape2d()],
        shape3d(),
        "Shape2D を YZ 平面上で X 方向に Float だけ押し出す (`s |> extrude_yz h`)",
    ));
    let r = r.add(mono(
        "extrude_xz",
        vec![float(), shape2d()],
        shape3d(),
        "Shape2D を XZ 平面上で Y 方向に Float だけ押し出す (`s |> extrude_xz h`)",
    ));

    // -- Transform 2D
    let r = r.add(mono(
        "translate2d",
        vec![point2d(), point2d(), shape2d()],
        shape2d(),
        "Shape2D の点 src を点 dst に運ぶ (`s |> translate2d src dst`)",
    ));

    // -- 2D CSG
    let r = r.add(mono(
        "union2d",
        vec![shape2d(), shape2d()],
        shape2d(),
        "2D 形状の和",
    ));
    let r = r.add(mono(
        "diff2d",
        vec![shape2d(), shape2d()],
        shape2d(),
        "2D 形状の差 (左 - 右)",
    ));
    let r = r.add(mono(
        "intersect2d",
        vec![shape2d(), shape2d()],
        shape2d(),
        "2D 形状の積",
    ));

    // -- revolve / complex_extrude / sweep_extrude (XY 平面上の profile)
    let r = r.add(mono(
        "revolve_xy",
        vec![float(), shape2d()],
        shape3d(),
        "XY 平面上の Shape2D を Z 軸まわりに Float (度) 回転して 3D 化",
    ));
    let r = r.add(mono(
        "revolve_yz",
        vec![float(), shape2d()],
        shape3d(),
        "YZ 平面上の Shape2D を X 軸まわりに Float (度) 回転して 3D 化",
    ));
    let r = r.add(mono(
        "revolve_xz",
        vec![float(), shape2d()],
        shape3d(),
        "XZ 平面上の Shape2D を Y 軸まわりに Float (度) 回転して 3D 化",
    ));

    // complex_extrude (XY): height, twist (degree), scale_x, scale_y, profile (pipe-friendly に最後)
    let r = r.add(poly(
        "complex_extrude_xy",
        vec![],
        vec![float(), float(), float(), float(), shape2d()],
        shape3d(),
        "Shape2D を XY 平面で twist+scale 付き push out (height, twist_deg, sx, sy, profile)",
    ));

    // sweep_extrude (XY): 3D path に沿って 2D profile を押し出し
    let r = r.add(mono(
        "sweep_extrude_xy",
        vec![Type::app("List", vec![point3d()]), shape2d()],
        shape3d(),
        "XY 平面上の Shape2D を Point3D 列 path に沿って sweep",
    ));

    // center3d / center2d: Shape の AABB 中心 (Point) を返す。
    // 移動したいときは translate3d / translate2d と組み合わせる:
    //   `s |> translate3d (center3d s) p`
    let r = r.add(mono(
        "center3d",
        vec![shape3d()],
        point3d(),
        "Shape3D の AABB 中心 Point3D を返す",
    ));
    let r = r.add(mono(
        "center2d",
        vec![shape2d()],
        point2d(),
        "Shape2D の AABB 中心 Point2D を返す",
    ));

    // control point: GUI ドラッグ用。戻り値はそのまま Point に解決 (override 無し時)。
    let r = r.add(mono(
        "control3d",
        vec![string(), point3d()],
        point3d(),
        "名前付き 3D control point。GUI ドラッグで上書き可能",
    ));
    let r = r.add(mono(
        "control2d",
        vec![string(), point2d()],
        point2d(),
        "名前付き 2D control point。GUI ドラッグで上書き可能",
    ));

    // -- Debug.log : forall a. String -> a -> a (Elm 互換)
    let dbg_a = g.fresh();
    let r = r.add(poly(
        "Debug.log",
        vec![dbg_a],
        vec![string(), Type::Var(dbg_a)],
        Type::Var(dbg_a),
        "ラベル文字列と値を stderr に出力し、値をそのまま返す (Elm 互換)",
    ));

    // -- Bezier 曲線サンプリング
    // List Point2D を返すので polygon の入力にそのまま渡せる:
    //   polygon ([p2 0 0] ++ bezier_quad (p2 0 0) (p2 5 10) (p2 10 0) 20)
    let r = r.add(mono(
        "bezier_quad",
        vec![point2d(), point2d(), point2d(), int()],
        Type::app("List", vec![point2d()]),
        "2 次 Bezier (start, control, end, segments) → List Point2D",
    ));
    let r = r.add(mono(
        "bezier_cubic",
        vec![point2d(), point2d(), point2d(), point2d(), int()],
        Type::app("List", vec![point2d()]),
        "3 次 Bezier (start, c1, c2, end, segments) → List Point2D",
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
    let r = r.add(mono(
        "stl",
        vec![string()],
        shape3d(),
        "STL ファイル読み込み",
    ));

    // -- 数値変換
    let r = r.add(mono("fromInt", vec![int()], float(), "Int を Float に変換"));

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

    #[test]
    fn type_registry_covers_signature_types() {
        // 関数 registry の型シグネチャに登場する型コンストラクタが
        // type_registry で全て引けることを検査する。
        fn collect(t: &Type, names: &mut Vec<String>) {
            match t {
                Type::Con(name, args) => {
                    names.push(name.clone());
                    for a in args {
                        collect(a, names);
                    }
                }
                Type::Arrow(a, b) => {
                    collect(a, names);
                    collect(b, names);
                }
                Type::Record(fields) => {
                    for (_, ft) in fields {
                        collect(ft, names);
                    }
                }
                Type::Var(_) => {}
            }
        }
        let mut names = Vec::new();
        for b in registry().iter() {
            collect(&b.scheme.ty, &mut names);
        }
        for name in names {
            assert!(get_type(&name).is_some(), "type_registry missing: {name}");
        }
    }

    #[test]
    fn builtin_type_decl_rendering() {
        assert_eq!(get_type("Shape3D").unwrap().decl(), "type Shape3D");
        assert_eq!(get_type("List").unwrap().decl(), "type List a");
        assert_eq!(
            get_type("Point2D").unwrap().decl(),
            "type alias Point2D = { x : Float, y : Float }"
        );
    }

    #[test]
    fn debug_log_is_polymorphic_identity() {
        let r = registry();
        let dbg = r.get("Debug.log").unwrap();
        assert_eq!(dbg.scheme.vars.len(), 1);
        // forall a. String -> a -> a
        let s = dbg.scheme.to_string();
        assert!(s.contains("String") && s.contains("-> t1 -> t1"));
    }
}
