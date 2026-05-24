//! ビルトイン functor の単一登録 (LANG_SPEC §6)。
//!
//! 旧コードではビルトインが
//! - `manifold_bridge::BUILTIN_FUNCTORS` (slice + FunctorTag dispatch)
//! - `term_processor::BuiltinFunctorSet` (inventory crate 経由)
//! - `cadhr-lsp::completion::builtin_completion_items` (LSP 補完)
//! - `cadhr-lsp::hover::functor_doc` (LSP hover)
//! - `typecheck::initial_builtin_signatures` (型推論 stub)
//! に分散していた。このモジュールでそれらを 1 箇所に集約する。

use crate::types::Type;
use std::collections::HashMap;

/// 1 つの builtin functor (特定 arity)。同名で arity の違う overload は別エントリ。
#[derive(Debug, Clone)]
pub struct Builtin {
    pub name: &'static str,
    pub params: Vec<Type>,
    pub return_ty: Type,
    /// LSP hover / completion 用の人間向け使用例。
    pub usage: &'static str,
    /// LSP hover / completion 用の説明 (1〜数行)。
    pub doc: &'static str,
    /// LSP completion の snippet。`$1`, `$2`, ... は LSP の placeholder。
    pub snippet: &'static str,
}

impl Builtin {
    pub fn arity(&self) -> usize {
        self.params.len()
    }
}

#[derive(Debug, Clone, Default)]
pub struct BuiltinRegistry {
    entries: Vec<Builtin>,
    by_name_arity: HashMap<(String, usize), usize>,
}

impl BuiltinRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add(mut self, b: Builtin) -> Self {
        let key = (b.name.to_string(), b.arity());
        let idx = self.entries.len();
        if let Some(prev) = self.by_name_arity.insert(key.clone(), idx) {
            panic!(
                "duplicate builtin {}/{} (existing at index {}, new at {})",
                key.0, key.1, prev, idx
            );
        }
        self.entries.push(b);
        self
    }

    pub fn lookup(&self, name: &str, arity: usize) -> Option<&Builtin> {
        self.by_name_arity
            .get(&(name.to_string(), arity))
            .map(|&i| &self.entries[i])
    }

    pub fn iter(&self) -> impl Iterator<Item = &Builtin> {
        self.entries.iter()
    }

    /// `(name, arity)` -> 全 builtin。
    pub fn signatures(&self) -> HashMap<(String, usize), (Vec<Type>, Type)> {
        self.entries
            .iter()
            .map(|b| {
                (
                    (b.name.to_string(), b.arity()),
                    (b.params.clone(), b.return_ty.clone()),
                )
            })
            .collect()
    }

    /// `name` の全アリティ overload を列挙。LSP completion で「同名複数あり」の表示に使う。
    pub fn variants(&self, name: &str) -> Vec<&Builtin> {
        self.entries.iter().filter(|b| b.name == name).collect()
    }
}

// ============================================================
// 初期 builtin 登録 (LANG_SPEC §6.1)
// ============================================================

/// アプリ全体で共有する registry のシングルトン。
/// LANG_SPEC §6 の方針通り、登録はこの 1 箇所のみ。
pub fn registry() -> BuiltinRegistry {
    use Type::*;
    let n = || Number;
    let s3 = || Shape3D;
    let s2 = || Shape2D;
    let placed = || PlacedShape2D;
    let p2 = || Point2D;
    let p3 = || Point3D;
    let path = || Path2D;
    let str_ = || String;

    BuiltinRegistry::new()
        // ----- 3D primitives -----
        .add(Builtin {
            name: "cube",
            params: vec![n(), n(), n()],
            return_ty: s3(),
            usage: "cube(X, Y, Z) -> Shape3D",
            doc: "Axis-aligned box primitive.",
            snippet: "cube($1)",
        })
        .add(Builtin {
            name: "sphere",
            params: vec![n()],
            return_ty: s3(),
            usage: "sphere(R) -> Shape3D",
            doc: "Sphere with default tessellation.",
            snippet: "sphere($1)",
        })
        .add(Builtin {
            name: "sphere",
            params: vec![n(), n()],
            return_ty: s3(),
            usage: "sphere(R, Segments) -> Shape3D",
            doc: "Sphere with specified segment count.",
            snippet: "sphere($1, $2)",
        })
        .add(Builtin {
            name: "cylinder",
            params: vec![n(), n()],
            return_ty: s3(),
            usage: "cylinder(R, H) -> Shape3D",
            doc: "Cylinder primitive (default tessellation).",
            snippet: "cylinder($1, $2)",
        })
        .add(Builtin {
            name: "tetrahedron",
            params: vec![],
            return_ty: s3(),
            usage: "tetrahedron -> Shape3D",
            doc: "Regular tetrahedron primitive.",
            snippet: "tetrahedron",
        })
        // ----- 3D CSG (dimension-suffixed; LANG_SPEC §1.1) -----
        .add(Builtin {
            name: "union3d",
            params: vec![s3(), s3()],
            return_ty: s3(),
            usage: "union3d(A: Shape3D, B: Shape3D) -> Shape3D",
            doc: "Boolean union of two 3D shapes.",
            snippet: "union3d($1, $2)",
        })
        .add(Builtin {
            name: "difference3d",
            params: vec![s3(), s3()],
            return_ty: s3(),
            usage: "difference3d(A: Shape3D, B: Shape3D) -> Shape3D",
            doc: "Boolean difference (A minus B) of two 3D shapes.",
            snippet: "difference3d($1, $2)",
        })
        .add(Builtin {
            name: "intersection3d",
            params: vec![s3(), s3()],
            return_ty: s3(),
            usage: "intersection3d(A: Shape3D, B: Shape3D) -> Shape3D",
            doc: "Boolean intersection of two 3D shapes.",
            snippet: "intersection3d($1, $2)",
        })
        .add(Builtin {
            name: "hull3d",
            params: vec![s3(), s3()],
            return_ty: s3(),
            usage: "hull3d(A: Shape3D, B: Shape3D) -> Shape3D",
            doc: "Convex hull of two 3D shapes.",
            snippet: "hull3d($1, $2)",
        })
        // ----- 3D transforms (point-based translate per §1.1) -----
        .add(Builtin {
            name: "translate3d",
            params: vec![s3(), p3(), p3()],
            return_ty: s3(),
            usage: "translate3d(Shape: Shape3D, Src: Point3D, Dst: Point3D) -> Shape3D",
            doc: "Move Shape so Src coincides with Dst.",
            snippet: "translate3d($1, p3d($2), p3d($3))",
        })
        .add(Builtin {
            name: "scale3d",
            params: vec![s3(), n(), n(), n()],
            return_ty: s3(),
            usage: "scale3d(Shape: Shape3D, X: Number, Y: Number, Z: Number) -> Shape3D",
            doc: "Scale Shape per axis.",
            snippet: "scale3d($1, $2, $3, $4)",
        })
        .add(Builtin {
            name: "rotate3d",
            params: vec![s3(), n(), n(), n()],
            return_ty: s3(),
            usage: "rotate3d(Shape: Shape3D, RX: Number, RY: Number, RZ: Number) -> Shape3D",
            doc: "Rotate Shape by Euler angles (degrees).",
            snippet: "rotate3d($1, $2, $3, $4)",
        })
        .add(Builtin {
            name: "center3d",
            params: vec![s3(), p3()],
            return_ty: s3(),
            usage: "center3d(Shape: Shape3D, Target: Point3D) -> Shape3D",
            doc: "Translate Shape so its bounding-box center sits at Target.",
            snippet: "center3d($1, p3d($2))",
        })
        // ----- 2D primitives -----
        .add(Builtin {
            name: "circle",
            params: vec![n()],
            return_ty: s2(),
            usage: "circle(R) -> Shape2D",
            doc: "2D circle primitive.",
            snippet: "circle($1)",
        })
        // ----- 2D CSG -----
        .add(Builtin {
            name: "union2d",
            params: vec![s2(), s2()],
            return_ty: s2(),
            usage: "union2d(A: Shape2D, B: Shape2D) -> Shape2D",
            doc: "Boolean union of two 2D shapes.",
            snippet: "union2d($1, $2)",
        })
        .add(Builtin {
            name: "difference2d",
            params: vec![s2(), s2()],
            return_ty: s2(),
            usage: "difference2d(A: Shape2D, B: Shape2D) -> Shape2D",
            doc: "Boolean difference of two 2D shapes.",
            snippet: "difference2d($1, $2)",
        })
        .add(Builtin {
            name: "intersection2d",
            params: vec![s2(), s2()],
            return_ty: s2(),
            usage: "intersection2d(A: Shape2D, B: Shape2D) -> Shape2D",
            doc: "Boolean intersection of two 2D shapes.",
            snippet: "intersection2d($1, $2)",
        })
        .add(Builtin {
            name: "center2d",
            params: vec![s2(), p2()],
            return_ty: s2(),
            usage: "center2d(Profile: Shape2D, Target: Point2D) -> Shape2D",
            doc: "Translate Profile so its bounding-box center sits at Target.",
            snippet: "center2d($1, p2d($2))",
        })
        // ----- Points -----
        .add(Builtin {
            name: "p2d",
            params: vec![n(), n()],
            return_ty: p2(),
            usage: "p2d(X, Y) -> Point2D",
            doc: "2D point literal.",
            snippet: "p2d($1, $2)",
        })
        .add(Builtin {
            name: "p3d",
            params: vec![n(), n(), n()],
            return_ty: p3(),
            usage: "p3d(X, Y, Z) -> Point3D",
            doc: "3D point literal.",
            snippet: "p3d($1, $2, $3)",
        })
        // ----- Plane placement -----
        .add(Builtin {
            name: "rotateToXY",
            params: vec![s2()],
            return_ty: placed(),
            usage: "rotateToXY(Profile: Shape2D) -> PlacedShape2D",
            doc: "Place a 2D profile on the XY plane (identity placement).",
            snippet: "rotateToXY($1)",
        })
        .add(Builtin {
            name: "rotateToYZ",
            params: vec![s2()],
            return_ty: placed(),
            usage: "rotateToYZ(Profile: Shape2D) -> PlacedShape2D",
            doc: "Place a 2D profile on the YZ plane (extrude direction +X).",
            snippet: "rotateToYZ($1)",
        })
        .add(Builtin {
            name: "rotateToXZ",
            params: vec![s2()],
            return_ty: placed(),
            usage: "rotateToXZ(Profile: Shape2D) -> PlacedShape2D",
            doc: "Place a 2D profile on the XZ plane (extrude direction +Y).",
            snippet: "rotateToXZ($1)",
        })
        // ----- Extrusion -----
        .add(Builtin {
            name: "linear_extrude",
            params: vec![placed(), n()],
            return_ty: s3(),
            usage: "linear_extrude(Profile: PlacedShape2D, Height: Number) -> Shape3D",
            doc: "Extrude a placed 2D profile along its placement normal.",
            snippet: "linear_extrude($1, $2)",
        })
        .add(Builtin {
            name: "complex_extrude",
            params: vec![placed(), n(), n(), n(), n()],
            return_ty: s3(),
            usage: "complex_extrude(Profile: PlacedShape2D, H: Number, Twist: Number, ScaleX: Number, ScaleY: Number) -> Shape3D",
            doc: "Extrude with twist and per-axis scaling.",
            snippet: "complex_extrude($1, $2, $3, $4, $5)",
        })
        .add(Builtin {
            name: "revolve",
            params: vec![placed(), n()],
            return_ty: s3(),
            usage: "revolve(Profile: PlacedShape2D, Degrees: Number) -> Shape3D",
            doc: "Revolve a placed 2D profile around Y axis.",
            snippet: "revolve($1, $2)",
        })
        .add(Builtin {
            name: "sweep_extrude",
            params: vec![placed(), path()],
            return_ty: s3(),
            usage: "sweep_extrude(Profile: PlacedShape2D, Path: Path2D) -> Shape3D",
            doc: "Sweep a 2D profile along a 2D path.",
            snippet: "sweep_extrude($1, $2)",
        })
        // ----- I/O -----
        .add(Builtin {
            name: "stl",
            params: vec![str_()],
            return_ty: s3(),
            usage: "stl(Path: String) -> Shape3D",
            doc: "Import a mesh from an STL file (resolved at runtime).",
            snippet: "stl(\"$1\")",
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_has_no_duplicates() {
        let _ = registry(); // panic on duplicate
    }

    #[test]
    fn lookup_cube_by_name_arity() {
        let r = registry();
        let b = r.lookup("cube", 3).expect("cube/3 should exist");
        assert_eq!(b.return_ty, Type::Shape3D);
        assert_eq!(b.params.len(), 3);
    }

    #[test]
    fn sphere_overloads_both_present() {
        let r = registry();
        assert!(r.lookup("sphere", 1).is_some());
        assert!(r.lookup("sphere", 2).is_some());
    }

    #[test]
    fn variants_lists_overloads() {
        let r = registry();
        let v = r.variants("sphere");
        assert_eq!(v.len(), 2);
    }

    #[test]
    fn signatures_map_matches_lookup() {
        let r = registry();
        let sigs = r.signatures();
        let cube_sig = sigs.get(&("cube".to_string(), 3)).unwrap();
        assert_eq!(cube_sig.1, Type::Shape3D);
    }
}
