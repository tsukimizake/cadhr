//! Prolog Term -> manifold-rs Manifold 変換層
//!
//! Term（書き換え後の項）を ManifoldExpr 中間表現に変換し、
//! それを manifold-rs の Manifold オブジェクトに評価する。

use crate::parse::{term_as_fixed_point, ArithOp, SrcSpan, Term};
use manifold_rs::{Manifold, Mesh};
use std::fmt;
use std::str::FromStr;
use strum_macros::{EnumIter, EnumString, IntoStaticStr};

// ============================================================
// TrackedF64: ソーススパン付きf64値
// ============================================================

#[derive(Debug, Clone, Copy)]
pub struct TrackedF64 {
    pub value: f64,
    pub source_span: Option<SrcSpan>,
}

impl TrackedF64 {
    pub fn plain(value: f64) -> Self {
        Self {
            value,
            source_span: None,
        }
    }

    pub fn with_span(value: f64, span: SrcSpan) -> Self {
        Self {
            value,
            source_span: Some(span),
        }
    }
}

const DEFAULT_SEGMENTS: u32 = 32;

/// ビルトインプリミティブのfunctor名を表すenum
/// strumによりfunctor文字列との相互変換が可能
#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumString, IntoStaticStr, EnumIter)]
#[strum(serialize_all = "lowercase")]
pub enum BuiltinFunctor {
    // プリミティブ
    Cube,
    Sphere,
    Cylinder,
    Tetrahedron,
    // CSG演算
    Union,
    Difference,
    Intersection,
    // 変形
    Translate,
    Scale,
    Rotate,
    // 2Dプロファイル
    #[strum(serialize = "p")]
    Point,
    Polygon,
    Circle,
    // 押し出し・回転体
    Extrude,
    Revolve,
    // ポリヘドロン
    Polyhedron,
}

/// functor名がビルトインプリミティブかどうかを判定
pub fn is_builtin_functor(functor: &str) -> bool {
    BuiltinFunctor::from_str(functor).is_ok()
}

/// 変換エラー
#[derive(Debug, Clone)]
pub enum ConversionError {
    /// 未知のプリミティブ/functor
    UnknownPrimitive(String),
    /// 引数の数が不一致
    ArityMismatch {
        functor: String,
        expected: String,
        got: usize,
    },
    /// 引数の型が不正
    TypeMismatch {
        functor: String,
        arg_index: usize,
        expected: &'static str,
    },
    /// 未束縛変数
    UnboundVariable(String),
}

impl fmt::Display for ConversionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConversionError::UnknownPrimitive(name) => {
                write!(f, "Unknown primitive: {}", name)
            }
            ConversionError::ArityMismatch {
                functor,
                expected,
                got,
            } => {
                write!(
                    f,
                    "Arity mismatch for {}: expected {}, got {}",
                    functor, expected, got
                )
            }
            ConversionError::TypeMismatch {
                functor,
                arg_index,
                expected,
            } => {
                write!(
                    f,
                    "Type mismatch for {} arg {}: expected {}",
                    functor, arg_index, expected
                )
            }
            ConversionError::UnboundVariable(name) => {
                write!(f, "Unbound variable: {}", name)
            }
        }
    }
}

impl std::error::Error for ConversionError {}

/// 引数抽出用ヘルパー
struct Args<'a> {
    args: &'a [Term],
    functor: &'a str,
}

impl<'a> Args<'a> {
    fn new(functor: &'a str, args: &'a [Term]) -> Self {
        Self { args, functor }
    }

    fn len(&self) -> usize {
        self.args.len()
    }

    fn tracked_f64(&self, i: usize) -> Result<TrackedF64, ConversionError> {
        if let Some((fp, span)) = term_as_fixed_point(&self.args[i]) {
            return Ok(TrackedF64 {
                value: fp.to_f64(),
                source_span: span,
            });
        }
        match &self.args[i] {
            Term::Var { name } | Term::AnnotatedVar { name, .. } => {
                Err(ConversionError::UnboundVariable(name.clone()))
            }
            _ => Err(ConversionError::TypeMismatch {
                functor: self.functor.to_string(),
                arg_index: i,
                expected: "number",
            }),
        }
    }

    fn u32(&self, i: usize) -> Result<u32, ConversionError> {
        if let Some((fp, _)) = term_as_fixed_point(&self.args[i]) {
            return match fp.to_i64_checked() {
                Some(v) if v >= 0 => Ok(v as u32),
                _ => Err(ConversionError::TypeMismatch {
                    functor: self.functor.to_string(),
                    arg_index: i,
                    expected: "non-negative integer",
                }),
            };
        }
        match &self.args[i] {
            Term::Var { name } | Term::AnnotatedVar { name, .. } => {
                Err(ConversionError::UnboundVariable(name.clone()))
            }
            _ => Err(ConversionError::TypeMismatch {
                functor: self.functor.to_string(),
                arg_index: i,
                expected: "integer",
            }),
        }
    }

    fn term(&self, i: usize) -> Result<ManifoldExpr, ConversionError> {
        ManifoldExpr::from_term(&self.args[i])
    }

    fn arity_error(&self, expected: &str) -> ConversionError {
        ConversionError::ArityMismatch {
            functor: self.functor.to_string(),
            expected: expected.to_string(),
            got: self.len(),
        }
    }
}

/// manifold-rs APIへの中間表現
#[derive(Debug, Clone)]
pub enum ManifoldExpr {
    // プリミティブ
    Cube {
        x: TrackedF64,
        y: TrackedF64,
        z: TrackedF64,
    },
    Sphere {
        radius: TrackedF64,
        segments: u32,
    },
    Cylinder {
        radius: TrackedF64,
        height: TrackedF64,
        segments: u32,
    },
    Tetrahedron,

    // CSG演算
    Union(Box<ManifoldExpr>, Box<ManifoldExpr>),
    Difference(Box<ManifoldExpr>, Box<ManifoldExpr>),
    Intersection(Box<ManifoldExpr>, Box<ManifoldExpr>),

    // 変形
    Translate {
        expr: Box<ManifoldExpr>,
        x: TrackedF64,
        y: TrackedF64,
        z: TrackedF64,
    },
    Scale {
        expr: Box<ManifoldExpr>,
        x: TrackedF64,
        y: TrackedF64,
        z: TrackedF64,
    },
    Rotate {
        expr: Box<ManifoldExpr>,
        x: TrackedF64,
        y: TrackedF64,
        z: TrackedF64,
    },

    // 2Dプロファイル
    Polygon {
        points: Vec<f64>,
    },
    Circle {
        radius: TrackedF64,
        segments: u32,
    },

    // 押し出し・回転体
    Extrude {
        profile: Box<ManifoldExpr>,
        height: TrackedF64,
    },
    Revolve {
        profile: Box<ManifoldExpr>,
        degrees: TrackedF64,
        segments: u32,
    },

    // ポリヘドロン
    Polyhedron {
        points: Vec<f64>,
        faces: Vec<Vec<u32>>,
    },
}

fn extract_polygon_points(list_term: &Term, functor: &str) -> Result<Vec<f64>, ConversionError> {
    match list_term {
        Term::List { items, .. } => {
            let mut points = Vec::with_capacity(items.len() * 2);
            for (i, item) in items.iter().enumerate() {
                match item {
                    Term::Struct { functor: f, args } if f == "p" && args.len() == 2 => {
                        for arg in args.iter() {
                            match term_as_fixed_point(arg) {
                                Some((fp, _)) => points.push(fp.to_f64()),
                                None => {
                                    return Err(ConversionError::TypeMismatch {
                                        functor: functor.to_string(),
                                        arg_index: i,
                                        expected: "p(number, number)",
                                    });
                                }
                            }
                        }
                    }
                    _ => {
                        return Err(ConversionError::TypeMismatch {
                            functor: functor.to_string(),
                            arg_index: i,
                            expected: "p(x, y)",
                        });
                    }
                }
            }
            Ok(points)
        }
        _ => Err(ConversionError::TypeMismatch {
            functor: functor.to_string(),
            arg_index: 0,
            expected: "list of p(x, y)",
        }),
    }
}

fn extract_polyhedron_points(list_term: &Term, functor: &str) -> Result<Vec<f64>, ConversionError> {
    match list_term {
        Term::List { items, .. } => {
            let mut points = Vec::with_capacity(items.len() * 3);
            for (i, item) in items.iter().enumerate() {
                match item {
                    Term::Struct { functor: f, args } if f == "p" && args.len() == 3 => {
                        for arg in args.iter() {
                            match term_as_fixed_point(arg) {
                                Some((fp, _)) => points.push(fp.to_f64()),
                                None => {
                                    return Err(ConversionError::TypeMismatch {
                                        functor: functor.to_string(),
                                        arg_index: i,
                                        expected: "p(number, number, number)",
                                    });
                                }
                            }
                        }
                    }
                    _ => {
                        return Err(ConversionError::TypeMismatch {
                            functor: functor.to_string(),
                            arg_index: i,
                            expected: "p(x, y, z)",
                        });
                    }
                }
            }
            Ok(points)
        }
        _ => Err(ConversionError::TypeMismatch {
            functor: functor.to_string(),
            arg_index: 0,
            expected: "list of p(x, y, z)",
        }),
    }
}

fn extract_polyhedron_faces(
    list_term: &Term,
    functor: &str,
) -> Result<Vec<Vec<u32>>, ConversionError> {
    match list_term {
        Term::List { items, .. } => {
            let mut faces = Vec::with_capacity(items.len());
            for (i, item) in items.iter().enumerate() {
                match item {
                    Term::List {
                        items: indices,
                        tail: None,
                    } => {
                        let mut face = Vec::with_capacity(indices.len());
                        for idx_term in indices.iter() {
                            match term_as_fixed_point(idx_term) {
                                Some((fp, _)) => match fp.to_i64_checked() {
                                    Some(v) if v >= 0 => face.push(v as u32),
                                    _ => {
                                        return Err(ConversionError::TypeMismatch {
                                            functor: functor.to_string(),
                                            arg_index: i,
                                            expected: "non-negative integer index",
                                        });
                                    }
                                },
                                None => {
                                    return Err(ConversionError::TypeMismatch {
                                        functor: functor.to_string(),
                                        arg_index: i,
                                        expected: "list of integers",
                                    });
                                }
                            }
                        }
                        faces.push(face);
                    }
                    _ => {
                        return Err(ConversionError::TypeMismatch {
                            functor: functor.to_string(),
                            arg_index: i,
                            expected: "[v0, v1, v2, ...]",
                        });
                    }
                }
            }
            Ok(faces)
        }
        _ => Err(ConversionError::TypeMismatch {
            functor: functor.to_string(),
            arg_index: 1,
            expected: "list of face index lists",
        }),
    }
}

impl ManifoldExpr {
    fn to_polygon_data(&self) -> Option<Vec<f64>> {
        match self {
            ManifoldExpr::Polygon { points } => Some(points.clone()),
            ManifoldExpr::Circle { radius, segments } => {
                let r = radius.value;
                let mut points = Vec::with_capacity(*segments as usize * 2);
                for i in 0..*segments {
                    let angle = 2.0 * std::f64::consts::PI * (i as f64) / (*segments as f64);
                    points.push(r * angle.cos());
                    points.push(r * angle.sin());
                }
                Some(points)
            }
            _ => None,
        }
    }

    /// Prolog Term から ManifoldExpr へ変換
    pub fn from_term(term: &Term) -> Result<Self, ConversionError> {
        match term {
            Term::Struct { functor, args } => Self::from_struct(functor, args),
            Term::InfixExpr { op, left, right } => Self::from_infix_expr(*op, left, right),
            Term::Var { name } => Err(ConversionError::UnboundVariable(name.clone())),
            Term::AnnotatedVar { name, .. } => Err(ConversionError::UnboundVariable(name.clone())),
            Term::Constraint { .. } => Err(ConversionError::UnknownPrimitive(
                "constraint should not reach mesh generation".to_string(),
            )),
            _ => Err(ConversionError::UnknownPrimitive(format!("{:?}", term))),
        }
    }

    /// 中置演算子をCAD操作として変換
    /// + -> union, - -> difference, * -> intersection
    fn from_infix_expr(op: ArithOp, left: &Term, right: &Term) -> Result<Self, ConversionError> {
        let left_expr = Box::new(Self::from_term(left)?);
        let right_expr = Box::new(Self::from_term(right)?);

        match op {
            ArithOp::Add => Ok(ManifoldExpr::Union(left_expr, right_expr)),
            ArithOp::Sub => Ok(ManifoldExpr::Difference(left_expr, right_expr)),
            ArithOp::Mul => Ok(ManifoldExpr::Intersection(left_expr, right_expr)),
            ArithOp::Div => Err(ConversionError::UnknownPrimitive(
                "division operator (/) is not supported for CAD operations".to_string(),
            )),
        }
    }

    fn from_struct(functor: &str, args: &[Term]) -> Result<Self, ConversionError> {
        let a = Args::new(functor, args);

        let builtin = BuiltinFunctor::from_str(functor)
            .map_err(|_| ConversionError::UnknownPrimitive(functor.to_string()))?;

        match builtin {
            // プリミティブ
            BuiltinFunctor::Cube if a.len() == 3 => Ok(ManifoldExpr::Cube {
                x: a.tracked_f64(0)?,
                y: a.tracked_f64(1)?,
                z: a.tracked_f64(2)?,
            }),
            BuiltinFunctor::Cube => Err(a.arity_error("3")),

            BuiltinFunctor::Sphere if a.len() == 1 => Ok(ManifoldExpr::Sphere {
                radius: a.tracked_f64(0)?,
                segments: DEFAULT_SEGMENTS,
            }),
            BuiltinFunctor::Sphere if a.len() == 2 => Ok(ManifoldExpr::Sphere {
                radius: a.tracked_f64(0)?,
                segments: a.u32(1)?,
            }),
            BuiltinFunctor::Sphere => Err(a.arity_error("1 or 2")),

            BuiltinFunctor::Cylinder if a.len() == 2 => Ok(ManifoldExpr::Cylinder {
                radius: a.tracked_f64(0)?,
                height: a.tracked_f64(1)?,
                segments: DEFAULT_SEGMENTS,
            }),
            BuiltinFunctor::Cylinder if a.len() == 3 => Ok(ManifoldExpr::Cylinder {
                radius: a.tracked_f64(0)?,
                height: a.tracked_f64(1)?,
                segments: a.u32(2)?,
            }),
            BuiltinFunctor::Cylinder => Err(a.arity_error("2 or 3")),

            BuiltinFunctor::Tetrahedron if a.len() == 0 => Ok(ManifoldExpr::Tetrahedron),
            BuiltinFunctor::Tetrahedron => Err(a.arity_error("0")),

            // CSG演算
            BuiltinFunctor::Union if a.len() == 2 => Ok(ManifoldExpr::Union(
                Box::new(a.term(0)?),
                Box::new(a.term(1)?),
            )),
            BuiltinFunctor::Union => Err(a.arity_error("2")),

            BuiltinFunctor::Difference if a.len() == 2 => Ok(ManifoldExpr::Difference(
                Box::new(a.term(0)?),
                Box::new(a.term(1)?),
            )),
            BuiltinFunctor::Difference => Err(a.arity_error("2")),

            BuiltinFunctor::Intersection if a.len() == 2 => Ok(ManifoldExpr::Intersection(
                Box::new(a.term(0)?),
                Box::new(a.term(1)?),
            )),
            BuiltinFunctor::Intersection => Err(a.arity_error("2")),

            // 変形
            BuiltinFunctor::Translate if a.len() == 4 => Ok(ManifoldExpr::Translate {
                expr: Box::new(a.term(0)?),
                x: a.tracked_f64(1)?,
                y: a.tracked_f64(2)?,
                z: a.tracked_f64(3)?,
            }),
            BuiltinFunctor::Translate => Err(a.arity_error("4")),

            BuiltinFunctor::Scale if a.len() == 4 => Ok(ManifoldExpr::Scale {
                expr: Box::new(a.term(0)?),
                x: a.tracked_f64(1)?,
                y: a.tracked_f64(2)?,
                z: a.tracked_f64(3)?,
            }),
            BuiltinFunctor::Scale => Err(a.arity_error("4")),

            BuiltinFunctor::Rotate if a.len() == 4 => Ok(ManifoldExpr::Rotate {
                expr: Box::new(a.term(0)?),
                x: a.tracked_f64(1)?,
                y: a.tracked_f64(2)?,
                z: a.tracked_f64(3)?,
            }),
            BuiltinFunctor::Rotate => Err(a.arity_error("4")),

            // 2Dプロファイル
            BuiltinFunctor::Point => {
                return Err(ConversionError::UnknownPrimitive(
                    "p is a data constructor, not a shape primitive".to_string(),
                ));
            }
            BuiltinFunctor::Polygon if a.len() == 1 => {
                let points = extract_polygon_points(&a.args[0], a.functor)?;
                Ok(ManifoldExpr::Polygon { points })
            }
            BuiltinFunctor::Polygon => Err(a.arity_error("1")),

            BuiltinFunctor::Circle if a.len() == 1 => Ok(ManifoldExpr::Circle {
                radius: a.tracked_f64(0)?,
                segments: DEFAULT_SEGMENTS,
            }),
            BuiltinFunctor::Circle if a.len() == 2 => Ok(ManifoldExpr::Circle {
                radius: a.tracked_f64(0)?,
                segments: a.u32(1)?,
            }),
            BuiltinFunctor::Circle => Err(a.arity_error("1 or 2")),

            // 押し出し・回転体
            BuiltinFunctor::Extrude if a.len() == 2 => Ok(ManifoldExpr::Extrude {
                profile: Box::new(a.term(0)?),
                height: a.tracked_f64(1)?,
            }),
            BuiltinFunctor::Extrude => Err(a.arity_error("2")),

            BuiltinFunctor::Revolve if a.len() == 2 => Ok(ManifoldExpr::Revolve {
                profile: Box::new(a.term(0)?),
                degrees: a.tracked_f64(1)?,
                segments: DEFAULT_SEGMENTS,
            }),
            BuiltinFunctor::Revolve if a.len() == 3 => Ok(ManifoldExpr::Revolve {
                profile: Box::new(a.term(0)?),
                degrees: a.tracked_f64(1)?,
                segments: a.u32(2)?,
            }),
            BuiltinFunctor::Revolve => Err(a.arity_error("2 or 3")),

            // ポリヘドロン
            BuiltinFunctor::Polyhedron if a.len() == 2 => {
                let points = extract_polyhedron_points(&a.args[0], a.functor)?;
                let faces = extract_polyhedron_faces(&a.args[1], a.functor)?;
                Ok(ManifoldExpr::Polyhedron { points, faces })
            }
            BuiltinFunctor::Polyhedron => Err(a.arity_error("2")),
        }
    }

    /// ManifoldExpr を manifold-rs の Manifold に評価
    pub fn evaluate(&self) -> Manifold {
        match self {
            // プリミティブ
            ManifoldExpr::Cube { x, y, z } => Manifold::cube(x.value, y.value, z.value),
            ManifoldExpr::Sphere { radius, segments } => Manifold::sphere(radius.value, *segments),
            ManifoldExpr::Cylinder {
                radius,
                height,
                segments,
            } => Manifold::cylinder(radius.value, radius.value, height.value, *segments),
            ManifoldExpr::Tetrahedron => Manifold::tetrahedron(),

            // CSG
            ManifoldExpr::Union(a, b) => a.evaluate().union(&b.evaluate()),
            ManifoldExpr::Difference(a, b) => a.evaluate().difference(&b.evaluate()),
            ManifoldExpr::Intersection(a, b) => a.evaluate().intersection(&b.evaluate()),

            // 変形
            ManifoldExpr::Translate { expr, x, y, z } => {
                expr.evaluate().translate(x.value, y.value, z.value)
            }
            ManifoldExpr::Scale { expr, x, y, z } => {
                expr.evaluate().scale(x.value, y.value, z.value)
            }
            ManifoldExpr::Rotate { expr, x, y, z } => {
                expr.evaluate().rotate(x.value, y.value, z.value)
            }

            // 2Dプロファイル (単体プレビュー時は薄いextrudeで3D化)
            ManifoldExpr::Polygon { points } => {
                Manifold::extrude(&[points], 0.001, 0, 0.0, 1.0, 1.0)
            }
            ManifoldExpr::Circle { .. } => {
                let data = self.to_polygon_data().unwrap();
                Manifold::extrude(&[&data], 0.001, 0, 0.0, 1.0, 1.0)
            }

            // 押し出し・回転体
            ManifoldExpr::Extrude { profile, height } => {
                let data = profile.to_polygon_data().unwrap();
                Manifold::extrude(&[&data], height.value, 0, 0.0, 1.0, 1.0)
            }
            ManifoldExpr::Revolve {
                profile,
                degrees,
                segments,
            } => {
                let data = profile.to_polygon_data().unwrap();
                Manifold::revolve(&[&data], *segments, degrees.value)
            }

            // ポリヘドロン
            ManifoldExpr::Polyhedron { points, faces } => {
                // manifold-rs の from_mesh で構築
                let verts: Vec<f32> = points.iter().map(|&v| v as f32).collect();
                let tri_indices: Vec<u32> = faces
                    .iter()
                    .flat_map(|face| {
                        // Fan triangulation for faces with > 3 vertices
                        (1..face.len() - 1).flat_map(move |i| {
                            vec![face[0], face[i as usize], face[i as usize + 1]]
                        })
                    })
                    .collect();
                let mesh = Mesh::new(&verts, &tri_indices);
                Manifold::from_mesh(mesh)
            }
        }
    }

    /// ManifoldExpr を Mesh に変換（法線計算込み）
    pub fn to_mesh(&self) -> Mesh {
        let manifold = self.evaluate();
        let with_normals = manifold.calculate_normals(0, 30.0);
        with_normals.to_mesh()
    }
}

/// 評価済みノード: ManifoldExpr + Manifold + Mesh + AABB + children
/// raycastによるノード特定に使用
#[derive(Clone)]
pub struct EvaluatedNode {
    pub expr: ManifoldExpr,
    pub mesh_verts: Vec<f32>,
    pub mesh_indices: Vec<u32>,
    pub aabb_min: [f64; 3],
    pub aabb_max: [f64; 3],
    pub children: Vec<EvaluatedNode>,
}

impl EvaluatedNode {
    /// ManifoldExprからTrackedF64のsource_spanを収集
    pub fn collect_tracked_spans(&self) -> Vec<(String, TrackedF64)> {
        collect_tracked_spans_from_expr(&self.expr)
    }
}

pub fn collect_tracked_spans_from_expr(expr: &ManifoldExpr) -> Vec<(String, TrackedF64)> {
    match expr {
        ManifoldExpr::Cube { x, y, z } => {
            vec![("x".into(), *x), ("y".into(), *y), ("z".into(), *z)]
        }
        ManifoldExpr::Sphere { radius, .. } => vec![("radius".into(), *radius)],
        ManifoldExpr::Cylinder { radius, height, .. } => {
            vec![("radius".into(), *radius), ("height".into(), *height)]
        }
        ManifoldExpr::Translate { x, y, z, .. } => {
            vec![("x".into(), *x), ("y".into(), *y), ("z".into(), *z)]
        }
        ManifoldExpr::Scale { x, y, z, .. } => {
            vec![("x".into(), *x), ("y".into(), *y), ("z".into(), *z)]
        }
        ManifoldExpr::Rotate { x, y, z, .. } => {
            vec![("x".into(), *x), ("y".into(), *y), ("z".into(), *z)]
        }
        ManifoldExpr::Extrude { height, .. } => vec![("height".into(), *height)],
        ManifoldExpr::Revolve { degrees, .. } => vec![("degrees".into(), *degrees)],
        ManifoldExpr::Circle { radius, .. } => vec![("radius".into(), *radius)],
        _ => vec![],
    }
}

fn build_evaluated_node(expr: &ManifoldExpr) -> EvaluatedNode {
    let manifold = expr.evaluate();
    let mesh = manifold.calculate_normals(0, 30.0).to_mesh();
    let mesh_verts = mesh.vertices();
    let mesh_indices = mesh.indices();
    let num_props = mesh.num_props() as usize;

    assert!(
        num_props >= 3,
        "mesh must have at least 3 properties (xyz) per vertex, got {num_props}"
    );
    let mut aabb_min = [f64::INFINITY; 3];
    let mut aabb_max = [f64::NEG_INFINITY; 3];
    for chunk in mesh_verts.chunks(num_props) {
        for i in 0..3 {
            let v = chunk[i] as f64;
            if v < aabb_min[i] {
                aabb_min[i] = v;
            }
            if v > aabb_max[i] {
                aabb_max[i] = v;
            }
        }
    }

    let children = match expr {
        ManifoldExpr::Union(a, b)
        | ManifoldExpr::Difference(a, b)
        | ManifoldExpr::Intersection(a, b) => {
            vec![build_evaluated_node(a), build_evaluated_node(b)]
        }
        ManifoldExpr::Translate { expr: e, .. }
        | ManifoldExpr::Scale { expr: e, .. }
        | ManifoldExpr::Rotate { expr: e, .. }
        | ManifoldExpr::Extrude { profile: e, .. }
        | ManifoldExpr::Revolve { profile: e, .. } => {
            vec![build_evaluated_node(e)]
        }
        _ => vec![],
    };

    EvaluatedNode {
        expr: expr.clone(),
        mesh_verts,
        mesh_indices,
        aabb_min,
        aabb_max,
        children,
    }
}

/// 複数のTermからMeshとEvaluatedNodeツリーを生成する
pub fn generate_mesh_and_tree_from_terms(
    terms: &[Term],
) -> Result<(Mesh, Vec<EvaluatedNode>), ConversionError> {
    if terms.is_empty() {
        return Err(ConversionError::UnknownPrimitive(
            "empty term list".to_string(),
        ));
    }

    let exprs: Vec<ManifoldExpr> = terms
        .iter()
        .map(ManifoldExpr::from_term)
        .collect::<Result<Vec<_>, _>>()?;

    let nodes: Vec<EvaluatedNode> = exprs.iter().map(|e| build_evaluated_node(e)).collect();

    let manifold = exprs
        .into_iter()
        .map(|e| e.evaluate())
        .reduce(|acc, m| acc.union(&m))
        .unwrap();

    let with_normals = manifold.calculate_normals(0, 30.0);
    Ok((with_normals.to_mesh(), nodes))
}

/// 複数のTermからMeshを生成する（全てをunionする）
pub fn generate_mesh_from_terms(terms: &[Term]) -> Result<Mesh, ConversionError> {
    if terms.is_empty() {
        return Err(ConversionError::UnknownPrimitive(
            "empty term list".to_string(),
        ));
    }

    let exprs: Vec<ManifoldExpr> = terms
        .iter()
        .map(ManifoldExpr::from_term)
        .collect::<Result<Vec<_>, _>>()?;

    // 全てのManifoldExprをunionで結合
    let manifold = exprs
        .into_iter()
        .map(|e| e.evaluate())
        .reduce(|acc, m| acc.union(&m))
        .unwrap(); // exprsが空でないことは上でチェック済み

    let with_normals = manifold.calculate_normals(0, 30.0);
    Ok(with_normals.to_mesh())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{number_int, struc, var};

    #[test]
    fn test_cube_conversion() {
        let term = struc(
            "cube".into(),
            vec![number_int(10), number_int(20), number_int(30)],
        );
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Cube { x, y, z } => {
                assert_eq!(x.value, 10.0);
                assert_eq!(y.value, 20.0);
                assert_eq!(z.value, 30.0);
            }
            _ => panic!("Expected Cube"),
        }
    }

    #[test]
    fn test_sphere_default_segments() {
        let term = struc("sphere".into(), vec![number_int(5)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Sphere { radius, segments } => {
                assert_eq!(radius.value, 5.0);
                assert_eq!(segments, DEFAULT_SEGMENTS);
            }
            _ => panic!("Expected Sphere"),
        }
    }

    #[test]
    fn test_sphere_explicit_segments() {
        let term = struc("sphere".into(), vec![number_int(5), number_int(16)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Sphere { radius, segments } => {
                assert_eq!(radius.value, 5.0);
                assert_eq!(segments, 16);
            }
            _ => panic!("Expected Sphere"),
        }
    }

    #[test]
    fn test_cylinder_default_segments() {
        let term = struc("cylinder".into(), vec![number_int(3), number_int(10)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Cylinder {
                radius,
                height,
                segments,
            } => {
                assert_eq!(radius.value, 3.0);
                assert_eq!(height.value, 10.0);
                assert_eq!(segments, DEFAULT_SEGMENTS);
            }
            _ => panic!("Expected Cylinder"),
        }
    }

    #[test]
    fn test_union_conversion() {
        let cube1 = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let cube2 = struc(
            "cube".into(),
            vec![number_int(2), number_int(2), number_int(2)],
        );
        let union_term = struc("union".into(), vec![cube1, cube2]);
        let expr = ManifoldExpr::from_term(&union_term).unwrap();
        assert!(matches!(expr, ManifoldExpr::Union(_, _)));
    }

    #[test]
    fn test_translate_conversion() {
        let cube = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let translated = struc(
            "translate".into(),
            vec![cube, number_int(5), number_int(10), number_int(15)],
        );
        let expr = ManifoldExpr::from_term(&translated).unwrap();
        match expr {
            ManifoldExpr::Translate { x, y, z, .. } => {
                assert_eq!(x.value, 5.0);
                assert_eq!(y.value, 10.0);
                assert_eq!(z.value, 15.0);
            }
            _ => panic!("Expected Translate"),
        }
    }

    #[test]
    fn test_unbound_variable_error() {
        let term = struc(
            "cube".into(),
            vec![var("X".into()), number_int(1), number_int(1)],
        );
        let result = ManifoldExpr::from_term(&term);
        assert!(matches!(result, Err(ConversionError::UnboundVariable(_))));
    }

    #[test]
    fn test_arity_mismatch() {
        let term = struc("cube".into(), vec![number_int(1), number_int(2)]);
        let result = ManifoldExpr::from_term(&term);
        assert!(matches!(result, Err(ConversionError::ArityMismatch { .. })));
    }

    #[test]
    fn test_unknown_primitive() {
        let term = struc("unknown_shape".into(), vec![number_int(1)]);
        let result = ManifoldExpr::from_term(&term);
        assert!(matches!(result, Err(ConversionError::UnknownPrimitive(_))));
    }

    #[test]
    fn test_nested_csg() {
        // difference(union(cube(1,1,1), cube(2,2,2)), sphere(1))
        let cube1 = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let cube2 = struc(
            "cube".into(),
            vec![number_int(2), number_int(2), number_int(2)],
        );
        let union_term = struc("union".into(), vec![cube1, cube2]);
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let diff = struc("difference".into(), vec![union_term, sphere]);

        let expr = ManifoldExpr::from_term(&diff).unwrap();
        assert!(matches!(expr, ManifoldExpr::Difference(_, _)));
    }

    #[test]
    fn test_operator_union() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // cube(1,1,1) + sphere(1) -> union
        let cube = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let add_term = arith_expr(ArithOp::Add, cube, sphere);

        let expr = ManifoldExpr::from_term(&add_term).unwrap();
        assert!(matches!(expr, ManifoldExpr::Union(_, _)));
    }

    #[test]
    fn test_operator_difference() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // cube(1,1,1) - sphere(1) -> difference
        let cube = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let sub_term = arith_expr(ArithOp::Sub, cube, sphere);

        let expr = ManifoldExpr::from_term(&sub_term).unwrap();
        assert!(matches!(expr, ManifoldExpr::Difference(_, _)));
    }

    #[test]
    fn test_operator_intersection() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // cube(1,1,1) * sphere(1) -> intersection
        let cube = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let mul_term = arith_expr(ArithOp::Mul, cube, sphere);

        let expr = ManifoldExpr::from_term(&mul_term).unwrap();
        assert!(matches!(expr, ManifoldExpr::Intersection(_, _)));
    }

    #[test]
    fn test_operator_nested() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // (cube(1,1,1) + sphere(1)) - cylinder(1,2)
        let cube = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let cylinder = struc("cylinder".into(), vec![number_int(1), number_int(2)]);

        let union_term = arith_expr(ArithOp::Add, cube, sphere);
        let diff_term = arith_expr(ArithOp::Sub, union_term, cylinder);

        let expr = ManifoldExpr::from_term(&diff_term).unwrap();
        match expr {
            ManifoldExpr::Difference(left, _) => {
                assert!(matches!(*left, ManifoldExpr::Union(_, _)));
            }
            _ => panic!("Expected Difference"),
        }
    }

    #[test]
    fn test_operator_division_error() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // cube(1,1,1) / sphere(1) -> error
        let cube = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let div_term = arith_expr(ArithOp::Div, cube, sphere);

        let result = ManifoldExpr::from_term(&div_term);
        assert!(matches!(result, Err(ConversionError::UnknownPrimitive(_))));
    }

    fn make_polygon_term(pts: Vec<(i64, i64)>) -> Term {
        let points: Vec<Term> = pts
            .into_iter()
            .map(|(x, y)| struc("p".into(), vec![number_int(x), number_int(y)]))
            .collect();
        struc("polygon".into(), vec![crate::parse::list(points, None)])
    }

    #[test]
    fn test_polygon_conversion() {
        let term = make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Polygon { points } => {
                assert_eq!(points, vec![1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
            }
            _ => panic!("Expected Polygon"),
        }
    }

    #[test]
    fn test_circle_default_segments() {
        let term = struc("circle".into(), vec![number_int(5)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Circle { radius, segments } => {
                assert_eq!(radius.value, 5.0);
                assert_eq!(segments, DEFAULT_SEGMENTS);
            }
            _ => panic!("Expected Circle"),
        }
    }

    #[test]
    fn test_extrude_polygon() {
        let polygon = make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]);
        let term = struc("extrude".into(), vec![polygon, number_int(3)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Extrude { profile, height } => {
                assert!(matches!(*profile, ManifoldExpr::Polygon { .. }));
                assert_eq!(height.value, 3.0);
            }
            _ => panic!("Expected Extrude"),
        }
    }

    #[test]
    fn test_revolve_circle() {
        let circle = struc("circle".into(), vec![number_int(5)]);
        let term = struc("revolve".into(), vec![circle, number_int(360)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Revolve {
                profile,
                degrees,
                segments,
            } => {
                assert!(matches!(*profile, ManifoldExpr::Circle { .. }));
                assert_eq!(degrees.value, 360.0);
                assert_eq!(segments, DEFAULT_SEGMENTS);
            }
            _ => panic!("Expected Revolve"),
        }
    }

    #[test]
    fn test_extrude_circle() {
        let circle = struc("circle".into(), vec![number_int(5)]);
        let term = struc("extrude".into(), vec![circle, number_int(10)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        match expr {
            ManifoldExpr::Extrude { profile, height } => {
                assert!(matches!(*profile, ManifoldExpr::Circle { .. }));
                assert_eq!(height.value, 10.0);
            }
            _ => panic!("Expected Extrude"),
        }
    }

    #[test]
    fn test_polygon_standalone_evaluate() {
        let term = make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        let mesh = expr.to_mesh();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_extrude_evaluate() {
        let polygon = make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]);
        let term = struc("extrude".into(), vec![polygon, number_int(3)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        let mesh = expr.to_mesh();
        assert!(mesh.vertices().len() > 0);
    }
}
