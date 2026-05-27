//! builtin 関数の評価実装。
//!
//! `sema::builtin::registry()` で型シグネチャを定義しているのと対応する形で、各
//! 関数の実行時挙動をここに登録する。manifold-rs 呼び出しは行わず、宣言的な
//! `Model3D` を組み立てるだけ (Phase 4 で `manifold_bridge::evaluate` に接続)。

use crate::runtime::value::{Model3D, Value};
use std::collections::HashMap;

pub type BuiltinFn = fn(&[Value]) -> Result<Value, String>;

#[derive(Clone)]
pub struct BuiltinEval {
    pub arity: usize,
    pub eval: BuiltinFn,
}

/// 実行時 builtin 辞書。`sema::builtin::registry()` と name set が一致する。
#[derive(Default)]
pub struct BuiltinEvalRegistry {
    pub by_name: HashMap<&'static str, BuiltinEval>,
}

impl BuiltinEvalRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add(mut self, name: &'static str, arity: usize, eval: BuiltinFn) -> Self {
        self.by_name.insert(name, BuiltinEval { arity, eval });
        self
    }

    pub fn get(&self, name: &str) -> Option<&BuiltinEval> {
        self.by_name.get(name)
    }
}

fn as_f64(v: &Value) -> Result<f64, String> {
    match v {
        Value::Float(x) => Ok(*x),
        Value::Int(n) => Ok(*n as f64),
        _ => Err(format!("Float が期待されましたが {v} でした")),
    }
}

fn as_int(v: &Value) -> Result<i64, String> {
    match v {
        Value::Int(n) => Ok(*n),
        _ => Err(format!("Int が期待されましたが {v} でした")),
    }
}

fn as_shape3d(v: &Value) -> Result<Model3D, String> {
    match v {
        Value::Shape3D(m) => Ok(m.clone()),
        _ => Err(format!("Shape3D が期待されましたが {v} でした")),
    }
}

fn as_point3d(v: &Value) -> Result<(f64, f64, f64), String> {
    match v {
        Value::Opaque(tag, args) if tag == "Point3D" && args.len() == 3 => {
            Ok((as_f64(&args[0])?, as_f64(&args[1])?, as_f64(&args[2])?))
        }
        _ => Err(format!("Point3D が期待されましたが {v} でした")),
    }
}

pub fn registry() -> BuiltinEvalRegistry {
    BuiltinEvalRegistry::new()
        // -- 3D primitives
        .add("cube", 3, |args| {
            Ok(Value::Shape3D(Model3D::Cube {
                x: as_f64(&args[0])?,
                y: as_f64(&args[1])?,
                z: as_f64(&args[2])?,
            }))
        })
        .add("sphere", 1, |args| {
            Ok(Value::Shape3D(Model3D::Sphere(as_f64(&args[0])?)))
        })
        .add("cylinder", 2, |args| {
            Ok(Value::Shape3D(Model3D::Cylinder {
                r: as_f64(&args[0])?,
                h: as_f64(&args[1])?,
            }))
        })
        .add("tetrahedron", 0, |_| Ok(Value::Shape3D(Model3D::Tetrahedron)))
        .add("empty_3d", 0, |_| Ok(Value::Shape3D(Model3D::Empty)))
        // -- CSG 3D
        .add("union3d", 2, |args| {
            Ok(Value::Shape3D(Model3D::Union(
                Box::new(as_shape3d(&args[0])?),
                Box::new(as_shape3d(&args[1])?),
            )))
        })
        .add("diff3d", 2, |args| {
            Ok(Value::Shape3D(Model3D::Diff(
                Box::new(as_shape3d(&args[0])?),
                Box::new(as_shape3d(&args[1])?),
            )))
        })
        .add("intersect3d", 2, |args| {
            Ok(Value::Shape3D(Model3D::Intersect(
                Box::new(as_shape3d(&args[0])?),
                Box::new(as_shape3d(&args[1])?),
            )))
        })
        .add("hull3d", 2, |args| {
            Ok(Value::Shape3D(Model3D::Hull(
                Box::new(as_shape3d(&args[0])?),
                Box::new(as_shape3d(&args[1])?),
            )))
        })
        // -- Transform 3D (Shape3D を最後の引数にする)
        .add("translate3d", 3, |args| {
            Ok(Value::Shape3D(Model3D::Translate {
                src: as_point3d(&args[0])?,
                dst: as_point3d(&args[1])?,
                shape: Box::new(as_shape3d(&args[2])?),
            }))
        })
        .add("scale3d", 2, |args| {
            Ok(Value::Shape3D(Model3D::Scale {
                factor: as_point3d(&args[0])?,
                shape: Box::new(as_shape3d(&args[1])?),
            }))
        })
        .add("rotate3d", 2, |args| {
            Ok(Value::Shape3D(Model3D::Rotate {
                angles: as_point3d(&args[0])?,
                shape: Box::new(as_shape3d(&args[1])?),
            }))
        })
        // -- Points
        .add("p3", 3, |args| {
            Ok(Value::Opaque(
                "Point3D".to_string(),
                vec![
                    Value::Float(as_f64(&args[0])?),
                    Value::Float(as_f64(&args[1])?),
                    Value::Float(as_f64(&args[2])?),
                ],
            ))
        })
        .add("p2", 2, |args| {
            Ok(Value::Opaque(
                "Point2D".to_string(),
                vec![
                    Value::Float(as_f64(&args[0])?),
                    Value::Float(as_f64(&args[1])?),
                ],
            ))
        })
        // -- 数値変換
        .add("fromInt", 1, |args| Ok(Value::Float(as_int(&args[0])? as f64)))
        // -- Range の集合演算
        .add("intersect", 2, |args| match (&args[0], &args[1]) {
            (
                Value::Range {
                    lo: l1,
                    hi: h1,
                    is_int: ii1,
                },
                Value::Range {
                    lo: l2,
                    hi: h2,
                    is_int: ii2,
                },
            ) if ii1 == ii2 => {
                let lo = l1.max(*l2);
                let hi = h1.min(*h2);
                if lo > hi {
                    return Err(format!(
                        "intersect: 空の range ({lo}..{hi} となりました)"
                    ));
                }
                Ok(Value::Range {
                    lo,
                    hi,
                    is_int: *ii1,
                })
            }
            _ => Err("intersect: Range a を 2 つ要求".to_string()),
        })
        // -- 簡単な I/O プレースホルダ (Phase 4 以降で実装)
        .add("stl", 1, |args| match &args[0] {
            Value::String(path) => Ok(Value::Opaque(
                "Stl".to_string(),
                vec![Value::String(path.clone())],
            )),
            _ => Err("stl: 文字列パス を要求".to_string()),
        })
        // -- 2D primitives (簡略実装)
        .add("circle", 1, |args| {
            Ok(Value::Opaque(
                "Shape2D".to_string(),
                vec![Value::Float(as_f64(&args[0])?)],
            ))
        })
        .add("empty_2d", 0, |_| {
            Ok(Value::Opaque("Shape2D".to_string(), vec![]))
        })
        // place / linear_extrude (Shape2D を Shape3D に変換 — 当面 opaque で
        // データを保持するだけ)
        .add("place", 2, |args| {
            Ok(Value::Opaque(
                "PlacedShape2D".to_string(),
                vec![args[0].clone(), args[1].clone()],
            ))
        })
        .add("linear_extrude", 2, |args| match &args[0] {
            Value::Opaque(tag, _) if tag == "PlacedShape2D" => {
                // 仮実装: Empty Shape3D を返す。Phase 4 で manifold-rs に橋渡し。
                Ok(Value::Shape3D(Model3D::Empty))
            }
            _ => Err("linear_extrude: PlacedShape2D を要求".to_string()),
        })
}
