//! builtin 関数の評価実装。
//!
//! `sema::builtin::registry()` で型シグネチャを定義しているのと対応する形で、各
//! 関数の実行時挙動をここに登録する。manifold-csg 呼び出しは行わず、宣言的な
//! `Model3D` を組み立てて `manifold_bridge::evaluate` に渡す。

use crate::runtime::value::{Model2D, Model3D, Plane3D, Value};
use std::cell::RefCell;
use std::collections::HashMap;
use std::path::PathBuf;

thread_local! {
    /// GUI が control point をドラッグした場合の override マップ。`run_main` 直前に
    /// `set_control_overrides` で書き換える。eval 中 `control3d` / `control2d` builtin
    /// がこのマップを参照する。
    pub static CONTROL_OVERRIDES: RefCell<HashMap<String, [f64; 3]>> = RefCell::new(HashMap::new());
    /// eval 中に呼ばれた control point の (name, current_value) を記録する。
    /// `run_main` が完了したあと `take_recorded_controls` で取り出す。
    pub static RECORDED_CONTROLS: RefCell<Vec<(String, [f64; 3])>> = RefCell::new(Vec::new());
    /// `center3d` などの builtin が内部で manifold 評価する際に使う STL 検索パス。
    /// `run_main` 直前に `set_include_paths` で更新する。
    pub static INCLUDE_PATHS: RefCell<Vec<PathBuf>> = RefCell::new(Vec::new());
}

/// `run_main` が eval 前に呼んで thread-local の override map を更新する。
/// 同時に前回の `RECORDED_CONTROLS` を確実に初期化する (前回の eval が早期 return で
/// take を呼ばずに終わった場合に備えて)。
pub fn set_control_overrides(overrides: HashMap<String, [f64; 3]>) {
    CONTROL_OVERRIDES.with(|c| *c.borrow_mut() = overrides);
    RECORDED_CONTROLS.with(|r| r.borrow_mut().clear());
}

/// `run_main` が eval 後に呼んで recorded control points を取り出す。
pub fn take_recorded_controls() -> Vec<(String, [f64; 3])> {
    RECORDED_CONTROLS.with(|r| std::mem::take(&mut *r.borrow_mut()))
}

/// `run_main` 直前に呼んで STL 検索パスを更新する。
pub fn set_include_paths(paths: Vec<PathBuf>) {
    INCLUDE_PATHS.with(|p| *p.borrow_mut() = paths);
}

fn as_string(v: &Value) -> Result<String, String> {
    match v {
        Value::String(s) => Ok(s.clone()),
        _ => Err(format!("String が期待されましたが {v} でした")),
    }
}

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

/// Point2D / Point3D は record 値 (`{ x, y }` / `{ x, y, z }`) として表現する。
fn point2d_value(x: f64, y: f64) -> Value {
    Value::Record(vec![
        ("x".to_string(), Value::Float(x)),
        ("y".to_string(), Value::Float(y)),
    ])
}

fn point3d_value(x: f64, y: f64, z: f64) -> Value {
    Value::Record(vec![
        ("x".to_string(), Value::Float(x)),
        ("y".to_string(), Value::Float(y)),
        ("z".to_string(), Value::Float(z)),
    ])
}

fn record_float(fields: &[(String, Value)], name: &str) -> Result<f64, String> {
    fields
        .iter()
        .find(|(n, _)| n == name)
        .ok_or_else(|| format!("record に field `{name}` がありません"))
        .and_then(|(_, v)| as_f64(v))
}

fn as_point3d(v: &Value) -> Result<(f64, f64, f64), String> {
    match v {
        Value::Record(fs) => Ok((
            record_float(fs, "x")?,
            record_float(fs, "y")?,
            record_float(fs, "z")?,
        )),
        _ => Err(format!("Point3D が期待されましたが {v} でした")),
    }
}

fn as_point2d(v: &Value) -> Result<(f64, f64), String> {
    match v {
        Value::Record(fs) => Ok((record_float(fs, "x")?, record_float(fs, "y")?)),
        _ => Err(format!("Point2D が期待されましたが {v} でした")),
    }
}

fn as_shape2d(v: &Value) -> Result<Model2D, String> {
    match v {
        Value::Shape2D(m) => Ok(m.clone()),
        _ => Err(format!("Shape2D が期待されましたが {v} でした")),
    }
}

fn as_list(v: &Value) -> Result<&[Value], String> {
    match v {
        Value::List(vs) => Ok(vs.as_slice()),
        _ => Err(format!("List が期待されましたが {v} でした")),
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
        .add("tetrahedron", 0, |_| {
            Ok(Value::Shape3D(Model3D::Tetrahedron))
        })
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
            Ok(point3d_value(
                as_f64(&args[0])?,
                as_f64(&args[1])?,
                as_f64(&args[2])?,
            ))
        })
        .add("p2", 2, |args| {
            Ok(point2d_value(as_f64(&args[0])?, as_f64(&args[1])?))
        })
        // -- 数値変換
        .add("fromInt", 1, |args| {
            Ok(Value::Float(as_int(&args[0])? as f64))
        })
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
                    return Err(format!("intersect: 空の range ({lo}..{hi} となりました)"));
                }
                Ok(Value::Range {
                    lo,
                    hi,
                    is_int: *ii1,
                })
            }
            _ => Err("intersect: Range a を 2 つ要求".to_string()),
        })
        .add("stl", 1, |args| match &args[0] {
            Value::String(path) => Ok(Value::Opaque(
                "Stl".to_string(),
                vec![Value::String(path.clone())],
            )),
            _ => Err("stl: 文字列パス を要求".to_string()),
        })
        // -- 2D primitives
        .add("circle", 1, |args| {
            let r = as_f64(&args[0])?;
            // 32 角形で近似。GUI 描画でほとんど円に見える程度。
            let n = 32;
            let mut pts: Vec<(f64, f64)> = Vec::with_capacity(n);
            for i in 0..n {
                let t = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                pts.push((r * t.cos(), r * t.sin()));
            }
            Ok(Value::Shape2D(Model2D::Polygon(pts)))
        })
        .add("empty_2d", 0, |_| Ok(Value::Shape2D(Model2D::Empty2D)))
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
                // `place` 経由は当面 Empty 返却。`extrude_xy` / `_yz` / `_xz` を使う。
                Ok(Value::Shape3D(Model3D::Empty))
            }
            _ => Err("linear_extrude: PlacedShape2D を要求".to_string()),
        })
        // -- 2D ポリゴン + 平面別 extrude
        .add("polygon", 1, |args| {
            let points = as_list(&args[0])?;
            let mut pts: Vec<(f64, f64)> = Vec::with_capacity(points.len());
            for p in points {
                pts.push(as_point2d(p)?);
            }
            Ok(Value::Shape2D(Model2D::Polygon(pts)))
        })
        .add("extrude_xy", 2, |args| {
            Ok(Value::Shape3D(Model3D::LinearExtrude {
                profile: as_shape2d(&args[1])?,
                plane: Plane3D::XY,
                height: as_f64(&args[0])?,
            }))
        })
        .add("extrude_yz", 2, |args| {
            Ok(Value::Shape3D(Model3D::LinearExtrude {
                profile: as_shape2d(&args[1])?,
                plane: Plane3D::YZ,
                height: as_f64(&args[0])?,
            }))
        })
        .add("extrude_xz", 2, |args| {
            Ok(Value::Shape3D(Model3D::LinearExtrude {
                profile: as_shape2d(&args[1])?,
                plane: Plane3D::XZ,
                height: as_f64(&args[0])?,
            }))
        })
        // -- 2D CSG
        .add("union2d", 2, |args| {
            Ok(Value::Shape2D(Model2D::Union2D(
                Box::new(as_shape2d(&args[0])?),
                Box::new(as_shape2d(&args[1])?),
            )))
        })
        .add("diff2d", 2, |args| {
            Ok(Value::Shape2D(Model2D::Diff2D(
                Box::new(as_shape2d(&args[0])?),
                Box::new(as_shape2d(&args[1])?),
            )))
        })
        .add("intersect2d", 2, |args| {
            Ok(Value::Shape2D(Model2D::Intersect2D(
                Box::new(as_shape2d(&args[0])?),
                Box::new(as_shape2d(&args[1])?),
            )))
        })
        // -- revolve
        .add("revolve_xy", 2, |args| {
            Ok(Value::Shape3D(Model3D::Revolve {
                profile: as_shape2d(&args[1])?,
                plane: Plane3D::XY,
                degrees: as_f64(&args[0])?,
            }))
        })
        .add("revolve_yz", 2, |args| {
            Ok(Value::Shape3D(Model3D::Revolve {
                profile: as_shape2d(&args[1])?,
                plane: Plane3D::YZ,
                degrees: as_f64(&args[0])?,
            }))
        })
        .add("revolve_xz", 2, |args| {
            Ok(Value::Shape3D(Model3D::Revolve {
                profile: as_shape2d(&args[1])?,
                plane: Plane3D::XZ,
                degrees: as_f64(&args[0])?,
            }))
        })
        // -- complex_extrude
        .add("complex_extrude_xy", 5, |args| {
            Ok(Value::Shape3D(Model3D::ComplexExtrude {
                profile: as_shape2d(&args[4])?,
                plane: Plane3D::XY,
                height: as_f64(&args[0])?,
                twist: as_f64(&args[1])?,
                scale_x: as_f64(&args[2])?,
                scale_y: as_f64(&args[3])?,
            }))
        })
        // -- sweep_extrude (XY 平面 profile + 3D path)
        .add("sweep_extrude_xy", 2, |args| {
            let path_v = as_list(&args[0])?;
            let mut path: Vec<(f64, f64, f64)> = Vec::with_capacity(path_v.len());
            for p in path_v {
                path.push(as_point3d(p)?);
            }
            Ok(Value::Shape3D(Model3D::SweepExtrude {
                profile: as_shape2d(&args[1])?,
                plane: Plane3D::XY,
                path,
            }))
        })
        // -- center3d / center2d: Shape の AABB 中心 Point を返す。
        //    Manifold を実評価するため STL 検索パスが必要 (`INCLUDE_PATHS`)。
        .add("center3d", 1, |args| {
            let model = as_shape3d(&args[0])?;
            let paths = INCLUDE_PATHS.with(|p| p.borrow().clone());
            let (cx, cy, cz) =
                crate::runtime::manifold_bridge::bbox_center_3d(&model, &paths)
                    .map_err(|e| format!("center3d: {e}"))?;
            Ok(point3d_value(cx, cy, cz))
        })
        .add("center2d", 1, |args| {
            let model = as_shape2d(&args[0])?;
            let (cx, cy) = crate::runtime::manifold_bridge::bbox_center_2d(&model)
                .map_err(|e| format!("center2d: {e}"))?;
            Ok(point2d_value(cx, cy))
        })
        // -- 2D translate: src 点を dst 点に運ぶ。
        .add("translate2d", 3, |args| {
            Ok(Value::Shape2D(Model2D::Translate2D {
                src: as_point2d(&args[0])?,
                dst: as_point2d(&args[1])?,
                shape: Box::new(as_shape2d(&args[2])?),
            }))
        })
        // -- control points: 第 1 引数を name (String) として保持しつつ第 2 引数の Point
        //    を返す。GUI 側は MainOutput.controls から拾って描画 + ドラッグ override。
        //    ここでは「裸の Point2D/3D」をそのまま返す簡易実装 (GUI 側で Ctor として
        //    詰めなおす)。
        .add("control3d", 2, |args| {
            let name = as_string(&args[0])?;
            let default = as_point3d(&args[1])?;
            let current = CONTROL_OVERRIDES.with(|c| {
                c.borrow()
                    .get(&name)
                    .copied()
                    .unwrap_or([default.0, default.1, default.2])
            });
            RECORDED_CONTROLS.with(|r| r.borrow_mut().push((name, current)));
            Ok(point3d_value(current[0], current[1], current[2]))
        })
        .add("control2d", 2, |args| {
            let name = as_string(&args[0])?;
            let default = as_point2d(&args[1])?;
            let current = CONTROL_OVERRIDES.with(|c| {
                c.borrow()
                    .get(&name)
                    .copied()
                    .unwrap_or([default.0, default.1, 0.0])
            });
            // 2D は z を 0 として記録する (GUI 側で扱いを分岐)。
            RECORDED_CONTROLS.with(|r| r.borrow_mut().push((name, current)));
            Ok(point2d_value(current[0], current[1]))
        })
        // -- Bezier サンプリング
        .add("bezier_quad", 4, |args| {
            let p0 = as_point2d(&args[0])?;
            let c = as_point2d(&args[1])?;
            let p1 = as_point2d(&args[2])?;
            let n = as_int(&args[3])?.max(2) as usize;
            let mut pts: Vec<Value> = Vec::with_capacity(n);
            // start (t=0) は含めず、segments 個に分割した点列 (t=1..=n-1) と end (t=1) を返す。
            // start を含めると polygon に append したときに duplicate が出やすいため。
            for i in 1..=n {
                let t = i as f64 / n as f64;
                let mt = 1.0 - t;
                let x = mt * mt * p0.0 + 2.0 * mt * t * c.0 + t * t * p1.0;
                let y = mt * mt * p0.1 + 2.0 * mt * t * c.1 + t * t * p1.1;
                pts.push(point2d_value(x, y));
            }
            Ok(Value::List(pts))
        })
        .add("bezier_cubic", 5, |args| {
            let p0 = as_point2d(&args[0])?;
            let c1 = as_point2d(&args[1])?;
            let c2 = as_point2d(&args[2])?;
            let p1 = as_point2d(&args[3])?;
            let n = as_int(&args[4])?.max(2) as usize;
            let mut pts: Vec<Value> = Vec::with_capacity(n);
            for i in 1..=n {
                let t = i as f64 / n as f64;
                let mt = 1.0 - t;
                let b0 = mt * mt * mt;
                let b1 = 3.0 * mt * mt * t;
                let b2 = 3.0 * mt * t * t;
                let b3 = t * t * t;
                let x = b0 * p0.0 + b1 * c1.0 + b2 * c2.0 + b3 * p1.0;
                let y = b0 * p0.1 + b1 * c1.1 + b2 * c2.1 + b3 * p1.1;
                pts.push(point2d_value(x, y));
            }
            Ok(Value::List(pts))
        })
}
