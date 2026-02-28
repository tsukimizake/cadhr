//! Prolog Term -> manifold-rs Manifold 変換層
//!
//! Term（書き換え後の項）を ManifoldExpr 中間表現に変換し、
//! それを manifold-rs の Manifold オブジェクトに評価する。

use crate::parse::{ArithOp, FixedPoint, SrcSpan, Term, term_as_fixed_point};
use cadhr_lang_macros::define_manifold_expr;
use manifold_rs::{Manifold, Mesh};
use std::fmt;
use std::path::{Path, PathBuf};
use std::str::FromStr;

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

// 以下を生成:
// - pub enum ManifoldExpr { Cube{..}, Sphere{..}, ... }  (@no_variant付きは除外)
// - pub enum ManifoldTag { Cube, Sphere, ..., Point, ... } (全エントリ)
// - impl FromStr for ManifoldTag  (functor文字列 → タグ。@nameがあればその名前を使用)
// - pub const BUILTIN_FUNCTORS: &[(&str, &[usize])]  (functor名 → 許容arity一覧)
// - pub fn is_builtin_functor(functor: &str) -> bool
define_manifold_expr! {
    Cube { x: TrackedF64, y: TrackedF64, z: TrackedF64 };
    @also_arity(1)
    Sphere { radius: TrackedF64, segments: u32 };
    @also_arity(2)
    Cylinder { radius: TrackedF64, height: TrackedF64, segments: u32 };
    Tetrahedron;
    Union(Box<ManifoldExpr>, Box<ManifoldExpr>);
    Difference(Box<ManifoldExpr>, Box<ManifoldExpr>);
    Intersection(Box<ManifoldExpr>, Box<ManifoldExpr>);
    Translate { expr: Box<ManifoldExpr>, x: TrackedF64, y: TrackedF64, z: TrackedF64 };
    Scale { expr: Box<ManifoldExpr>, x: TrackedF64, y: TrackedF64, z: TrackedF64 };
    Rotate { expr: Box<ManifoldExpr>, x: TrackedF64, y: TrackedF64, z: TrackedF64 };
    @name("p") @no_variant
    Point { _x: TrackedF64, _y: TrackedF64 };
    Polygon { points: Vec<f64> };
    @also_arity(1)
    Circle { radius: TrackedF64, segments: u32 };
    Extrude { profile: Box<ManifoldExpr>, height: TrackedF64 };
    @also_arity(2)
    Revolve { profile: Box<ManifoldExpr>, degrees: TrackedF64, segments: u32 };
    Polyhedron { points: Vec<f64>, faces: Vec<Vec<u32>> };
    Stl { path: String };
    @also_arity(3) @no_variant
    Control { x: TrackedF64, y: TrackedF64, z: TrackedF64, name: String };
}

pub fn is_builtin_functor_with_arity(functor: &str, arity: usize) -> bool {
    BUILTIN_FUNCTORS
        .iter()
        .any(|(name, arities)| *name == functor && arities.contains(&arity))
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
    /// I/Oエラー（ファイル読み込み失敗など）
    IoError { functor: String, message: String },
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
            ConversionError::IoError { functor, message } => {
                write!(f, "I/O error in {}: {}", functor, message)
            }
        }
    }
}

impl std::error::Error for ConversionError {}

// ============================================================
// ControlPoint: ドラッグ可能なコントロールポイント
// ============================================================

#[derive(Debug, Clone)]
pub struct ControlPoint {
    pub x: TrackedF64,
    pub y: TrackedF64,
    pub z: TrackedF64,
    pub name: Option<String>,
    /// リネーム前の変数名 (サフィックス _\d+ 除去済み)。x,y,zそれぞれに対応。
    pub var_names: [Option<String>; 3],
}

/// TermからTrackedF64を抽出する。Number, AnnotatedVar(デフォルト値あり), Var(0にフォールバック)に対応。
fn term_to_tracked_f64(term: &Term) -> Option<TrackedF64> {
    if let Some((fp, span)) = term_as_fixed_point(term) {
        return Some(TrackedF64 {
            value: fp.to_f64(),
            source_span: span,
        });
    }
    match term {
        Term::AnnotatedVar {
            default_value: Some(fp),
            span,
            ..
        } => Some(TrackedF64 {
            value: fp.to_f64(),
            source_span: *span,
        }),
        Term::AnnotatedVar { span, .. } => Some(TrackedF64 {
            value: 0.0,
            source_span: *span,
        }),
        Term::Var { .. } => Some(TrackedF64 {
            value: 0.0,
            source_span: None,
        }),
        _ => None,
    }
}

/// Var/AnnotatedVarから変数名を取り出す
fn var_name(term: &Term) -> Option<&str> {
    match term {
        Term::Var { name } | Term::AnnotatedVar { name, .. } => Some(name),
        _ => None,
    }
}

/// control(X,Y,Z) / control(X,Y,Z,Name) のTermを抽出し、残りのTermを返す。
/// control座標がVarの場合、同名の変数を残りのtermsからも置換する。
pub fn extract_control_points(terms: &mut Vec<Term>) -> Vec<ControlPoint> {
    let mut control_points = Vec::new();
    // Var名 → 置換するNumber値 のマッピング
    let mut var_substitutions: Vec<(String, FixedPoint)> = Vec::new();

    terms.retain(|term| {
        if let Term::Struct { functor, args } = term {
            if functor == "control" && (args.len() == 3 || args.len() == 4) {
                let x = term_to_tracked_f64(&args[0]);
                let y = term_to_tracked_f64(&args[1]);
                let z = term_to_tracked_f64(&args[2]);
                let name = if args.len() == 4 {
                    match &args[3] {
                        Term::StringLit { value } => Some(value.clone()),
                        Term::Struct { functor, args } if args.is_empty() => {
                            Some(functor.clone())
                        }
                        _ => None,
                    }
                } else {
                    None
                };
                if let (Some(x), Some(y), Some(z)) = (x, y, z) {
                    let mut vnames: [Option<String>; 3] = [None, None, None];
                    for (idx, (arg, val)) in [(&args[0], x.value), (&args[1], y.value), (&args[2], z.value)].iter().enumerate() {
                        if let Some(vname) = var_name(arg) {
                            vnames[idx] = Some(strip_rename_suffix(vname).to_string());
                            var_substitutions.push((vname.to_string(), FixedPoint::from_hundredths((val * 100.0).round() as i64)));
                        }
                    }
                    control_points.push(ControlPoint { x, y, z, name, var_names: vnames });
                    return false;
                }
            }
        }
        true
    });

    // 残りのtermsに変数置換を適用し、算術式を評価
    if !var_substitutions.is_empty() {
        for term in terms.iter_mut() {
            substitute_vars(term, &var_substitutions);
            crate::term_rewrite::eval_arith_in_place(term);
        }
    }

    control_points
}

fn substitute_vars(term: &mut Term, subs: &[(String, FixedPoint)]) {
    match term {
        Term::Var { name } | Term::AnnotatedVar { name, .. } => {
            if let Some((_, val)) = subs.iter().find(|(n, _)| n == name) {
                *term = Term::Number { value: *val };
            }
        }
        Term::Struct { args, .. } => {
            for arg in args.iter_mut() {
                substitute_vars(arg, subs);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            substitute_vars(left, subs);
            substitute_vars(right, subs);
        }
        Term::List { items, tail } => {
            for item in items.iter_mut() {
                substitute_vars(item, subs);
            }
            if let Some(t) = tail {
                substitute_vars(t, subs);
            }
        }
        _ => {}
    }
}

/// リネームサフィックス `_\d+` を除去して元の変数名を返す
fn strip_rename_suffix(name: &str) -> &str {
    if let Some(pos) = name.rfind('_') {
        let suffix = &name[pos + 1..];
        if pos > 0 && !suffix.is_empty() && suffix.chars().all(|c| c.is_ascii_digit()) {
            return &name[..pos];
        }
    }
    name
}

/// override mapに基づいてterms中のVar/AnnotatedVarを置換する
pub fn apply_var_overrides(terms: &mut Vec<Term>, overrides: &std::collections::HashMap<String, f64>) {
    if overrides.is_empty() {
        return;
    }
    for term in terms.iter_mut() {
        apply_var_overrides_to_term(term, overrides);
        crate::term_rewrite::eval_arith_in_place(term);
    }
}

fn apply_var_overrides_to_term(term: &mut Term, overrides: &std::collections::HashMap<String, f64>) {
    match term {
        Term::Var { name } | Term::AnnotatedVar { name, .. } => {
            let base = strip_rename_suffix(name);
            if let Some(&val) = overrides.get(base) {
                *term = Term::Number {
                    value: FixedPoint::from_hundredths((val * 100.0).round() as i64),
                };
            }
        }
        Term::Struct { args, .. } => {
            for arg in args.iter_mut() {
                apply_var_overrides_to_term(arg, overrides);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            apply_var_overrides_to_term(left, overrides);
            apply_var_overrides_to_term(right, overrides);
        }
        Term::List { items, tail } => {
            for item in items.iter_mut() {
                apply_var_overrides_to_term(item, overrides);
            }
            if let Some(t) = tail {
                apply_var_overrides_to_term(t, overrides);
            }
        }
        _ => {}
    }
}

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

    fn string(&self, i: usize) -> Result<String, ConversionError> {
        match &self.args[i] {
            Term::StringLit { value } => Ok(value.clone()),
            _ => Err(ConversionError::TypeMismatch {
                functor: self.functor.to_string(),
                arg_index: i,
                expected: "string",
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
        let tag = ManifoldTag::from_str(functor)
            .map_err(|_| ConversionError::UnknownPrimitive(functor.to_string()))?;

        match tag {
            ManifoldTag::Cube if a.len() == 3 => Ok(ManifoldExpr::Cube {
                x: a.tracked_f64(0)?,
                y: a.tracked_f64(1)?,
                z: a.tracked_f64(2)?,
            }),
            ManifoldTag::Cube => Err(a.arity_error("3")),

            ManifoldTag::Sphere if a.len() == 1 => Ok(ManifoldExpr::Sphere {
                radius: a.tracked_f64(0)?,
                segments: DEFAULT_SEGMENTS,
            }),
            ManifoldTag::Sphere if a.len() == 2 => Ok(ManifoldExpr::Sphere {
                radius: a.tracked_f64(0)?,
                segments: a.u32(1)?,
            }),
            ManifoldTag::Sphere => Err(a.arity_error("1 or 2")),

            ManifoldTag::Cylinder if a.len() == 2 => Ok(ManifoldExpr::Cylinder {
                radius: a.tracked_f64(0)?,
                height: a.tracked_f64(1)?,
                segments: DEFAULT_SEGMENTS,
            }),
            ManifoldTag::Cylinder if a.len() == 3 => Ok(ManifoldExpr::Cylinder {
                radius: a.tracked_f64(0)?,
                height: a.tracked_f64(1)?,
                segments: a.u32(2)?,
            }),
            ManifoldTag::Cylinder => Err(a.arity_error("2 or 3")),

            ManifoldTag::Tetrahedron if a.len() == 0 => Ok(ManifoldExpr::Tetrahedron),
            ManifoldTag::Tetrahedron => Err(a.arity_error("0")),

            ManifoldTag::Union if a.len() == 2 => Ok(ManifoldExpr::Union(
                Box::new(a.term(0)?),
                Box::new(a.term(1)?),
            )),
            ManifoldTag::Union => Err(a.arity_error("2")),

            ManifoldTag::Difference if a.len() == 2 => Ok(ManifoldExpr::Difference(
                Box::new(a.term(0)?),
                Box::new(a.term(1)?),
            )),
            ManifoldTag::Difference => Err(a.arity_error("2")),

            ManifoldTag::Intersection if a.len() == 2 => Ok(ManifoldExpr::Intersection(
                Box::new(a.term(0)?),
                Box::new(a.term(1)?),
            )),
            ManifoldTag::Intersection => Err(a.arity_error("2")),

            ManifoldTag::Translate if a.len() == 4 => Ok(ManifoldExpr::Translate {
                expr: Box::new(a.term(0)?),
                x: a.tracked_f64(1)?,
                y: a.tracked_f64(2)?,
                z: a.tracked_f64(3)?,
            }),
            ManifoldTag::Translate => Err(a.arity_error("4")),

            ManifoldTag::Scale if a.len() == 4 => Ok(ManifoldExpr::Scale {
                expr: Box::new(a.term(0)?),
                x: a.tracked_f64(1)?,
                y: a.tracked_f64(2)?,
                z: a.tracked_f64(3)?,
            }),
            ManifoldTag::Scale => Err(a.arity_error("4")),

            ManifoldTag::Rotate if a.len() == 4 => Ok(ManifoldExpr::Rotate {
                expr: Box::new(a.term(0)?),
                x: a.tracked_f64(1)?,
                y: a.tracked_f64(2)?,
                z: a.tracked_f64(3)?,
            }),
            ManifoldTag::Rotate => Err(a.arity_error("4")),

            ManifoldTag::Point => Err(ConversionError::UnknownPrimitive(
                "p is a data constructor, not a shape primitive".to_string(),
            )),

            ManifoldTag::Polygon if a.len() == 1 => {
                let points = extract_polygon_points(&a.args[0], a.functor)?;
                Ok(ManifoldExpr::Polygon { points })
            }
            ManifoldTag::Polygon => Err(a.arity_error("1")),

            ManifoldTag::Circle if a.len() == 1 => Ok(ManifoldExpr::Circle {
                radius: a.tracked_f64(0)?,
                segments: DEFAULT_SEGMENTS,
            }),
            ManifoldTag::Circle if a.len() == 2 => Ok(ManifoldExpr::Circle {
                radius: a.tracked_f64(0)?,
                segments: a.u32(1)?,
            }),
            ManifoldTag::Circle => Err(a.arity_error("1 or 2")),

            ManifoldTag::Extrude if a.len() == 2 => Ok(ManifoldExpr::Extrude {
                profile: Box::new(a.term(0)?),
                height: a.tracked_f64(1)?,
            }),
            ManifoldTag::Extrude => Err(a.arity_error("2")),

            ManifoldTag::Revolve if a.len() == 2 => Ok(ManifoldExpr::Revolve {
                profile: Box::new(a.term(0)?),
                degrees: a.tracked_f64(1)?,
                segments: DEFAULT_SEGMENTS,
            }),
            ManifoldTag::Revolve if a.len() == 3 => Ok(ManifoldExpr::Revolve {
                profile: Box::new(a.term(0)?),
                degrees: a.tracked_f64(1)?,
                segments: a.u32(2)?,
            }),
            ManifoldTag::Revolve => Err(a.arity_error("2 or 3")),

            ManifoldTag::Polyhedron if a.len() == 2 => {
                let points = extract_polyhedron_points(&a.args[0], a.functor)?;
                let faces = extract_polyhedron_faces(&a.args[1], a.functor)?;
                Ok(ManifoldExpr::Polyhedron { points, faces })
            }
            ManifoldTag::Polyhedron => Err(a.arity_error("2")),

            ManifoldTag::Stl if a.len() == 1 => {
                let path = a.string(0)?;
                Ok(ManifoldExpr::Stl { path })
            }
            ManifoldTag::Stl => Err(a.arity_error("1")),

            ManifoldTag::Control => Err(ConversionError::UnknownPrimitive(
                "control is a data constructor, not a shape primitive".to_string(),
            )),
        }
    }

    /// ManifoldExpr を manifold-rs の Manifold に評価
    pub fn evaluate(&self, include_paths: &[PathBuf]) -> Result<Manifold, ConversionError> {
        match self {
            ManifoldExpr::Cube { x, y, z } => Ok(Manifold::cube(x.value, y.value, z.value)),
            ManifoldExpr::Sphere { radius, segments } => {
                Ok(Manifold::sphere(radius.value, *segments))
            }
            ManifoldExpr::Cylinder {
                radius,
                height,
                segments,
            } => Ok(Manifold::cylinder(
                radius.value,
                radius.value,
                height.value,
                *segments,
            )),
            ManifoldExpr::Tetrahedron => Ok(Manifold::tetrahedron()),

            ManifoldExpr::Union(a, b) => Ok(a
                .evaluate(include_paths)?
                .union(&b.evaluate(include_paths)?)),
            ManifoldExpr::Difference(a, b) => Ok(a
                .evaluate(include_paths)?
                .difference(&b.evaluate(include_paths)?)),
            ManifoldExpr::Intersection(a, b) => Ok(a
                .evaluate(include_paths)?
                .intersection(&b.evaluate(include_paths)?)),

            ManifoldExpr::Translate { expr, x, y, z } => Ok(expr
                .evaluate(include_paths)?
                .translate(x.value, y.value, z.value)),
            ManifoldExpr::Scale { expr, x, y, z } => Ok(expr
                .evaluate(include_paths)?
                .scale(x.value, y.value, z.value)),
            ManifoldExpr::Rotate { expr, x, y, z } => Ok(expr
                .evaluate(include_paths)?
                .rotate(x.value, y.value, z.value)),

            ManifoldExpr::Polygon { points } => {
                Ok(Manifold::extrude(&[points], 0.001, 0, 0.0, 1.0, 1.0))
            }
            ManifoldExpr::Circle { .. } => {
                let data = self
                    .to_polygon_data()
                    .ok_or_else(|| ConversionError::TypeMismatch {
                        functor: "circle".to_string(),
                        arg_index: 0,
                        expected: "polygon data",
                    })?;
                Ok(Manifold::extrude(&[&data], 0.001, 0, 0.0, 1.0, 1.0))
            }

            ManifoldExpr::Extrude { profile, height } => {
                let data =
                    profile
                        .to_polygon_data()
                        .ok_or_else(|| ConversionError::TypeMismatch {
                            functor: "extrude".to_string(),
                            arg_index: 0,
                            expected: "polygon data",
                        })?;
                Ok(Manifold::extrude(&[&data], height.value, 0, 0.0, 1.0, 1.0))
            }
            ManifoldExpr::Revolve {
                profile,
                degrees,
                segments,
            } => {
                let data =
                    profile
                        .to_polygon_data()
                        .ok_or_else(|| ConversionError::TypeMismatch {
                            functor: "revolve".to_string(),
                            arg_index: 0,
                            expected: "polygon data",
                        })?;
                Ok(Manifold::revolve(&[&data], *segments, degrees.value))
            }

            ManifoldExpr::Polyhedron { points, faces } => {
                let verts: Vec<f32> = points.iter().map(|&v| v as f32).collect();
                let tri_indices: Vec<u32> = faces
                    .iter()
                    .flat_map(|face| {
                        (1..face.len() - 1).flat_map(move |i| {
                            vec![face[0], face[i as usize], face[i as usize + 1]]
                        })
                    })
                    .collect();
                let mesh = Mesh::new(&verts, &tri_indices);
                Ok(Manifold::from_mesh(mesh))
            }

            ManifoldExpr::Stl { path } => {
                let raw = Path::new(path);
                let resolved = if raw.is_absolute() {
                    PathBuf::from(path)
                } else {
                    include_paths
                        .iter()
                        .map(|dir| dir.join(raw))
                        .find(|p| p.exists())
                        .unwrap_or_else(|| PathBuf::from(path))
                };
                let mut file = std::fs::OpenOptions::new()
                    .read(true)
                    .open(&resolved)
                    .map_err(|e| ConversionError::IoError {
                        functor: "stl".into(),
                        message: format!("{}: {}", resolved.display(), e),
                    })?;
                let stl = stl_io::read_stl(&mut file).map_err(|e| ConversionError::IoError {
                    functor: "stl".into(),
                    message: format!("{}: {}", resolved.display(), e),
                })?;
                let verts: Vec<f32> = stl
                    .vertices
                    .iter()
                    .flat_map(|v| [v[0], v[1], v[2]])
                    .collect();
                let indices: Vec<u32> = stl
                    .faces
                    .iter()
                    .flat_map(|f| f.vertices.iter().map(|&i| i as u32))
                    .collect();
                let mesh = Mesh::new(&verts, &indices);
                Ok(Manifold::from_mesh(mesh))
            }
        }
    }

    /// ManifoldExpr を Mesh に変換（法線計算込み）
    pub fn to_mesh(&self, include_paths: &[PathBuf]) -> Result<Mesh, ConversionError> {
        let manifold = self.evaluate(include_paths)?;
        let with_normals = manifold.calculate_normals(0, 30.0);
        Ok(with_normals.to_mesh())
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

fn build_evaluated_node(
    expr: &ManifoldExpr,
    include_paths: &[PathBuf],
) -> Result<EvaluatedNode, ConversionError> {
    let manifold = expr.evaluate(include_paths)?;
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
            vec![
                build_evaluated_node(a, include_paths)?,
                build_evaluated_node(b, include_paths)?,
            ]
        }
        ManifoldExpr::Translate { expr: e, .. }
        | ManifoldExpr::Scale { expr: e, .. }
        | ManifoldExpr::Rotate { expr: e, .. }
        | ManifoldExpr::Extrude { profile: e, .. }
        | ManifoldExpr::Revolve { profile: e, .. } => {
            vec![build_evaluated_node(e, include_paths)?]
        }
        _ => vec![],
    };

    Ok(EvaluatedNode {
        expr: expr.clone(),
        mesh_verts,
        mesh_indices,
        aabb_min,
        aabb_max,
        children,
    })
}

/// 複数のTermからMeshとEvaluatedNodeツリーを生成する
pub fn generate_mesh_and_tree_from_terms(
    terms: &[Term],
    include_paths: &[PathBuf],
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

    let nodes: Vec<EvaluatedNode> = exprs
        .iter()
        .map(|e| build_evaluated_node(e, include_paths))
        .collect::<Result<Vec<_>, _>>()?;

    let manifold = exprs
        .iter()
        .map(|e| e.evaluate(include_paths))
        .reduce(|acc, m| Ok(acc?.union(&m?)))
        .unwrap()?;

    let with_normals = manifold.calculate_normals(0, 30.0);
    Ok((with_normals.to_mesh(), nodes))
}

/// 複数のTermからMeshを生成する（全てをunionする）
pub fn generate_mesh_from_terms(
    terms: &[Term],
    include_paths: &[PathBuf],
) -> Result<Mesh, ConversionError> {
    if terms.is_empty() {
        return Err(ConversionError::UnknownPrimitive(
            "empty term list".to_string(),
        ));
    }

    let exprs: Vec<ManifoldExpr> = terms
        .iter()
        .map(ManifoldExpr::from_term)
        .collect::<Result<Vec<_>, _>>()?;

    let manifold = exprs
        .iter()
        .map(|e| e.evaluate(include_paths))
        .reduce(|acc, m| Ok(acc?.union(&m?)))
        .unwrap()?;

    let with_normals = manifold.calculate_normals(0, 30.0);
    Ok(with_normals.to_mesh())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{number_int, string_lit, struc, var};

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
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_extrude_evaluate() {
        let polygon = make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]);
        let term = struc("extrude".into(), vec![polygon, number_int(3)]);
        let expr = ManifoldExpr::from_term(&term).unwrap();
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_stl_conversion() {
        use stl_io::{Normal, Triangle, Vertex};

        let v0 = Vertex::new([0.0, 0.0, 0.0]);
        let v1 = Vertex::new([1.0, 0.0, 0.0]);
        let v2 = Vertex::new([0.0, 1.0, 0.0]);
        let v3 = Vertex::new([0.0, 0.0, 1.0]);
        let tris = vec![
            Triangle { normal: Normal::new([0.0, 0.0, -1.0]), vertices: [v0, v2, v1] },
            Triangle { normal: Normal::new([0.0, -1.0, 0.0]), vertices: [v0, v1, v3] },
            Triangle { normal: Normal::new([-1.0, 0.0, 0.0]), vertices: [v0, v3, v2] },
            Triangle { normal: Normal::new([1.0, 1.0, 1.0]),  vertices: [v1, v2, v3] },
        ];

        let dir = std::env::temp_dir().join("cadhr_test_stl");
        std::fs::create_dir_all(&dir).unwrap();
        let stl_path = dir.join("test.stl");
        {
            let mut file = std::fs::File::create(&stl_path).unwrap();
            stl_io::write_stl(&mut file, tris.iter()).unwrap();
        }

        let term = struc(
            "stl".into(),
            vec![string_lit(stl_path.to_str().unwrap().into())],
        );
        let expr = ManifoldExpr::from_term(&term).unwrap();
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn test_extract_control_points() {
        let cube = struc(
            "cube".into(),
            vec![number_int(10), number_int(20), number_int(30)],
        );
        let cp1 = struc(
            "control".into(),
            vec![number_int(1), number_int(2), number_int(3)],
        );
        let cp2 = struc(
            "control".into(),
            vec![
                number_int(4),
                number_int(5),
                number_int(6),
                string_lit("origin".into()),
            ],
        );
        let mut terms = vec![cube, cp1, cp2];
        let cps = extract_control_points(&mut terms);
        assert_eq!(terms.len(), 1); // cube remains
        assert_eq!(cps.len(), 2);
        assert_eq!(cps[0].x.value, 1.0);
        assert_eq!(cps[0].y.value, 2.0);
        assert_eq!(cps[0].z.value, 3.0);
        assert!(cps[0].name.is_none());
        assert_eq!(cps[1].x.value, 4.0);
        assert_eq!(cps[1].name.as_deref(), Some("origin"));
    }

    #[test]
    fn test_extract_control_points_with_var() {
        let cube = struc(
            "cube".into(),
            vec![number_int(10), number_int(20), number_int(30)],
        );
        let cp = struc(
            "control".into(),
            vec![var("X".into()), number_int(0), number_int(0)],
        );
        let mut terms = vec![cube, cp];
        let cps = extract_control_points(&mut terms);
        assert_eq!(terms.len(), 1);
        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].x.value, 0.0); // Varは0にフォールバック
        assert_eq!(cps[0].var_names[0].as_deref(), Some("X"));
        assert!(cps[0].var_names[1].is_none());
        assert!(cps[0].var_names[2].is_none());
    }

    #[test]
    fn test_control_shared_var_with_geometry() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- extrude(polygon([p(0, 0), p(0, 40), p(30, 0)]), X@10), control(X, 0, 0, \"width\")."
        ).unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let mut resolved = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved);

        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].name.as_deref(), Some("width"));
        // X@10 のデフォルト値がcontrolにも伝播
        assert_eq!(cps[0].x.value, 10.0);
        // 残りのgeometryでメッシュ生成が成功する
        assert_eq!(resolved.len(), 1);
        let (mesh, _) = generate_mesh_and_tree_from_terms(&resolved, &[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_control_shared_var_without_default() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        // X@なし: controlのVar座標が0にフォールバックし、extrude側にも0が代入される
        let mut db = database(
            "main :- extrude(polygon([p(0, 0), p(0, 40), p(30, 0)]), X), control(X, -10, -10).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let mut resolved = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved);

        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].x.value, 0.0);
        assert_eq!(cps[0].y.value, -10.0);
        assert_eq!(cps[0].z.value, -10.0);
        // Xが0に代入されたのでメッシュ生成がエラーにならない（高さ0のextrudeは空メッシュ）
        assert_eq!(resolved.len(), 1);
        let _result = generate_mesh_and_tree_from_terms(&resolved, &[]).unwrap();
    }

    #[test]
    fn test_control_shared_var_in_arith_expr() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- polygon([p(0,0), p(0,40), p(30,0)]) |> extrude(X+1), control(X, -10, -10).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let mut resolved = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved);

        assert_eq!(cps.len(), 1);
        assert_eq!(resolved.len(), 1);
        let _result = generate_mesh_and_tree_from_terms(&resolved, &[]).unwrap();
    }

    #[test]
    fn test_control_is_builtin_functor() {
        assert!(is_builtin_functor("control"));
    }

    #[test]
    fn test_strip_rename_suffix() {
        assert_eq!(strip_rename_suffix("X_1"), "X");
        assert_eq!(strip_rename_suffix("Foo_123"), "Foo");
        assert_eq!(strip_rename_suffix("X"), "X");
        assert_eq!(strip_rename_suffix("X_abc"), "X_abc");
        assert_eq!(strip_rename_suffix("_1"), "_1"); // 空になるケースは除去しない（rfind('_')=0, prefix=""）
    }

    #[test]
    fn test_resolved_var_names_after_execute() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        // クエリの変数名を確認
        let mut db = database(
            "box(X) :- cube(X, X, X).\nmain :- box(10), box(20), control(X, 0, 0).",
        ).unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let resolved = execute(&mut db, q).unwrap();
        eprintln!("case1: {:?}", resolved);

        // 2つのcontrolが同じ変数名Xを使うケース
        let mut db2 = database(
            "main :- cube(X+Y, 20, 30), control(X, 0, 0), control(Y, 0, 0).",
        ).unwrap();
        let (_, q2) = parse_query("main.").unwrap();
        let resolved2 = execute(&mut db2, q2).unwrap();
        eprintln!("case2: {:?}", resolved2);

        // ルール経由で同名変数が複数スコープに存在するケース
        let mut db3 = database(
            "helper(X) :- cube(X, X, X), control(X, 0, 0).\nmain :- helper(10), helper(20).",
        ).unwrap();
        let (_, q3) = parse_query("main.").unwrap();
        let resolved3 = execute(&mut db3, q3).unwrap();
        eprintln!("case3: {:?}", resolved3);
    }

    #[test]
    fn test_apply_var_overrides() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;
        use std::collections::HashMap;

        let mut db = database(
            "main :- cube(X+10, 20, 30), control(X, 0, 0).",
        ).unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let mut resolved = execute(&mut db, q).unwrap();

        let mut overrides = HashMap::new();
        overrides.insert("X".to_string(), 5.0);
        apply_var_overrides(&mut resolved, &overrides);

        let cps = extract_control_points(&mut resolved);
        assert_eq!(cps.len(), 1);
        assert_eq!(resolved.len(), 1);
        // cube(X+10, 20, 30) where X=5 → cube(15, 20, 30)
        let (mesh, _) = generate_mesh_and_tree_from_terms(&resolved, &[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_apply_var_overrides_no_cross_contamination() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;
        use std::collections::HashMap;

        // box(X)が2回使われ、control(X,0,0)のXはクエリ由来
        // overrideはcontrolのXのみに影響し、box(10),box(20)は変わらないはず
        let mut db = database(
            "box(X) :- cube(X, X, X).\nmain :- box(10), box(20), control(X, 0, 0).",
        ).unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let mut resolved = execute(&mut db, q).unwrap();

        let mut overrides = HashMap::new();
        overrides.insert("X".to_string(), 5.0);
        apply_var_overrides(&mut resolved, &overrides);

        let cps = extract_control_points(&mut resolved);
        assert_eq!(cps.len(), 1);
        // box(10)→cube(10,10,10), box(20)→cube(20,20,20) が残るはず
        assert_eq!(resolved.len(), 2);
        let (mesh, _) = generate_mesh_and_tree_from_terms(&resolved, &[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }
}
