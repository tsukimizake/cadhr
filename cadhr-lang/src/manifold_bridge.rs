//! Prolog Term -> manifold-rs Manifold 変換層
//!
//! Term（書き換え後の項）を Model3D / Model2D 中間表現に変換し、
//! それを manifold-rs の Manifold オブジェクトに評価する。

use crate::parse::{ArithOp, Bound, SrcSpan, Term, term_as_number};
use crate::rational::Rational;
use manifold_rs::{Manifold, Mesh};
use std::fmt;
use std::path::{Path as StdPath, PathBuf};
use std::str::FromStr;

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

#[derive(Debug, Clone)]
pub enum Model3D {
    Cube {
        x: f64,
        y: f64,
        z: f64,
    },
    Sphere {
        radius: f64,
    },
    Cylinder {
        radius: f64,
        height: f64,
    },
    Tetrahedron,
    Union(Box<Model3D>, Box<Model3D>),
    Difference(Box<Model3D>, Box<Model3D>),
    Intersection(Box<Model3D>, Box<Model3D>),
    Hull(Box<Model3D>, Box<Model3D>),
    Translate {
        model: Box<Model3D>,
        x: f64,
        y: f64,
        z: f64,
    },
    Scale {
        model: Box<Model3D>,
        x: f64,
        y: f64,
        z: f64,
    },
    Rotate {
        model: Box<Model3D>,
        x: f64,
        y: f64,
        z: f64,
    },
    LinearExtrude {
        profile: PlacedSketch,
        height: f64,
    },
    ComplexExtrude {
        profile: PlacedSketch,
        height: f64,
        twist: f64,
        scale_x: f64,
        scale_y: f64,
    },
    Revolve {
        profile: PlacedSketch,
        degrees: f64,
    },
    Stl {
        path: String,
    },
    SweepExtrude {
        profile: PlacedSketch,
        path: Path,
    },
    Center3D {
        model: Box<Model3D>,
        x: f64,
        y: f64,
        z: f64,
    },
}

/// 閉じた2Dプロファイル。extrude / boolean / center2d の入力に使える。
/// `Sketch` の `points` は構築時に必ず CCW・閉路 (last != first 状態で保存) ・自己交差なし
/// が保証されている。rotateTo* は含まれないため、Union/Difference/Intersection/Center2D を
/// 自由に組み合わせて良い。3D操作 (linear_extrude等) は `PlacedSketch` を要求し、その内部
/// で `Model2D` が使われる。
#[derive(Debug, Clone)]
pub enum Model2D {
    Sketch {
        points: Vec<(f64, f64)>,
    },
    Circle {
        radius: f64,
    },
    Union(Box<Model2D>, Box<Model2D>),
    Difference(Box<Model2D>, Box<Model2D>),
    Intersection(Box<Model2D>, Box<Model2D>),
    Center2D {
        profile: Box<Model2D>,
        x: f64,
        y: f64,
    },
}

/// 開いた進行方向付き polyline。sweep_extrude の path 引数専用。
/// extrude / boolean / rotateTo* には渡せない (型システムで弾く)。
/// `points` は曲線セグメント (bezier_to) を tessellate した後の頂点列。
/// 閉路化・CCW化・自己交差チェックは行わない (開いた path を許容するため)。
#[derive(Debug, Clone)]
pub struct Path {
    pub points: Vec<(f64, f64)>,
}

/// 3D空間内のいずれかの平面に配置された2Dプロファイル。
/// linear_extrude / complex_extrude / revolve はこの型を受け取る。
/// プロファイル自身 (`Model2D`) は rotateTo* を含まないので「rotate後のスケッチに2D操作」は
/// 構築不可能 (型エラー) になる。
#[derive(Debug, Clone)]
pub struct PlacedSketch {
    pub plane: Plane3D,
    pub profile: Model2D,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Plane3D {
    XY,
    YZ,
    XZ,
}

const DEFAULT_SEGMENTS: u32 = 32;

pub const BUILTIN_FUNCTORS: &[(&str, &[usize])] = &[
    ("cube", &[3]),
    ("sphere", &[1, 2]),
    ("cylinder", &[2, 3]),
    ("tetrahedron", &[0]),
    ("union", &[2]),
    ("difference", &[2]),
    ("intersection", &[2]),
    ("hull", &[2]),
    ("translate", &[3]),
    ("scale", &[4]),
    ("rotate", &[4]),
    ("p", &[2, 3]),
    ("sketch", &[2]),
    ("rotateToXY", &[1]),
    ("rotateToYZ", &[1]),
    ("rotateToXZ", &[1]),
    ("circle", &[1, 2]),
    ("linear_extrude", &[2]),
    ("complex_extrude", &[5]),
    ("revolve", &[2, 3]),
    ("stl", &[1]),
    ("line_to", &[1]),
    ("bezier_to", &[2, 3]),
    ("path", &[2]),
    ("sweep_extrude", &[2]),
    ("control2d", &[1, 2]),
    ("control3d", &[1, 2]),
    ("center3d", &[2]),
    ("center2d", &[2]),
];

inventory::submit! {
    crate::term_processor::BuiltinFunctorSet {
        functors: BUILTIN_FUNCTORS,
        resolve_args: true,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FunctorTag {
    Cube,
    Sphere,
    Cylinder,
    Tetrahedron,
    Union,
    Difference,
    Intersection,
    Hull,
    Translate,
    Scale,
    Rotate,
    Point,
    Sketch,
    RotateToXY,
    RotateToYZ,
    RotateToXZ,
    Circle,
    LinearExtrude,
    ComplexExtrude,
    Revolve,
    Stl,
    LineTo,
    BezierTo,
    Path,
    SweepExtrude,
    Control2D,
    Control3D,
    Center3D,
    Center2D,
}

impl FromStr for FunctorTag {
    type Err = ();
    fn from_str(s: &str) -> Result<Self, ()> {
        match s {
            "cube" => Ok(FunctorTag::Cube),
            "sphere" => Ok(FunctorTag::Sphere),
            "cylinder" => Ok(FunctorTag::Cylinder),
            "tetrahedron" => Ok(FunctorTag::Tetrahedron),
            "union" => Ok(FunctorTag::Union),
            "difference" => Ok(FunctorTag::Difference),
            "intersection" => Ok(FunctorTag::Intersection),
            "hull" => Ok(FunctorTag::Hull),
            "translate" => Ok(FunctorTag::Translate),
            "scale" => Ok(FunctorTag::Scale),
            "rotate" => Ok(FunctorTag::Rotate),
            "p" => Ok(FunctorTag::Point),
            "sketch" => Ok(FunctorTag::Sketch),
            "rotateToXY" => Ok(FunctorTag::RotateToXY),
            "rotateToYZ" => Ok(FunctorTag::RotateToYZ),
            "rotateToXZ" => Ok(FunctorTag::RotateToXZ),
            "circle" => Ok(FunctorTag::Circle),
            "linear_extrude" => Ok(FunctorTag::LinearExtrude),
            "complex_extrude" => Ok(FunctorTag::ComplexExtrude),
            "revolve" => Ok(FunctorTag::Revolve),
            "stl" => Ok(FunctorTag::Stl),
            "line_to" => Ok(FunctorTag::LineTo),
            "bezier_to" => Ok(FunctorTag::BezierTo),
            "path" => Ok(FunctorTag::Path),
            "sweep_extrude" => Ok(FunctorTag::SweepExtrude),
            "control2d" => Ok(FunctorTag::Control2D),
            "control3d" => Ok(FunctorTag::Control3D),
            "center3d" => Ok(FunctorTag::Center3D),
            "center2d" => Ok(FunctorTag::Center2D),
            _ => Err(()),
        }
    }
}

impl fmt::Display for FunctorTag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            FunctorTag::Cube => "cube",
            FunctorTag::Sphere => "sphere",
            FunctorTag::Cylinder => "cylinder",
            FunctorTag::Tetrahedron => "tetrahedron",
            FunctorTag::Union => "union",
            FunctorTag::Difference => "difference",
            FunctorTag::Intersection => "intersection",
            FunctorTag::Hull => "hull",
            FunctorTag::Translate => "translate",
            FunctorTag::Scale => "scale",
            FunctorTag::Rotate => "rotate",
            FunctorTag::Point => "p",
            FunctorTag::Sketch => "sketch",
            FunctorTag::RotateToXY => "rotateToXY",
            FunctorTag::RotateToYZ => "rotateToYZ",
            FunctorTag::RotateToXZ => "rotateToXZ",
            FunctorTag::Circle => "circle",
            FunctorTag::LinearExtrude => "linear_extrude",
            FunctorTag::ComplexExtrude => "complex_extrude",
            FunctorTag::Revolve => "revolve",
            FunctorTag::Stl => "stl",
            FunctorTag::LineTo => "line_to",
            FunctorTag::BezierTo => "bezier_to",
            FunctorTag::Path => "path",
            FunctorTag::SweepExtrude => "sweep_extrude",
            FunctorTag::Control2D => "control2d",
            FunctorTag::Control3D => "control3d",
            FunctorTag::Center3D => "center3d",
            FunctorTag::Center2D => "center2d",
        };
        f.write_str(s)
    }
}

// ============================================================
// ConversionError
// ============================================================

#[derive(Debug, Clone)]
pub enum ConversionError {
    UnknownPrimitive(String),
    ArityMismatch {
        functor: String,
        expected: String,
        got: usize,
    },
    TypeMismatch {
        functor: String,
        arg_index: usize,
        expected: &'static str,
    },
    UnboundVariable(String),
    IoError {
        functor: String,
        message: String,
    },
    /// rotateTo* でラップされた 2D プロファイルに対して、さらに 2D 操作 (boolean / center2d / rotateTo*) を
    /// 適用しようとした場合に発生する。
    RotatedSketchSealed {
        op: &'static str,
    },
    /// linear_extrude / complex_extrude / revolve などの3D操作のプロファイル引数に、
    /// rotateToXY/YZ/XZ で平面を明示していない 2D プロファイルが渡された場合のエラー。
    MissingPlaneSpecification {
        functor: String,
    },
    /// `sketch(...)` で自己交差する閉路が指定された場合のエラー。
    /// 検出位置 (どのセグメントとどのセグメントが交差したか) を含む。
    SelfIntersectingSketch {
        segment_a: usize,
        segment_b: usize,
    },
    /// `sketch(...)` で頂点数不足や重複頂点による潰れたセグメントを検出した場合。
    DegenerateSketch {
        reason: &'static str,
    },
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
            ConversionError::RotatedSketchSealed { op } => {
                write!(
                    f,
                    "rotated 2D profile is sealed; cannot apply `{}`. rotateTo* must be the outermost 2D operation; combine sketches first, then rotate.",
                    op
                )
            }
            ConversionError::MissingPlaneSpecification { functor } => {
                write!(
                    f,
                    "{} requires an explicit plane placement on its profile; wrap the 2D profile with rotateToXY / rotateToYZ / rotateToXZ.",
                    functor
                )
            }
            ConversionError::SelfIntersectingSketch {
                segment_a,
                segment_b,
            } => {
                write!(
                    f,
                    "sketch outline is self-intersecting (segments {} and {} cross). sketch must define a simple closed polygon.",
                    segment_a, segment_b
                )
            }
            ConversionError::DegenerateSketch { reason } => {
                write!(f, "sketch is degenerate: {}", reason)
            }
        }
    }
}

impl std::error::Error for ConversionError {}

impl crate::term_rewrite::CadhrError for ConversionError {
    fn error_message(&self) -> String {
        self.to_string()
    }
    fn span(&self) -> Option<SrcSpan> {
        None
    }
}

// ============================================================
// ControlPoint: ドラッグ可能なコントロールポイント
// ============================================================

#[derive(Debug, Clone)]
pub struct ControlPoint {
    pub x: TrackedF64,
    pub y: TrackedF64,
    pub z: TrackedF64,
    pub name: Option<String>,
    /// x,y,zそれぞれに対応するVar名。override mapのキーとして使用。
    pub var_names: [Option<String>; 3],
    /// x,y,zそれぞれの値域 (min, max)。GUI スライダーの範囲指定に使用。
    /// 軸の Var に range 注釈 (例: `0<X<100`) が付いていた場合に設定。
    pub axis_ranges: [Option<(f64, f64)>; 3],
}

fn term_to_tracked_f64<S>(term: &Term<S>) -> Option<TrackedF64> {
    if let Some((r, span)) = term_as_number(term) {
        return Some(TrackedF64 {
            value: r.to_f64(),
            source_span: span,
        });
    }
    match term {
        Term::Var {
            default_value: Some(r),
            span,
            ..
        } => Some(TrackedF64 {
            value: r.to_f64(),
            source_span: *span,
        }),
        Term::Var {
            min: Some(lo),
            max: Some(hi),
            span,
            ..
        } => Some(TrackedF64 {
            value: (lo.value.to_f64() + hi.value.to_f64()) / 2.0,
            source_span: *span,
        }),
        // annotation 全 None: zero-length span (=value 挿入位置)
        Term::Var {
            default_value: None,
            min: None,
            max: None,
            span,
            ..
        } => Some(TrackedF64 {
            value: 0.0,
            source_span: span.map(|s| SrcSpan {
                start: s.end,
                end: s.end,
                file_id: s.file_id,
            }),
        }),
        Term::Var { span, .. } => Some(TrackedF64 {
            value: 0.0,
            source_span: *span,
        }),
        _ => None,
    }
}

/// 2 つの Bound から Rational の midpoint を計算する。
fn midpoint(lo: &Bound, hi: &Bound) -> Rational {
    Rational::from_f64((lo.value.to_f64() + hi.value.to_f64()) / 2.0)
}

/// `p(A, B)` 形式の Term から内部 2 引数を借用で取り出す(control2d 用)。
fn point_2d_args<S>(term: &Term<S>) -> Option<&[Term<S>; 2]> {
    if let Term::Struct {
        functor: f, args, ..
    } = term
    {
        if f == "p" && args.len() == 2 {
            return args.as_slice().try_into().ok();
        }
    }
    None
}

/// control2d(p(X,Y)[, Name]) / control3d(p(X,Y,Z)[, Name]) の Term を抽出し、
/// 残りの Term を返す。各軸の Var に対する override を適用し、同名 Var の他の参照箇所も
/// 数値に置換する。control2d は z=0 固定、var_names[2]=None。
pub fn extract_control_points<S: Clone + PartialEq + fmt::Debug>(
    terms: &mut Vec<Term<S>>,
    overrides: &std::collections::HashMap<String, f64>,
) -> Vec<ControlPoint> {
    let mut control_points = Vec::new();

    // Var substitution は (名前, スコープ, 値) の三つ組で識別する。スコープを含めることで
    // 別スコープに同名 Var があっても巻き込まない。
    let mut var_substitutions: Vec<(String, S, Term<S>)> = Vec::new();

    terms.retain(|term| {
        let Term::Struct { functor, args, .. } = term else {
            return true;
        };
        let is_2d = functor == "control2d";
        let is_3d = functor == "control3d";
        if !(is_2d || is_3d) || !(args.len() == 1 || args.len() == 2) {
            return true;
        }

        let name = if args.len() == 2 {
            match &args[1] {
                Term::StringLit { value } => Some(value.clone()),
                Term::Struct { functor, args, .. } if args.is_empty() => Some(functor.clone()),
                _ => None,
            }
        } else {
            None
        };

        let axis_count = if is_3d { 3 } else { 2 };
        let axis_labels = ["x", "y", "z"];

        // 第 1 引数が bare Var (注釈付きも含む) の場合: per-axis Var に unify する。
        // `control2d(CENTER)` → CENTER = p(CENTER.x, CENTER.y) (fresh Var)
        // `control2d(-100<CENTER<100)` → 各 axis Var が range -100..100 を継承
        // 各 fresh Var には bare Var の annotation (default/min/max) を伝搬し、range のみ
        // 指定された場合は midpoint を default に補う(center2d 等の strict 評価先で値が
        // 必要になるため)。軸名 `<bare>.<axis>` の `.` はパーサが識別子に許容しない
        // (is_id_continue 参照) ためユーザ Var との衝突は起きない。
        if let Term::Var {
            name: bare_name,
            scope: bare_scope,
            default_value,
            min,
            max,
            ..
        } = &args[0]
        {
            let inherited_default = default_value.clone().or_else(|| {
                if let (Some(lo), Some(hi)) = (min, max) {
                    Some(midpoint(lo, hi))
                } else {
                    None
                }
            });

            let mut p_args: Vec<Term<S>> = Vec::with_capacity(axis_count);
            let mut vnames: [Option<String>; 3] = [None, None, None];
            let mut axis_ranges: [Option<(f64, f64)>; 3] = [None, None, None];
            let mut tracked_axes = [
                TrackedF64 { value: 0.0, source_span: None },
                TrackedF64 { value: 0.0, source_span: None },
                TrackedF64 { value: 0.0, source_span: None },
            ];
            let bare_axis_range = if let (Some(lo), Some(hi)) = (min, max) {
                Some((lo.value.to_f64(), hi.value.to_f64()))
            } else {
                None
            };

            // 軸名は `<bare>.<axis>#<scope>` 形式。`#` はパーサが識別子に許容しない
            // (is_id_continue 参照) ため、ユーザ Var との衝突は起きない。スコープ ID を
            // 含めることで、別 predicate で同名 (例 CENTER) を control2d/3d で unify した
            // とき、それぞれ独立した override map キーになり drag が混ざらない。
            let scope_suffix = format!("{:?}", bare_scope);
            for i in 0..axis_count {
                let axis_name =
                    format!("{}.{}#{}", bare_name, axis_labels[i], scope_suffix);
                let fresh = Term::Var {
                    name: axis_name.clone(),
                    scope: bare_scope.clone(),
                    default_value: inherited_default.clone(),
                    min: min.clone(),
                    max: max.clone(),
                    span: None,
                };
                // 初期値: override > 継承 default (range のみなら midpoint) > 0
                let init = overrides
                    .get(&axis_name)
                    .copied()
                    .or_else(|| inherited_default.as_ref().map(|r| r.to_f64()))
                    .unwrap_or(0.0);
                tracked_axes[i] = TrackedF64 {
                    value: init,
                    source_span: None,
                };
                vnames[i] = Some(axis_name);
                axis_ranges[i] = bare_axis_range;
                p_args.push(fresh);
            }

            let p_term: Term<S> = Term::Struct {
                functor: "p".to_string(),
                args: p_args,
                span: None,
            };
            var_substitutions.push((bare_name.clone(), bare_scope.clone(), p_term));

            let z = if is_2d {
                TrackedF64 { value: 0.0, source_span: None }
            } else {
                tracked_axes[2].clone()
            };
            control_points.push(ControlPoint {
                x: tracked_axes[0].clone(),
                y: tracked_axes[1].clone(),
                z,
                name,
                var_names: vnames,
                axis_ranges,
            });
            return false;
        }

        // 通常パス: 引数は `p(X, Y[, Z])` リテラル。
        let axes: Option<Vec<&Term<S>>> = if is_3d {
            point_3d_args(&args[0]).map(|a| a.iter().collect())
        } else {
            point_2d_args(&args[0]).map(|a| a.iter().collect())
        };
        let Some(axes) = axes else {
            return true;
        };

        // 各軸を TrackedF64 化。失敗(構造的に不適切)なら term を残す。
        let tracked_axes: Option<Vec<TrackedF64>> =
            axes.iter().map(|t| term_to_tracked_f64(t)).collect();
        let Some(mut tracked_axes) = tracked_axes else {
            return true;
        };

        let mut vnames: [Option<String>; 3] = [None, None, None];
        let mut axis_ranges: [Option<(f64, f64)>; 3] = [None, None, None];
        for (idx, &arg) in axes.iter().enumerate() {
            // 各軸 Var の range 注釈があれば axis_ranges に保存(GUI スライダーで使う)
            if let Term::Var {
                min: Some(lo),
                max: Some(hi),
                ..
            } = arg
            {
                axis_ranges[idx] = Some((lo.value.to_f64(), hi.value.to_f64()));
            }
            if let Term::Var {
                name: vname,
                scope: vscope,
                ..
            } = arg
                && vname != "_"
            {
                let val = overrides
                    .get(vname)
                    .copied()
                    .unwrap_or(tracked_axes[idx].value);
                tracked_axes[idx].value = val;
                vnames[idx] = Some(vname.clone());
                var_substitutions.push((
                    vname.clone(),
                    vscope.clone(),
                    Term::Number {
                        value: Rational::from_f64(val),
                    },
                ));
            }
        }

        // 2D の場合は z 軸を 0 固定、var_names[2] も None のまま。
        let z = if is_2d {
            TrackedF64 {
                value: 0.0,
                source_span: None,
            }
        } else {
            tracked_axes[2].clone()
        };

        control_points.push(ControlPoint {
            x: tracked_axes[0].clone(),
            y: tracked_axes[1].clone(),
            z,
            name,
            var_names: vnames,
            axis_ranges,
        });
        false
    });

    // 残りのtermsに変数置換を適用し、算術式を評価
    if !var_substitutions.is_empty() {
        for term in terms.iter_mut() {
            substitute_vars(term, &var_substitutions);
            crate::term_rewrite::fold_number_literals_in_place(term);
        }
    }

    control_points
}

/// Var を任意の Term で substitute する。名前 + スコープの両方でマッチさせ、別スコープに
/// 同名 Var があっても巻き込まないようにする。
fn substitute_vars<S: Clone + PartialEq>(term: &mut Term<S>, subs: &[(String, S, Term<S>)]) {
    match term {
        Term::Var { name, scope, .. } => {
            if let Some((_, _, val)) = subs.iter().find(|(n, s, _)| n == name && s == scope) {
                *term = val.clone();
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

/// override mapに基づいてterms中のVar/Varを置換する
pub fn apply_var_overrides<S>(
    terms: &mut Vec<Term<S>>,
    overrides: &std::collections::HashMap<String, f64>,
) {
    if overrides.is_empty() {
        return;
    }
    for term in terms.iter_mut() {
        apply_var_overrides_to_term(term, overrides);
        crate::term_rewrite::fold_number_literals_in_place(term);
    }
}

fn apply_var_overrides_to_term<S>(
    term: &mut Term<S>,
    overrides: &std::collections::HashMap<String, f64>,
) {
    match term {
        Term::Var { name, .. } => {
            if let Some(&val) = overrides.get(name) {
                *term = Term::Number {
                    value: Rational::from_f64(val),
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

// ============================================================
// Args: 引数抽出用ヘルパー
// ============================================================

struct Args<'a, S> {
    args: &'a [Term<S>],
    functor: &'a str,
}

impl<'a, S> Args<'a, S> {
    fn new(functor: &'a str, args: &'a [Term<S>]) -> Self {
        Self { args, functor }
    }

    fn len(&self) -> usize {
        self.args.len()
    }

    fn f64(&self, i: usize) -> Result<f64, ConversionError> {
        if let Some(r) = crate::term_rewrite::try_eval_to_number(&self.args[i]) {
            return Ok(r.to_f64());
        }
        if let Some((r, _)) = term_as_number(&self.args[i]) {
            return Ok(r.to_f64());
        }
        match &self.args[i] {
            Term::Var {
                min: Some(lo),
                max: Some(hi),
                ..
            } => {
                let mid = (lo.value.to_f64() + hi.value.to_f64()) / 2.0;
                Ok(mid)
            }
            Term::Var { name, .. } => Err(ConversionError::UnboundVariable(name.clone())),
            Term::Number { .. }
            | Term::InfixExpr { .. }
            | Term::Struct { .. }
            | Term::List { .. }
            | Term::StringLit { .. }
            | Term::Constraint { .. } => Err(ConversionError::TypeMismatch {
                functor: self.functor.to_string(),
                arg_index: i,
                expected: "number",
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

    fn term_3d(&self, i: usize) -> Result<Model3D, ConversionError> {
        Model3D::from_term(&self.args[i])
    }

    fn raw(&self, i: usize) -> &Term<S> {
        &self.args[i]
    }

    /// 閉じた2Dプロファイル (extrude / boolean / center2d / rotateTo* に渡せるもの) を読む。
    /// `path(...)` (開いた polyline) が渡されたら型エラーになる。
    /// 現状の Model3D 経路は全て `term_placed` (PlacedSketch) を通って入るので未使用だが、
    /// 「Model2D を直接受け取る将来の API 用 + テスト用」として保持。
    #[allow(dead_code)]
    fn term_sketch(&self, i: usize) -> Result<Model2D, ConversionError> {
        if let Term::Struct { functor, .. } = &self.args[i] {
            if functor == "path" {
                return Err(ConversionError::TypeMismatch {
                    functor: self.functor.to_string(),
                    arg_index: i,
                    expected: "closed 2D profile (sketch / circle / boolean / center2d), not path",
                });
            }
        }
        Model2D::from_term(&self.args[i])
    }

    /// 開いた path (sweep_extrude の path 引数専用) を読む。
    /// sketch / circle / boolean などの閉じた profile が渡されたら型エラーになる。
    fn term_path(&self, i: usize) -> Result<Path, ConversionError> {
        Path::from_term(&self.args[i], self.functor, i)
    }

    fn term_placed(&self, i: usize) -> Result<PlacedSketch, ConversionError> {
        PlacedSketch::from_term(&self.args[i], self.functor)
    }

    fn arity_error(&self, expected: &str) -> ConversionError {
        ConversionError::ArityMismatch {
            functor: self.functor.to_string(),
            expected: expected.to_string(),
            got: self.len(),
        }
    }
}

// ============================================================
// Term → Model2D 変換
// ============================================================

fn ensure_ccw(points: &mut Vec<(f64, f64)>) {
    if points.len() < 3 {
        return;
    }
    let signed_area: f64 = points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .map(|(a, b)| a.0 * b.1 - b.0 * a.1)
        .sum();
    if signed_area < 0.0 {
        points.reverse();
    }
}

fn pairs_to_flat(pairs: &[(f64, f64)]) -> Vec<f64> {
    pairs.iter().flat_map(|&(x, y)| [x, y]).collect()
}

/// 入力点列を「閉路だが先頭点と末尾点は重複しない」状態に正規化する。
/// 末尾点と先頭点が一致していたら末尾を落とす (ユーザーが明示的に閉じた場合の重複排除)。
/// 末尾点が先頭点と異なる場合はそのまま (auto-close: 暗黙の最終セグメントが先頭点へ戻る)。
fn close_polygon_loop(mut points: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    const EPS: f64 = 1e-12;
    if points.len() >= 2 {
        let first = points[0];
        let last = *points.last().unwrap();
        if (first.0 - last.0).abs() < EPS && (first.1 - last.1).abs() < EPS {
            points.pop();
        }
    }
    points
}

/// 閉路として解釈した点列の simplicity (= 自己交差なし、退化セグメントなし) を検証する。
/// `points` は閉路を表すが末尾点と先頭点は重複しない状態 (segment i = points[i]→points[(i+1) % n])。
/// 隣接セグメント (共有端点) の合流は無視する。閉路の `last—first` セグメントとその両端も
/// 隣接判定の対象にする。
fn check_simple_polygon(points: &[(f64, f64)]) -> Result<(), ConversionError> {
    let n = points.len();
    if n < 3 {
        return Err(ConversionError::DegenerateSketch {
            reason: "needs at least 3 distinct vertices",
        });
    }
    const EPS: f64 = 1e-12;
    for i in 0..n {
        let a = points[i];
        let b = points[(i + 1) % n];
        if (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS {
            return Err(ConversionError::DegenerateSketch {
                reason: "consecutive duplicate vertices produce a zero-length segment",
            });
        }
    }
    for i in 0..n {
        let a1 = points[i];
        let a2 = points[(i + 1) % n];
        // 隣接セグメント (i, i+1) は端点 a2 を共有するのでスキップ。
        // 閉路では last (= n-1) と first (= 0) も隣接。
        let skip_until = if i == 0 { n - 1 } else { i + 1 };
        for j in (i + 2)..n {
            if j == skip_until {
                continue;
            }
            let b1 = points[j];
            let b2 = points[(j + 1) % n];
            if segments_cross(a1, a2, b1, b2, EPS) {
                return Err(ConversionError::SelfIntersectingSketch {
                    segment_a: i,
                    segment_b: j,
                });
            }
        }
    }
    Ok(())
}

/// 線分 a1-a2 と b1-b2 が proper に交差するか判定。
/// 共有端点・端点が他方の線分上に乗っている (T-junction) も交差として扱う。
fn segments_cross(
    a1: (f64, f64),
    a2: (f64, f64),
    b1: (f64, f64),
    b2: (f64, f64),
    eps: f64,
) -> bool {
    let d1 = orient(b1, b2, a1);
    let d2 = orient(b1, b2, a2);
    let d3 = orient(a1, a2, b1);
    let d4 = orient(a1, a2, b2);

    if ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps))
        && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))
    {
        return true;
    }
    // 端点が他方の線分の interior にある場合 (= 同一直線で重なる)
    if d1.abs() <= eps && on_segment(b1, b2, a1, eps) {
        return true;
    }
    if d2.abs() <= eps && on_segment(b1, b2, a2, eps) {
        return true;
    }
    if d3.abs() <= eps && on_segment(a1, a2, b1, eps) {
        return true;
    }
    if d4.abs() <= eps && on_segment(a1, a2, b2, eps) {
        return true;
    }
    false
}

fn orient(p: (f64, f64), q: (f64, f64), r: (f64, f64)) -> f64 {
    (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0)
}

fn on_segment(p: (f64, f64), q: (f64, f64), r: (f64, f64), eps: f64) -> bool {
    // 共線であることを呼び出し側で確認済み。bbox 内に厳密に含まれる (端点とは一致しない) ことだけ確認。
    let in_x = r.0 > p.0.min(q.0) + eps && r.0 < p.0.max(q.0) - eps;
    let in_y = r.1 > p.1.min(q.1) + eps && r.1 < p.1.max(q.1) - eps;
    // 水平/垂直線分の場合は片方の軸が degenerate。OR で許容する。
    in_x || in_y
}

/// `term` を 2D 点として抽出する。`p(number, number)` リテラルのみを受け入れる。
/// 各座標は `term_as_number` (Number / Var with default) で値化できる必要がある。
fn extract_point_2d_at<S>(
    term: &Term<S>,
    functor: &str,
    arg_index: usize,
) -> Result<(f64, f64), ConversionError> {
    if let Term::Struct {
        functor: f, args, ..
    } = term
        && f == "p"
        && args.len() == 2
    {
        let x = term_as_number(&args[0])
            .ok_or_else(|| ConversionError::TypeMismatch {
                functor: functor.to_string(),
                arg_index,
                expected: "p(number, number)",
            })?
            .0
            .to_f64();
        let y = term_as_number(&args[1])
            .ok_or_else(|| ConversionError::TypeMismatch {
                functor: functor.to_string(),
                arg_index,
                expected: "p(number, number)",
            })?
            .0
            .to_f64();
        return Ok((x, y));
    }
    Err(ConversionError::TypeMismatch {
        functor: functor.to_string(),
        arg_index,
        expected: "p(x, y)",
    })
}

fn extract_point_2d<S>(
    term: &Term<S>,
    tag: FunctorTag,
    arg_index: usize,
) -> Result<(f64, f64), ConversionError> {
    extract_point_2d_at(term, &tag.to_string(), arg_index)
}

/// `p(A, B, C)` 形式の Term から内部 3 引数を借用で取り出す。
/// 構造マッチに集中させ、各 sub-term をどう値化するかは呼び出し側に委ねる。
/// (control は TrackedF64 + Var 追跡が必要、center3d/translate は単純な f64 が必要、と
///  要求が異なるため。)
fn point_3d_args<S>(term: &Term<S>) -> Option<&[Term<S>; 3]> {
    if let Term::Struct {
        functor: f, args, ..
    } = term
    {
        if f == "p" && args.len() == 3 {
            return args.as_slice().try_into().ok();
        }
    }
    None
}

fn extract_point_3d<S>(
    term: &Term<S>,
    tag: FunctorTag,
    arg_index: usize,
) -> Result<(f64, f64, f64), ConversionError> {
    let args = point_3d_args(term).ok_or_else(|| ConversionError::TypeMismatch {
        functor: tag.to_string(),
        arg_index,
        expected: "p(x, y, z)",
    })?;
    let mut out = [0.0; 3];
    for (i, a) in args.iter().enumerate() {
        let (r, _) = term_as_number(a).ok_or_else(|| ConversionError::TypeMismatch {
            functor: tag.to_string(),
            arg_index,
            expected: "p(number, number, number)",
        })?;
        out[i] = r.to_f64();
    }
    Ok((out[0], out[1], out[2]))
}

/// `path(Start, Segments)` および `sketch(Start, Segments)` の共通パーサ。
/// 戻り値は tessellate 済みの頂点列 (先頭は Start、以降は各セグメントを評価した点)。
/// 閉路化・CCW化・自己交差チェックは呼び出し側 (Sketch のみ実施) の責任。
fn extract_segment_points<S>(
    start_term: &Term<S>,
    segments_term: &Term<S>,
    owner_tag: FunctorTag,
) -> Result<Vec<(f64, f64)>, ConversionError> {
    let owner_name = owner_tag.to_string();
    let mut current = extract_point_2d(start_term, owner_tag, 0)?;
    let mut points = vec![current];

    let segments = match segments_term {
        Term::List { items, .. } => items,
        _ => {
            return Err(ConversionError::TypeMismatch {
                functor: owner_name.clone(),
                arg_index: 1,
                expected: "list of line_to/bezier_to segments",
            });
        }
    };

    for (i, seg) in segments.iter().enumerate() {
        let (tag, args) = match seg {
            Term::Struct { functor, args, .. } => {
                (FunctorTag::from_str(functor).ok(), Some(args.as_slice()))
            }
            _ => (None, None),
        };
        match (tag, args) {
            (Some(FunctorTag::LineTo), Some([end_term])) => {
                let end = extract_point_2d(end_term, FunctorTag::LineTo, i)?;
                points.push(end);
                current = end;
            }
            (Some(FunctorTag::BezierTo), Some([cp_term, end_term])) => {
                let cp = extract_point_2d(cp_term, FunctorTag::BezierTo, i)?;
                let end = extract_point_2d(end_term, FunctorTag::BezierTo, i)?;
                points.extend(crate::bezier::evaluate_quadratic(
                    current,
                    cp,
                    end,
                    crate::bezier::DEFAULT_STEPS,
                ));
                current = end;
            }
            (Some(FunctorTag::BezierTo), Some([cp1_term, cp2_term, end_term])) => {
                let cp1 = extract_point_2d(cp1_term, FunctorTag::BezierTo, i)?;
                let cp2 = extract_point_2d(cp2_term, FunctorTag::BezierTo, i)?;
                let end = extract_point_2d(end_term, FunctorTag::BezierTo, i)?;
                points.extend(crate::bezier::evaluate_cubic(
                    current,
                    cp1,
                    cp2,
                    end,
                    crate::bezier::DEFAULT_STEPS,
                ));
                current = end;
            }
            _ => {
                return Err(ConversionError::TypeMismatch {
                    functor: owner_name.clone(),
                    arg_index: i + 1,
                    expected: "line_to(p) or bezier_to(cp,end) or bezier_to(cp1,cp2,end)",
                });
            }
        }
    }

    Ok(points)
}

impl Model2D {
    fn from_term<S>(term: &Term<S>) -> Result<Self, ConversionError> {
        match term {
            Term::Struct { functor, args, .. } => Self::from_struct(functor, args),
            Term::InfixExpr { op, left, right } => Self::from_infix_expr(*op, left, right),
            Term::Var { name, .. } => Err(ConversionError::UnboundVariable(name.clone())),
            _ => Err(ConversionError::UnknownPrimitive(format!(
                "expected 2D profile, got {:?}",
                term
            ))),
        }
    }

    fn from_infix_expr<S>(
        op: ArithOp,
        left: &Term<S>,
        right: &Term<S>,
    ) -> Result<Self, ConversionError> {
        let op_str = match op {
            ArithOp::Add => "+",
            ArithOp::Sub => "-",
            ArithOp::Mul => "*",
            ArithOp::Div => {
                return Err(ConversionError::UnknownPrimitive(
                    "division operator (/) is not supported for CAD operations".to_string(),
                ));
            }
        };
        let left = Box::new(Self::parse_2d_op_arg(left, op_str)?);
        let right = Box::new(Self::parse_2d_op_arg(right, op_str)?);
        match op {
            ArithOp::Add => Ok(Model2D::Union(left, right)),
            ArithOp::Sub => Ok(Model2D::Difference(left, right)),
            ArithOp::Mul => Ok(Model2D::Intersection(left, right)),
            ArithOp::Div => unreachable!(),
        }
    }

    fn from_struct<S>(functor: &str, args: &[Term<S>]) -> Result<Self, ConversionError> {
        let a = Args::new(functor, args);
        let tag = FunctorTag::from_str(functor)
            .map_err(|_| ConversionError::UnknownPrimitive(functor.to_string()))?;

        match tag {
            FunctorTag::Sketch if a.len() == 2 => {
                let raw = extract_segment_points(&a.args[0], &a.args[1], FunctorTag::Sketch)?;
                let mut points = close_polygon_loop(raw);
                check_simple_polygon(&points)?;
                ensure_ccw(&mut points);
                Ok(Model2D::Sketch { points })
            }
            FunctorTag::Sketch => Err(a.arity_error("2")),

            FunctorTag::Circle if a.len() == 1 => Ok(Model2D::Circle { radius: a.f64(0)? }),
            FunctorTag::Circle if a.len() == 2 => {
                // segments引数は無視（常にDEFAULT_SEGMENTS）
                Ok(Model2D::Circle { radius: a.f64(0)? })
            }
            FunctorTag::Circle => Err(a.arity_error("1 or 2")),

            // path は Model2D に含まれない (型システムで sweep_extrude 専用に閉じ込めている)。
            FunctorTag::Path => Err(ConversionError::TypeMismatch {
                functor: a.functor.to_string(),
                arg_index: 0,
                expected: "closed 2D profile, not an open path",
            }),

            FunctorTag::Union if a.len() == 2 => {
                let l = Model2D::parse_2d_op_arg(&a.args[0], "union")?;
                let r = Model2D::parse_2d_op_arg(&a.args[1], "union")?;
                Ok(Model2D::Union(Box::new(l), Box::new(r)))
            }
            FunctorTag::Difference if a.len() == 2 => {
                let l = Model2D::parse_2d_op_arg(&a.args[0], "difference")?;
                let r = Model2D::parse_2d_op_arg(&a.args[1], "difference")?;
                Ok(Model2D::Difference(Box::new(l), Box::new(r)))
            }
            FunctorTag::Intersection if a.len() == 2 => {
                let l = Model2D::parse_2d_op_arg(&a.args[0], "intersection")?;
                let r = Model2D::parse_2d_op_arg(&a.args[1], "intersection")?;
                Ok(Model2D::Intersection(Box::new(l), Box::new(r)))
            }

            FunctorTag::Center2D if a.len() == 2 => {
                let profile = Model2D::parse_2d_op_arg(&a.args[0], "center2d")?;
                let (x, y) = extract_point_2d(&a.args[1], FunctorTag::Center2D, 1)?;
                Ok(Model2D::Center2D {
                    profile: Box::new(profile),
                    x,
                    y,
                })
            }
            FunctorTag::Center2D => Err(a.arity_error("2")),

            _ => Err(ConversionError::UnknownPrimitive(format!(
                "expected 2D profile, got {}",
                functor
            ))),
        }
    }

    /// 2D操作 (union / difference / intersection / center2d / +,-,*) の引数として与えられた項を
    /// パースする。トップレベルが rotateTo* だった場合は親演算名 `parent_op` を含めたエラーを返す。
    /// 内部の Model2D::from_term は rotateTo* を含まない型に対してのみ成功するため、深く
    /// ネストした場合も型システムによって自然に弾かれる。
    fn parse_2d_op_arg<S>(
        term: &Term<S>,
        parent_op: &'static str,
    ) -> Result<Self, ConversionError> {
        if let Term::Struct { functor, .. } = term {
            if matches!(functor.as_str(), "rotateToXY" | "rotateToYZ" | "rotateToXZ") {
                return Err(ConversionError::RotatedSketchSealed { op: parent_op });
            }
        }
        Self::from_term(term)
    }

    fn to_polygon_rings(&self) -> Option<Vec<Vec<f64>>> {
        match self {
            Model2D::Sketch { points } => {
                // points は構築時に CCW 化済なので clone のみ。
                Some(vec![pairs_to_flat(points)])
            }
            Model2D::Circle { radius } => {
                let points: Vec<f64> = (0..DEFAULT_SEGMENTS)
                    .flat_map(|i| {
                        let angle =
                            2.0 * std::f64::consts::PI * (i as f64) / (DEFAULT_SEGMENTS as f64);
                        [radius * angle.cos(), radius * angle.sin()]
                    })
                    .collect();
                Some(vec![points])
            }
            Model2D::Union(a, b) => polygon_boolean_2d(a, b, |ma, mb| ma.union(mb)),
            Model2D::Difference(a, b) => polygon_boolean_2d(a, b, |ma, mb| ma.difference(mb)),
            Model2D::Intersection(a, b) => polygon_boolean_2d(a, b, |ma, mb| ma.intersection(mb)),
            Model2D::Center2D { profile, x, y } => {
                let rings = profile.to_polygon_rings()?;
                let mut min_x = f64::INFINITY;
                let mut min_y = f64::INFINITY;
                let mut max_x = f64::NEG_INFINITY;
                let mut max_y = f64::NEG_INFINITY;
                for ring in &rings {
                    for chunk in ring.chunks_exact(2) {
                        if chunk[0] < min_x {
                            min_x = chunk[0];
                        }
                        if chunk[0] > max_x {
                            max_x = chunk[0];
                        }
                        if chunk[1] < min_y {
                            min_y = chunk[1];
                        }
                        if chunk[1] > max_y {
                            max_y = chunk[1];
                        }
                    }
                }
                let dx = x - (min_x + max_x) / 2.0;
                let dy = y - (min_y + max_y) / 2.0;
                let translated = rings
                    .into_iter()
                    .map(|ring| {
                        ring.chunks_exact(2)
                            .flat_map(|c| [c[0] + dx, c[1] + dy])
                            .collect()
                    })
                    .collect();
                Some(translated)
            }
        }
    }
}

impl Path {
    /// `path(Start, Segments)` Term から開いた polyline を構築する。
    /// 構築時の検証は最低限 (頂点数2以上) のみで、自己交差や閉路性はチェックしない。
    fn from_term<S>(
        term: &Term<S>,
        consumer: &str,
        arg_index: usize,
    ) -> Result<Self, ConversionError> {
        let Term::Struct { functor, args, .. } = term else {
            return Err(ConversionError::TypeMismatch {
                functor: consumer.to_string(),
                arg_index,
                expected: "path(Start, [segments])",
            });
        };
        if functor != "path" {
            return Err(ConversionError::TypeMismatch {
                functor: consumer.to_string(),
                arg_index,
                expected: "path(Start, [segments])",
            });
        }
        if args.len() != 2 {
            return Err(ConversionError::ArityMismatch {
                functor: "path".to_string(),
                expected: "2".to_string(),
                got: args.len(),
            });
        }
        let points = extract_segment_points(&args[0], &args[1], FunctorTag::Path)?;
        if points.len() < 2 {
            return Err(ConversionError::TypeMismatch {
                functor: "path".to_string(),
                arg_index: 1,
                expected: "path with at least one segment (2+ points)",
            });
        }
        Ok(Path { points })
    }
}

impl PlacedSketch {
    /// 3D操作 (linear_extrude / complex_extrude / revolve) のプロファイル引数のパース。
    /// 外側が rotateToXY/YZ/XZ である必要があり、無ければ `MissingPlaneSpecification` エラー。
    /// ただし2D操作の中に rotateTo* が紛れ込んでいる場合は (より具体的な)
    /// `RotatedSketchSealed` を優先して返す。
    /// `consumer` には呼び出し元の3D操作名 (例: "linear_extrude") を渡す。
    fn from_term<S>(term: &Term<S>, consumer: &str) -> Result<Self, ConversionError> {
        if let Term::Struct { functor, args, .. } = term {
            let plane = match functor.as_str() {
                "rotateToXY" => Some(Plane3D::XY),
                "rotateToYZ" => Some(Plane3D::YZ),
                "rotateToXZ" => Some(Plane3D::XZ),
                _ => None,
            };
            if let Some(plane) = plane {
                if args.len() != 1 {
                    return Err(ConversionError::ArityMismatch {
                        functor: functor.to_string(),
                        expected: "1".to_string(),
                        got: args.len(),
                    });
                }
                let outer_op = match plane {
                    Plane3D::XY => "rotateToXY",
                    Plane3D::YZ => "rotateToYZ",
                    Plane3D::XZ => "rotateToXZ",
                };
                // rotateTo* をネストすると同じエラーで弾く
                let profile = Model2D::parse_2d_op_arg(&args[0], outer_op)?;
                return Ok(PlacedSketch { plane, profile });
            }
        }
        // ここに来るのは「最外殻が rotateTo* でない」ケース。
        // 中に rotateTo* が紛れ込んでいるかは Model2D::from_term の seal 検査で判明する。
        // 紛れ込んでいたら、ユーザーに直接「rotateTo* が 2D 操作内にある」と伝える方が親切。
        match Model2D::from_term(term) {
            Err(e @ ConversionError::RotatedSketchSealed { .. }) => Err(e),
            _ => Err(ConversionError::MissingPlaneSpecification {
                functor: consumer.to_string(),
            }),
        }
    }

    /// 押し出し後の3Dソリッドに適用する回転角度 (rx, ry, rz) を返す。
    fn plane_rotation(&self) -> Option<(f64, f64, f64)> {
        match self.plane {
            Plane3D::XY => None,
            Plane3D::YZ => Some((90.0, 0.0, 90.0)),
            Plane3D::XZ => Some((-90.0, 0.0, 0.0)),
        }
    }

    fn to_polygon_rings(&self) -> Option<Vec<Vec<f64>>> {
        let rings = self.profile.to_polygon_rings()?;
        // XZ平面はRx(-90°)で+Y方向に押し出すため、ローカルY座標を反転して回転後の+Y方向に合わせる。
        // Y反転は多角形の符号付き面積を反転させCW化するので、Manifold::extrudeに渡す前にensure_ccwで戻す。
        if self.plane == Plane3D::XZ {
            Some(
                rings
                    .into_iter()
                    .map(|ring| {
                        let mut pairs: Vec<(f64, f64)> =
                            ring.chunks_exact(2).map(|c| (c[0], -c[1])).collect();
                        ensure_ccw(&mut pairs);
                        pairs_to_flat(&pairs)
                    })
                    .collect(),
            )
        } else {
            Some(rings)
        }
    }
}

const THIN_EXTRUDE_HEIGHT: f64 = 1.0;

fn polygon_boolean_2d(
    a: &Model2D,
    b: &Model2D,
    op: impl FnOnce(&Manifold, &Manifold) -> Manifold,
) -> Option<Vec<Vec<f64>>> {
    let rings_a = a.to_polygon_rings()?;
    let rings_b = b.to_polygon_rings()?;
    let refs_a: Vec<&[f64]> = rings_a.iter().map(|r| r.as_slice()).collect();
    let refs_b: Vec<&[f64]> = rings_b.iter().map(|r| r.as_slice()).collect();
    let ma = Manifold::extrude(&refs_a, THIN_EXTRUDE_HEIGHT, 0, 0.0, 1.0, 1.0);
    let mb = Manifold::extrude(&refs_b, THIN_EXTRUDE_HEIGHT, 0, 0.0, 1.0, 1.0);
    let result = op(&ma, &mb);
    let polygons = result.project();
    let mut rings = Vec::with_capacity(polygons.size());
    for i in 0..polygons.size() {
        rings.push(polygons.get_as_slice(i).to_vec());
    }
    Some(rings)
}

fn polygon_rings_or_err(
    profile: &PlacedSketch,
    functor: &str,
) -> Result<Vec<Vec<f64>>, ConversionError> {
    profile
        .to_polygon_rings()
        .ok_or_else(|| ConversionError::TypeMismatch {
            functor: functor.to_string(),
            arg_index: 0,
            expected: "polygon data",
        })
}

fn flat_to_pairs(flat: &[f64]) -> Vec<(f64, f64)> {
    flat.chunks_exact(2).map(|c| (c[0], c[1])).collect()
}

fn apply_plane_rotation(m: Manifold, profile: &PlacedSketch) -> Manifold {
    match profile.plane_rotation() {
        Some((rx, ry, rz)) => m.rotate(rx, ry, rz),
        None => m,
    }
}

// ============================================================
// Term → Model3D 変換
// ============================================================

impl Model3D {
    pub fn from_term<S>(term: &Term<S>) -> Result<Self, ConversionError> {
        match term {
            Term::Struct { functor, args, .. } => Self::from_struct(functor, args),
            Term::InfixExpr { op, left, right } => Self::from_infix_expr(*op, left, right),
            Term::Var { name, .. } => Err(ConversionError::UnboundVariable(name.clone())),
            Term::Constraint { .. } => Err(ConversionError::UnknownPrimitive(
                "constraint should not reach mesh generation".to_string(),
            )),
            _ => Err(ConversionError::UnknownPrimitive(format!("{:?}", term))),
        }
    }

    /// 中置演算子をCAD操作として変換
    /// + -> union, - -> difference, * -> intersection
    fn from_infix_expr<S>(
        op: ArithOp,
        left: &Term<S>,
        right: &Term<S>,
    ) -> Result<Self, ConversionError> {
        // depth-first: まず2Dとして両辺を試み、両方成功したら2Dを含む3D(extrude)ではなく
        // 呼び出し元が3Dを期待しているので、3Dとして解釈する
        let left_expr = Box::new(Self::from_term(left)?);
        let right_expr = Box::new(Self::from_term(right)?);

        match op {
            ArithOp::Add => Ok(Model3D::Union(left_expr, right_expr)),
            ArithOp::Sub => Ok(Model3D::Difference(left_expr, right_expr)),
            ArithOp::Mul => Ok(Model3D::Intersection(left_expr, right_expr)),
            ArithOp::Div => Err(ConversionError::UnknownPrimitive(
                "division operator (/) is not supported for CAD operations".to_string(),
            )),
        }
    }

    fn from_struct<S>(functor: &str, args: &[Term<S>]) -> Result<Self, ConversionError> {
        let a = Args::new(functor, args);
        let tag = FunctorTag::from_str(functor)
            .map_err(|_| ConversionError::UnknownPrimitive(functor.to_string()))?;

        match tag {
            FunctorTag::Cube if a.len() == 3 => Ok(Model3D::Cube {
                x: a.f64(0)?,
                y: a.f64(1)?,
                z: a.f64(2)?,
            }),
            FunctorTag::Cube => Err(a.arity_error("3")),

            FunctorTag::Sphere if a.len() == 1 => Ok(Model3D::Sphere { radius: a.f64(0)? }),
            FunctorTag::Sphere if a.len() == 2 => {
                // segments引数は無視（常にDEFAULT_SEGMENTS）
                Ok(Model3D::Sphere { radius: a.f64(0)? })
            }
            FunctorTag::Sphere => Err(a.arity_error("1 or 2")),

            FunctorTag::Cylinder if a.len() == 2 => Ok(Model3D::Cylinder {
                radius: a.f64(0)?,
                height: a.f64(1)?,
            }),
            FunctorTag::Cylinder if a.len() == 3 => {
                // segments引数は無視（常にDEFAULT_SEGMENTS）
                Ok(Model3D::Cylinder {
                    radius: a.f64(0)?,
                    height: a.f64(1)?,
                })
            }
            FunctorTag::Cylinder => Err(a.arity_error("2 or 3")),

            FunctorTag::Tetrahedron if a.len() == 0 => Ok(Model3D::Tetrahedron),
            FunctorTag::Tetrahedron => Err(a.arity_error("0")),

            FunctorTag::Union if a.len() == 2 => Ok(Model3D::Union(
                Box::new(a.term_3d(0)?),
                Box::new(a.term_3d(1)?),
            )),
            FunctorTag::Union => Err(a.arity_error("2")),

            FunctorTag::Difference if a.len() == 2 => Ok(Model3D::Difference(
                Box::new(a.term_3d(0)?),
                Box::new(a.term_3d(1)?),
            )),
            FunctorTag::Difference => Err(a.arity_error("2")),

            FunctorTag::Intersection if a.len() == 2 => Ok(Model3D::Intersection(
                Box::new(a.term_3d(0)?),
                Box::new(a.term_3d(1)?),
            )),
            FunctorTag::Intersection => Err(a.arity_error("2")),

            FunctorTag::Hull if a.len() == 2 => Ok(Model3D::Hull(
                Box::new(a.term_3d(0)?),
                Box::new(a.term_3d(1)?),
            )),
            FunctorTag::Hull => Err(a.arity_error("2")),

            FunctorTag::Translate if a.len() == 3 => {
                let model = a.term_3d(0)?;
                let (sx, sy, sz) = extract_point_3d(a.raw(1), FunctorTag::Translate, 1)?;
                let (dx, dy, dz) = extract_point_3d(a.raw(2), FunctorTag::Translate, 2)?;
                Ok(Model3D::Translate {
                    model: Box::new(model),
                    x: dx - sx,
                    y: dy - sy,
                    z: dz - sz,
                })
            }
            FunctorTag::Translate => Err(a.arity_error("3")),

            FunctorTag::Scale if a.len() == 4 => Ok(Model3D::Scale {
                model: Box::new(a.term_3d(0)?),
                x: a.f64(1)?,
                y: a.f64(2)?,
                z: a.f64(3)?,
            }),
            FunctorTag::Scale => Err(a.arity_error("4")),

            FunctorTag::Rotate if a.len() == 4 => Ok(Model3D::Rotate {
                model: Box::new(a.term_3d(0)?),
                x: a.f64(1)?,
                y: a.f64(2)?,
                z: a.f64(3)?,
            }),
            FunctorTag::Rotate => Err(a.arity_error("4")),

            FunctorTag::LinearExtrude if a.len() == 2 => Ok(Model3D::LinearExtrude {
                profile: a.term_placed(0)?,
                height: a.f64(1)?,
            }),
            FunctorTag::LinearExtrude => Err(a.arity_error("2")),

            FunctorTag::ComplexExtrude if a.len() == 5 => Ok(Model3D::ComplexExtrude {
                profile: a.term_placed(0)?,
                height: a.f64(1)?,
                twist: a.f64(2)?,
                scale_x: a.f64(3)?,
                scale_y: a.f64(4)?,
            }),
            FunctorTag::ComplexExtrude => Err(a.arity_error("5")),

            FunctorTag::Revolve if a.len() == 2 => Ok(Model3D::Revolve {
                profile: a.term_placed(0)?,
                degrees: a.f64(1)?,
            }),
            FunctorTag::Revolve if a.len() == 3 => {
                // segments引数は無視（常にDEFAULT_SEGMENTS）
                Ok(Model3D::Revolve {
                    profile: a.term_placed(0)?,
                    degrees: a.f64(1)?,
                })
            }
            FunctorTag::Revolve => Err(a.arity_error("2 or 3")),

            FunctorTag::Stl if a.len() == 1 => {
                let path = a.string(0)?;
                Ok(Model3D::Stl { path })
            }
            FunctorTag::Stl => Err(a.arity_error("1")),

            FunctorTag::SweepExtrude if a.len() == 2 => {
                let profile = a.term_placed(0)?;
                let path = a.term_path(1)?;
                Ok(Model3D::SweepExtrude { profile, path })
            }
            FunctorTag::SweepExtrude => Err(a.arity_error("2")),

            FunctorTag::Point => Err(ConversionError::UnknownPrimitive(
                "p is a data constructor, not a shape primitive".to_string(),
            )),
            FunctorTag::LineTo | FunctorTag::BezierTo => {
                Err(ConversionError::UnknownPrimitive(format!(
                    "{} is a data constructor for path, not a shape primitive",
                    functor
                )))
            }
            FunctorTag::Center3D if a.len() == 2 => {
                let model = a.term_3d(0)?;
                let (x, y, z) = extract_point_3d(a.raw(1), FunctorTag::Center3D, 1)?;
                Ok(Model3D::Center3D {
                    model: Box::new(model),
                    x,
                    y,
                    z,
                })
            }
            FunctorTag::Center3D => Err(a.arity_error("2")),

            FunctorTag::Center2D => Err(ConversionError::UnknownPrimitive(
                "center2d is a 2D-only operation; wrap with linear_extrude/revolve before using as 3D".to_string(),
            )),

            FunctorTag::Control2D | FunctorTag::Control3D => Err(ConversionError::UnknownPrimitive(
                "control2d / control3d are data constructors, not shape primitives".to_string(),
            )),

            // 2Dプロファイルがそのまま3Dコンテキストに置かれた場合、薄いextrudeとして3D化する。
            // ただし平面の指定が必須なので rotateTo* で明示的にラップされている場合のみ受け付ける。
            FunctorTag::RotateToXY | FunctorTag::RotateToYZ | FunctorTag::RotateToXZ => {
                if a.len() != 1 {
                    return Err(a.arity_error("1"));
                }
                let (plane, inner_op) = match tag {
                    FunctorTag::RotateToXY => (Plane3D::XY, "rotateToXY"),
                    FunctorTag::RotateToYZ => (Plane3D::YZ, "rotateToYZ"),
                    FunctorTag::RotateToXZ => (Plane3D::XZ, "rotateToXZ"),
                    _ => unreachable!(),
                };
                let profile = Model2D::parse_2d_op_arg(&a.args[0], inner_op)?;
                Ok(Model3D::LinearExtrude {
                    profile: PlacedSketch { plane, profile },
                    height: 0.001,
                })
            }
            FunctorTag::Sketch | FunctorTag::Circle => {
                Err(ConversionError::MissingPlaneSpecification {
                    functor: functor.to_string(),
                })
            }
            FunctorTag::Path => Err(ConversionError::TypeMismatch {
                functor: functor.to_string(),
                arg_index: 0,
                expected: "path is only usable as the second argument of sweep_extrude",
            }),
        }
    }

    /// Model3D を manifold-rs の Manifold に評価
    pub fn evaluate(&self, include_paths: &[PathBuf]) -> Result<Manifold, ConversionError> {
        match self {
            Model3D::Cube { x, y, z } => Ok(Manifold::cube(*x, *y, *z)),
            Model3D::Sphere { radius } => Ok(Manifold::sphere(*radius, DEFAULT_SEGMENTS)),
            Model3D::Cylinder { radius, height } => Ok(Manifold::cylinder(
                *radius,
                *radius,
                *height,
                DEFAULT_SEGMENTS,
            )),
            Model3D::Tetrahedron => Ok(Manifold::tetrahedron()),

            Model3D::Union(a, b) => Ok(a
                .evaluate(include_paths)?
                .union(&b.evaluate(include_paths)?)),
            Model3D::Difference(a, b) => Ok(a
                .evaluate(include_paths)?
                .difference(&b.evaluate(include_paths)?)),
            Model3D::Intersection(a, b) => Ok(a
                .evaluate(include_paths)?
                .intersection(&b.evaluate(include_paths)?)),
            Model3D::Hull(a, b) => Ok(a
                .evaluate(include_paths)?
                .union(&b.evaluate(include_paths)?)
                .hull()),

            Model3D::Translate { model, x, y, z } => {
                Ok(model.evaluate(include_paths)?.translate(*x, *y, *z))
            }
            Model3D::Scale { model, x, y, z } => {
                Ok(model.evaluate(include_paths)?.scale(*x, *y, *z))
            }
            Model3D::Rotate { model, x, y, z } => {
                Ok(model.evaluate(include_paths)?.rotate(*x, *y, *z))
            }

            Model3D::LinearExtrude { profile, height } => {
                let rings = polygon_rings_or_err(profile, "linear_extrude")?;
                let refs: Vec<&[f64]> = rings.iter().map(|r| r.as_slice()).collect();
                let m = Manifold::extrude(&refs, *height, 0, 0.0, 1.0, 1.0);
                Ok(apply_plane_rotation(m, profile))
            }
            Model3D::ComplexExtrude {
                profile,
                height,
                twist,
                scale_x,
                scale_y,
            } => {
                let rings = polygon_rings_or_err(profile, "complex_extrude")?;
                let refs: Vec<&[f64]> = rings.iter().map(|r| r.as_slice()).collect();
                let n_divisions = (height.abs() as u32).max(1);
                let m = Manifold::extrude(&refs, *height, n_divisions, *twist, *scale_x, *scale_y);
                Ok(apply_plane_rotation(m, profile))
            }
            Model3D::Revolve { profile, degrees } => {
                let rings = polygon_rings_or_err(profile, "revolve")?;
                let refs: Vec<&[f64]> = rings.iter().map(|r| r.as_slice()).collect();
                let m = Manifold::revolve(&refs, DEFAULT_SEGMENTS, *degrees);
                Ok(apply_plane_rotation(m, profile))
            }

            Model3D::SweepExtrude { profile, path } => {
                let rings = polygon_rings_or_err(profile, "sweep_extrude")?;
                let profile_data = flat_to_pairs(&rings[0]);
                let (verts, indices) =
                    crate::sweep::sweep_extrude_mesh(&profile_data, &path.points)?;
                let mesh = Mesh::new(&verts, &indices);
                let m = Manifold::from_mesh(mesh);
                Ok(apply_plane_rotation(m, profile))
            }

            Model3D::Center3D { model, x, y, z } => {
                let manifold = model.evaluate(include_paths)?;
                let mesh = manifold.calculate_normals(0, 30.0).to_mesh();
                let num_props = mesh.num_props() as usize;
                let mut min = [f64::INFINITY; 3];
                let mut max = [f64::NEG_INFINITY; 3];
                for chunk in mesh.vertices().chunks(num_props) {
                    for i in 0..3 {
                        let v = chunk[i] as f64;
                        if v < min[i] {
                            min[i] = v;
                        }
                        if v > max[i] {
                            max[i] = v;
                        }
                    }
                }
                let cx = (min[0] + max[0]) / 2.0;
                let cy = (min[1] + max[1]) / 2.0;
                let cz = (min[2] + max[2]) / 2.0;
                Ok(manifold.translate(x - cx, y - cy, z - cz))
            }

            Model3D::Stl { path } => {
                let raw = StdPath::new(path);
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

    /// Model3D を Mesh に変換（法線計算込み）
    pub fn to_mesh(&self, include_paths: &[PathBuf]) -> Result<Mesh, ConversionError> {
        let manifold = self.evaluate(include_paths)?;
        let with_normals = manifold.calculate_normals(0, 30.0);
        Ok(with_normals.to_mesh())
    }
}

// ============================================================
// EvaluatedNode: raycastによるノード特定に使用
// ============================================================

#[derive(Clone)]
pub struct EvaluatedNode {
    pub expr: Model3D,
    pub mesh_verts: Vec<f32>,
    pub mesh_indices: Vec<u32>,
    pub aabb_min: [f64; 3],
    pub aabb_max: [f64; 3],
    pub children: Vec<EvaluatedNode>,
}

fn build_evaluated_node(
    expr: &Model3D,
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
        Model3D::Union(a, b)
        | Model3D::Difference(a, b)
        | Model3D::Intersection(a, b)
        | Model3D::Hull(a, b) => {
            vec![
                build_evaluated_node(a, include_paths)?,
                build_evaluated_node(b, include_paths)?,
            ]
        }
        Model3D::Translate { model: e, .. }
        | Model3D::Scale { model: e, .. }
        | Model3D::Rotate { model: e, .. }
        | Model3D::Center3D { model: e, .. } => {
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

// ============================================================
// MeshGenerator: TermProcessor実装
// ============================================================

pub struct MeshGenerator {
    pub include_paths: Vec<PathBuf>,
}

impl<S> crate::term_processor::TermProcessor<S> for MeshGenerator {
    type Output = (Mesh, Vec<EvaluatedNode>);
    type Error = ConversionError;

    fn process(&self, terms: &[Term<S>]) -> Result<Self::Output, Self::Error> {
        let exprs: Vec<Model3D> = terms
            .iter()
            .filter_map(|t| match Model3D::from_term(t) {
                Ok(e) => Some(Ok(e)),
                Err(ConversionError::UnknownPrimitive(_)) => None,
                Err(e) => Some(Err(e)),
            })
            .collect::<Result<Vec<_>, _>>()?;

        if exprs.is_empty() {
            return Err(ConversionError::UnknownPrimitive(
                "no mesh terms found".to_string(),
            ));
        }

        let nodes: Vec<EvaluatedNode> = exprs
            .iter()
            .map(|e| build_evaluated_node(e, &self.include_paths))
            .collect::<Result<Vec<_>, _>>()?;

        let manifold = exprs
            .iter()
            .map(|e| e.evaluate(&self.include_paths))
            .reduce(|acc, m| Ok(acc?.union(&m?)))
            .unwrap()?;

        let with_normals = manifold.calculate_normals(0, 30.0);
        Ok((with_normals.to_mesh(), nodes))
    }
}

pub fn generate_mesh_and_tree_from_terms<S>(
    terms: &[Term<S>],
    include_paths: &[PathBuf],
) -> Result<(Mesh, Vec<EvaluatedNode>), ConversionError> {
    use crate::term_processor::TermProcessor;
    MeshGenerator {
        include_paths: include_paths.to_vec(),
    }
    .process(terms)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{number_int, string_lit, struc, var};

    #[test]
    fn test_cube_conversion() {
        let term: Term = struc(
            "cube".into(),
            vec![number_int(10), number_int(20), number_int(30)],
        );
        let expr = Model3D::from_term(&term).unwrap();
        match expr {
            Model3D::Cube { x, y, z } => {
                assert_eq!(x, 10.0);
                assert_eq!(y, 20.0);
                assert_eq!(z, 30.0);
            }
            _ => panic!("Expected Cube"),
        }
    }

    #[test]
    fn test_sphere_default_segments() {
        let term: Term = struc("sphere".into(), vec![number_int(5)]);
        let expr = Model3D::from_term(&term).unwrap();
        match expr {
            Model3D::Sphere { radius } => {
                assert_eq!(radius, 5.0);
            }
            _ => panic!("Expected Sphere"),
        }
    }

    #[test]
    fn test_sphere_explicit_segments() {
        let term: Term = struc("sphere".into(), vec![number_int(5), number_int(16)]);
        let expr = Model3D::from_term(&term).unwrap();
        match expr {
            Model3D::Sphere { radius } => {
                assert_eq!(radius, 5.0);
            }
            _ => panic!("Expected Sphere"),
        }
    }

    #[test]
    fn test_cylinder_default_segments() {
        let term: Term = struc("cylinder".into(), vec![number_int(3), number_int(10)]);
        let expr = Model3D::from_term(&term).unwrap();
        match expr {
            Model3D::Cylinder { radius, height } => {
                assert_eq!(radius, 3.0);
                assert_eq!(height, 10.0);
            }
            _ => panic!("Expected Cylinder"),
        }
    }

    #[test]
    fn test_translate_conversion() {
        // 新 API: translate(Shape, SrcPoint, DstPoint) で SrcPoint を DstPoint に運ぶ。
        // 内部表現は DstPoint - SrcPoint の差分。
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let src = struc(
            "p".into(),
            vec![number_int(0), number_int(0), number_int(0)],
        );
        let dst = struc(
            "p".into(),
            vec![number_int(5), number_int(10), number_int(15)],
        );
        let translated = struc("translate".into(), vec![cube, src, dst]);
        let expr = Model3D::from_term(&translated).unwrap();
        match expr {
            Model3D::Translate { x, y, z, .. } => {
                assert_eq!(x, 5.0);
                assert_eq!(y, 10.0);
                assert_eq!(z, 15.0);
            }
            _ => panic!("Expected Translate"),
        }
    }

    #[test]
    fn test_translate_point_based_nonzero_src() {
        // SrcPoint が原点でない場合: 差分が translate 量になる。
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(2), number_int(2), number_int(2)],
        );
        let src = struc(
            "p".into(),
            vec![number_int(1), number_int(2), number_int(3)],
        );
        let dst = struc(
            "p".into(),
            vec![number_int(10), number_int(20), number_int(30)],
        );
        let translated = struc("translate".into(), vec![cube, src, dst]);
        let expr = Model3D::from_term(&translated).unwrap();
        match expr {
            Model3D::Translate { x, y, z, .. } => {
                assert_eq!(x, 9.0);
                assert_eq!(y, 18.0);
                assert_eq!(z, 27.0);
            }
            _ => panic!("Expected Translate"),
        }
    }

    #[test]
    fn test_translate_wrong_arity() {
        // 旧 API arity 4 は arity error になる。
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let translated = struc(
            "translate".into(),
            vec![cube, number_int(5), number_int(10), number_int(15)],
        );
        let result = Model3D::from_term(&translated);
        assert!(matches!(result, Err(ConversionError::ArityMismatch { .. })));
    }

    #[test]
    fn test_translate_non_point_arg() {
        // SrcPoint/DstPoint に p(...) 以外を渡すと TypeMismatch。
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let translated = struc(
            "translate".into(),
            vec![cube, number_int(0), number_int(5)],
        );
        let result = Model3D::from_term(&translated);
        assert!(matches!(result, Err(ConversionError::TypeMismatch { .. })));
    }

    #[test]
    fn test_unbound_variable_error() {
        let term: Term = struc(
            "cube".into(),
            vec![var("X".into()), number_int(1), number_int(1)],
        );
        let result = Model3D::from_term(&term);
        assert!(matches!(result, Err(ConversionError::UnboundVariable(_))));
    }

    #[test]
    fn test_arity_mismatch() {
        let term: Term = struc("cube".into(), vec![number_int(1), number_int(2)]);
        let result = Model3D::from_term(&term);
        assert!(matches!(result, Err(ConversionError::ArityMismatch { .. })));
    }

    #[test]
    fn test_unknown_primitive() {
        let term: Term = struc("unknown_shape".into(), vec![number_int(1)]);
        let result = Model3D::from_term(&term);
        assert!(matches!(result, Err(ConversionError::UnknownPrimitive(_))));
    }

    #[test]
    fn test_nested_csg() {
        // difference(union(cube(1,1,1), cube(2,2,2)), sphere(1))
        let cube1: Term = struc(
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

        let expr = Model3D::from_term(&diff).unwrap();
        assert!(matches!(expr, Model3D::Difference(_, _)));
    }

    #[test]
    fn test_operator_union() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // cube(1,1,1) + sphere(1) -> union
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let add_term = arith_expr(ArithOp::Add, cube, sphere);

        let expr = Model3D::from_term(&add_term).unwrap();
        assert!(matches!(expr, Model3D::Union(_, _)));
    }

    #[test]
    fn test_operator_difference() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // cube(1,1,1) - sphere(1) -> difference
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let sub_term = arith_expr(ArithOp::Sub, cube, sphere);

        let expr = Model3D::from_term(&sub_term).unwrap();
        assert!(matches!(expr, Model3D::Difference(_, _)));
    }

    #[test]
    fn test_operator_intersection() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // cube(1,1,1) * sphere(1) -> intersection
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let mul_term = arith_expr(ArithOp::Mul, cube, sphere);

        let expr = Model3D::from_term(&mul_term).unwrap();
        assert!(matches!(expr, Model3D::Intersection(_, _)));
    }

    #[test]
    fn test_operator_nested() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // (cube(1,1,1) + sphere(1)) - cylinder(1,2)
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let cylinder = struc("cylinder".into(), vec![number_int(1), number_int(2)]);

        let union_term = arith_expr(ArithOp::Add, cube, sphere);
        let diff_term = arith_expr(ArithOp::Sub, union_term, cylinder);

        let expr = Model3D::from_term(&diff_term).unwrap();
        match expr {
            Model3D::Difference(left, _) => {
                assert!(matches!(*left, Model3D::Union(_, _)));
            }
            _ => panic!("Expected Difference"),
        }
    }

    #[test]
    fn test_operator_division_error() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        // cube(1,1,1) / sphere(1) -> error
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let sphere = struc("sphere".into(), vec![number_int(1)]);
        let div_term = arith_expr(ArithOp::Div, cube, sphere);

        let result = Model3D::from_term(&div_term);
        assert!(matches!(result, Err(ConversionError::UnknownPrimitive(_))));
    }

    /// 旧 `sketch([p,p,...])` API のテスト用ヘルパ。新仕様の `sketch(Start, [line_to(p),...])`
    /// を生成する。`pts` は閉路の頂点列で、末尾の auto-close は新 sketch 側が処理する。
    fn make_polygon_term(pts: Vec<(i64, i64)>) -> Term {
        assert!(
            pts.len() >= 3,
            "sketch requires at least 3 vertices, got {}",
            pts.len()
        );
        let (sx, sy) = pts[0];
        let start = struc("p".into(), vec![number_int(sx), number_int(sy)]);
        let segments: Vec<Term> = pts[1..]
            .iter()
            .map(|&(x, y)| {
                struc(
                    "line_to".into(),
                    vec![struc("p".into(), vec![number_int(x), number_int(y)])],
                )
            })
            .collect();
        struc(
            "sketch".into(),
            vec![start, crate::parse::list(segments, None)],
        )
    }

    #[test]
    fn test_polygon_conversion() {
        // CCW 入力: そのまま保存される。auto-close で末尾頂点と先頭頂点は重複しないこと。
        let term = make_polygon_term(vec![(0, 0), (1, 0), (1, 1), (0, 1)]);
        let expr = Model2D::from_term(&term).unwrap();
        match expr {
            Model2D::Sketch { points } => {
                assert_eq!(points, vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
            }
            _ => panic!("Expected Sketch"),
        }
    }

    /// CW で書いた sketch が構築時に CCW に正規化されること。
    #[test]
    fn test_sketch_cw_input_is_normalized_to_ccw() {
        // 時計回り (CW): (0,0)->(0,1)->(1,1)->(1,0)
        let term = make_polygon_term(vec![(0, 0), (0, 1), (1, 1), (1, 0)]);
        let expr = Model2D::from_term(&term).unwrap();
        match expr {
            Model2D::Sketch { points } => {
                // ensure_ccw が呼ばれて反転されている
                assert_eq!(points, vec![(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]);
            }
            _ => panic!("Expected Sketch"),
        }
    }

    /// 末尾セグメントの終点を始点に明示的に戻した場合、重複頂点が増えないこと。
    #[test]
    fn test_sketch_explicit_close_dedupes_last_vertex() {
        // 明示的に始点に戻す: 4 line_to + 末尾は始点と一致 → 重複排除されて 4 頂点。
        let start: Term = struc("p".into(), vec![number_int(0), number_int(0)]);
        let segments: Vec<Term> = vec![
            line_to_term(1, 0),
            line_to_term(1, 1),
            line_to_term(0, 1),
            // 明示的に始点に戻る
            line_to_term(0, 0),
        ];
        let term: Term = struc(
            "sketch".into(),
            vec![start, crate::parse::list(segments, None)],
        );
        let expr = Model2D::from_term(&term).unwrap();
        match expr {
            Model2D::Sketch { points } => {
                assert_eq!(points.len(), 4, "explicit close must not duplicate first vertex");
                assert_eq!(points, vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
            }
            _ => panic!("Expected Sketch"),
        }
    }

    /// 砂時計型 (自己交差) の sketch は構築時に拒否される。
    #[test]
    fn test_sketch_rejects_self_intersection() {
        // (0,0)->(10,10)->(10,0)->(0,10)->close で X 字に交差する。
        let term = make_polygon_term(vec![(0, 0), (10, 10), (10, 0), (0, 10)]);
        let err = Model2D::from_term(&term).unwrap_err();
        assert!(
            matches!(err, ConversionError::SelfIntersectingSketch { .. }),
            "expected SelfIntersectingSketch, got {:?}",
            err
        );
    }

    /// 重複した連続頂点 (退化セグメント) は構築時に拒否される。
    #[test]
    fn test_sketch_rejects_consecutive_duplicate_vertex() {
        let term = make_polygon_term(vec![(0, 0), (1, 0), (1, 0), (1, 1), (0, 1)]);
        let err = Model2D::from_term(&term).unwrap_err();
        assert!(
            matches!(err, ConversionError::DegenerateSketch { .. }),
            "expected DegenerateSketch, got {:?}",
            err
        );
    }

    /// 頂点数 < 3 の sketch は構築時に拒否される。
    #[test]
    fn test_sketch_rejects_degenerate_two_vertices() {
        let term = make_polygon_term(vec![(0, 0), (1, 0), (1, 0)]);
        // この場合は重複頂点で先に DegenerateSketch になる。頂点数だけ少ないケースは
        // segment_points が enforce する (line_to 1個のみ → 2点 → close後も2点未満)。
        let err = Model2D::from_term(&term).unwrap_err();
        assert!(
            matches!(err, ConversionError::DegenerateSketch { .. }),
            "expected DegenerateSketch, got {:?}",
            err
        );
    }

    /// path を sketch のスロット (linear_extrude の profile) に渡すと型エラーになること。
    /// PlacedSketch::from_term の内部で Model2D::from_term が呼ばれ、Path は Model2D に
    /// 含まれないので TypeMismatch で弾かれる。
    #[test]
    fn test_path_rejected_in_extrude_profile() {
        let path = xy(make_path_term(
            (0, 0),
            vec![line_to_term(10, 0), line_to_term(10, 10), line_to_term(0, 10)],
        ));
        let term = struc("linear_extrude".into(), vec![path, number_int(5)]);
        let err = Model3D::from_term(&term).unwrap_err();
        assert!(
            matches!(err, ConversionError::TypeMismatch { .. }),
            "expected TypeMismatch, got {:?}",
            err
        );
    }

    /// path を boolean (`+`) の引数に渡すと型エラーになること。
    #[test]
    fn test_path_rejected_in_boolean() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        let path = make_path_term((0, 0), vec![line_to_term(10, 0), line_to_term(0, 10)]);
        let poly = make_polygon_term(vec![(0, 0), (5, 0), (5, 5)]);
        let union_term = arith_expr(ArithOp::Add, path, poly);
        let err = Model2D::from_term(&union_term).unwrap_err();
        assert!(
            matches!(err, ConversionError::TypeMismatch { .. }),
            "expected TypeMismatch, got {:?}",
            err
        );
    }

    /// sketch を sweep_extrude の path 引数に渡すと型エラーになること。
    #[test]
    fn test_sketch_rejected_in_sweep_path_slot() {
        let profile = xy(make_polygon_term(vec![(0, 0), (5, 0), (5, 5), (0, 5)]));
        let bogus_path = make_polygon_term(vec![(0, 0), (10, 0), (10, 10)]);
        let term = struc("sweep_extrude".into(), vec![profile, bogus_path]);
        let err = Model3D::from_term(&term).unwrap_err();
        assert!(
            matches!(err, ConversionError::TypeMismatch { .. }),
            "expected TypeMismatch, got {:?}",
            err
        );
    }

    /// sweep_extrude の profile に rotateTo* がないと MissingPlaneSpecification になる。
    #[test]
    fn test_sweep_extrude_requires_plane_on_profile() {
        let profile = make_polygon_term(vec![(0, 0), (5, 0), (5, 5), (0, 5)]);
        let path = make_path_term((0, 0), vec![line_to_term(0, 20)]);
        let term = struc("sweep_extrude".into(), vec![profile, path]);
        let err = Model3D::from_term(&term).unwrap_err();
        assert!(
            matches!(
                err,
                ConversionError::MissingPlaneSpecification { ref functor } if functor == "sweep_extrude"
            ),
            "expected MissingPlaneSpecification(sweep_extrude), got {:?}",
            err
        );
    }

    #[test]
    fn test_circle_default_segments() {
        let term: Term = struc("circle".into(), vec![number_int(5)]);
        let expr = Model2D::from_term(&term).unwrap();
        match expr {
            Model2D::Circle { radius } => {
                assert_eq!(radius, 5.0);
            }
            _ => panic!("Expected Circle"),
        }
    }

    fn xy(t: Term) -> Term {
        struc("rotateToXY".into(), vec![t])
    }

    #[test]
    fn test_extrude_polygon() {
        let polygon = xy(make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]));
        let term = struc("linear_extrude".into(), vec![polygon, number_int(3)]);
        let expr = Model3D::from_term(&term).unwrap();
        match expr {
            Model3D::LinearExtrude { profile, height } => {
                assert_eq!(profile.plane, Plane3D::XY);
                assert!(matches!(profile.profile, Model2D::Sketch { .. }));
                assert_eq!(height, 3.0);
            }
            _ => panic!("Expected LinearExtrude"),
        }
    }

    #[test]
    fn test_revolve_circle() {
        let circle: Term = xy(struc("circle".into(), vec![number_int(5)]));
        let term = struc("revolve".into(), vec![circle, number_int(360)]);
        let expr = Model3D::from_term(&term).unwrap();
        match expr {
            Model3D::Revolve { profile, degrees } => {
                assert!(matches!(profile.profile, Model2D::Circle { .. }));
                assert_eq!(degrees, 360.0);
            }
            _ => panic!("Expected Revolve"),
        }
    }

    #[test]
    fn test_extrude_circle() {
        let circle: Term = xy(struc("circle".into(), vec![number_int(5)]));
        let term = struc("linear_extrude".into(), vec![circle, number_int(10)]);
        let expr = Model3D::from_term(&term).unwrap();
        match expr {
            Model3D::LinearExtrude { profile, height } => {
                assert!(matches!(profile.profile, Model2D::Circle { .. }));
                assert_eq!(height, 10.0);
            }
            _ => panic!("Expected LinearExtrude"),
        }
    }

    #[test]
    fn test_polygon_standalone_evaluate() {
        // 2Dプロファイルをトップレベルで3Dとして使う (薄いextrudeになる) には
        // 平面指定 (rotateToXY/YZ/XZ) で明示的にラップする必要がある。
        let term = xy(make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]));
        let expr = Model3D::from_term(&term).unwrap();
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_polygon_standalone_without_plane_errors() {
        // 平面指定なしで2Dプロファイルを3Dコンテキストに置くとエラーになる
        let term = make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]);
        let err = Model3D::from_term(&term).unwrap_err();
        assert!(matches!(
            err,
            ConversionError::MissingPlaneSpecification { .. }
        ));
    }

    #[test]
    fn test_extrude_evaluate() {
        let polygon = xy(make_polygon_term(vec![(1, 0), (0, 0), (0, 1), (1, 1)]));
        let term = struc("linear_extrude".into(), vec![polygon, number_int(3)]);
        let expr = Model3D::from_term(&term).unwrap();
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_polygon_union_to_polygon_rings() {
        // 2つの重なるsketchのunionがpolygon ringsを返す
        let poly_a = make_polygon_term(vec![(0, 0), (2, 0), (2, 2), (0, 2)]);
        let poly_b = make_polygon_term(vec![(1, 1), (3, 1), (3, 3), (1, 3)]);
        let union_term = struc("union".into(), vec![poly_a, poly_b]);
        let expr = Model2D::from_term(&union_term).unwrap();
        let rings = expr.to_polygon_rings();
        assert!(
            rings.is_some(),
            "union of polygons should produce polygon rings"
        );
        let rings = rings.unwrap();
        assert!(!rings.is_empty());
    }

    #[test]
    fn test_polygon_difference_to_polygon_rings() {
        let poly_a = make_polygon_term(vec![(0, 0), (4, 0), (4, 4), (0, 4)]);
        let poly_b = make_polygon_term(vec![(1, 1), (3, 1), (3, 3), (1, 3)]);
        let diff_term = struc("difference".into(), vec![poly_a, poly_b]);
        let expr = Model2D::from_term(&diff_term).unwrap();
        let rings = expr.to_polygon_rings();
        assert!(
            rings.is_some(),
            "difference of polygons should produce polygon rings"
        );
    }

    #[test]
    fn test_polygon_difference_cw_subtrahend() {
        // CW(時計回り)の引く側ポリゴンを使っても CCW と同じ結果になること
        // CCW: (0,0)->(4,0)->(4,4)->(0,4)  CW: (0,0)->(0,4)->(4,4)->(4,0)
        let base = make_polygon_term(vec![(0, 0), (10, 0), (10, 10), (0, 10)]);
        let hole_ccw = make_polygon_term(vec![(0, 0), (5, 0), (5, 5), (0, 5)]);
        let hole_cw = make_polygon_term(vec![(0, 0), (0, 5), (5, 5), (5, 0)]);

        let diff_ccw = struc("difference".into(), vec![base.clone(), hole_ccw]);
        let diff_cw = struc("difference".into(), vec![base, hole_cw]);

        let rings_ccw = Model2D::from_term(&diff_ccw)
            .unwrap()
            .to_polygon_rings()
            .unwrap();
        let rings_cw = Model2D::from_term(&diff_cw)
            .unwrap()
            .to_polygon_rings()
            .unwrap();

        // 両方リングを持つこと
        assert!(!rings_ccw.is_empty());
        assert!(!rings_cw.is_empty());

        // 総頂点数が一致すること(同じ形状)
        let total_ccw: usize = rings_ccw.iter().map(|r| r.len()).sum();
        let total_cw: usize = rings_cw.iter().map(|r| r.len()).sum();
        assert_eq!(
            total_ccw, total_cw,
            "CW subtrahend should produce same shape as CCW"
        );
    }

    #[test]
    fn test_polygon_operator_plus() {
        use crate::parse::ArithOp;
        use crate::parse::arith_expr;

        let poly_a = make_polygon_term(vec![(0, 0), (2, 0), (2, 2), (0, 2)]);
        let poly_b = make_polygon_term(vec![(1, 1), (3, 1), (3, 3), (1, 3)]);
        let add_term = arith_expr(ArithOp::Add, poly_a, poly_b);
        let expr = Model2D::from_term(&add_term).unwrap();
        assert!(matches!(expr, Model2D::Union(_, _)));
        let rings = expr.to_polygon_rings();
        assert!(rings.is_some());
    }

    #[test]
    fn test_extrude_polygon_boolean() {
        // linear_extrude(sketch(...) + circle(...), height) が動作する
        let poly = make_polygon_term(vec![(0, 0), (5, 0), (5, 5), (0, 5)]);
        let circle: Term = struc("circle".into(), vec![number_int(3)]);
        let union_term: Term = struc("union".into(), vec![poly, circle]);
        let extrude_term = struc(
            "linear_extrude".into(),
            vec![xy(union_term), number_int(10)],
        );
        let expr = Model3D::from_term(&extrude_term).unwrap();
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_chained_polygon_difference_extrude() {
        // (rect - rect - rect) |> linear_extrude のケース
        let poly_a = make_polygon_term(vec![(-10, -10), (10, -10), (10, 10), (-10, 10)]);
        let poly_b = make_polygon_term(vec![(-6, -6), (6, -6), (6, 6), (-6, 6)]);
        let poly_c = make_polygon_term(vec![(-10, -3), (10, -3), (10, 3), (-10, 3)]);
        let diff1 = struc("difference".into(), vec![poly_a, poly_b]);
        let diff2 = xy(struc("difference".into(), vec![diff1, poly_c]));
        let extrude = struc("linear_extrude".into(), vec![diff2, number_int(50)]);
        let expr = Model3D::from_term(&extrude).unwrap();
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);
        assert_eq!(mesh.num_props(), 6); // xyz + normals
    }

    #[test]
    fn test_chained_difference_with_path() {
        // polygon - polygon をPathと組み合わせて使う。
        // sweep_extrude の profile は PlacedSketch 必須なので rotateToXY で包む。
        let poly = make_polygon_term(vec![(0, 0), (10, 0), (10, 10), (0, 10)]);
        let hole = make_polygon_term(vec![(2, 2), (8, 2), (8, 8), (2, 8)]);
        let diff = xy(struc("difference".into(), vec![poly, hole]));
        let path = make_path_term((0, 0), vec![line_to_term(10, 5), line_to_term(20, 0)]);
        let sweep = struc("sweep_extrude".into(), vec![diff, path]);
        let expr = Model3D::from_term(&sweep).unwrap();
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
            Triangle {
                normal: Normal::new([0.0, 0.0, -1.0]),
                vertices: [v0, v2, v1],
            },
            Triangle {
                normal: Normal::new([0.0, -1.0, 0.0]),
                vertices: [v0, v1, v3],
            },
            Triangle {
                normal: Normal::new([-1.0, 0.0, 0.0]),
                vertices: [v0, v3, v2],
            },
            Triangle {
                normal: Normal::new([1.0, 1.0, 1.0]),
                vertices: [v1, v2, v3],
            },
        ];

        let dir = std::env::temp_dir().join("cadhr_test_stl");
        std::fs::create_dir_all(&dir).unwrap();
        let stl_path = dir.join("test.stl");
        {
            let mut file = std::fs::File::create(&stl_path).unwrap();
            stl_io::write_stl(&mut file, tris.iter()).unwrap();
        }

        let term: Term = struc(
            "stl".into(),
            vec![string_lit(stl_path.to_str().unwrap().into())],
        );
        let expr = Model3D::from_term(&term).unwrap();
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn test_extract_control_points() {
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(10), number_int(20), number_int(30)],
        );
        let cp1 = struc(
            "control3d".into(),
            vec![struc(
                "p".into(),
                vec![number_int(1), number_int(2), number_int(3)],
            )],
        );
        let cp2 = struc(
            "control3d".into(),
            vec![
                struc(
                    "p".into(),
                    vec![number_int(4), number_int(5), number_int(6)],
                ),
                string_lit("origin".into()),
            ],
        );
        let mut terms = vec![cube, cp1, cp2];
        let cps = extract_control_points(&mut terms, &Default::default());
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
            "control3d".into(),
            vec![struc(
                "p".into(),
                vec![var("X".into()), number_int(0), number_int(0)],
            )],
        );
        let mut terms = vec![cube, cp];
        let cps = extract_control_points(&mut terms, &Default::default());
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
            "main :- linear_extrude(rotateToXY(sketch(p(0, 0), [line_to(p(0, 40)), line_to(p(30, 0))])), X@10), control3d(p(X, 0, 0), \"width\")."
        ).unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());

        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].name.as_deref(), Some("width"));
        // X=10 のデフォルト値がcontrolにも伝播
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

        // X=なし: controlのVar座標が0にフォールバックし、extrude側にも0が代入される
        let mut db = database(
            "main :- linear_extrude(rotateToXY(sketch(p(0, 0), [line_to(p(0, 40)), line_to(p(30, 0))])), X), control3d(p(X, -10, -10)).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());

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
            "main :- sketch(p(0,0), [line_to(p(0,40)), line_to(p(30,0))]) |> rotateToXY |> linear_extrude(X+1), control3d(p(X, -10, -10)).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());

        assert_eq!(cps.len(), 1);
        assert_eq!(resolved.len(), 1);
        let _result = generate_mesh_and_tree_from_terms(&resolved, &[]).unwrap();
    }

    #[test]
    fn test_control_override_preserves_var_names() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let src = "main :- sketch(p(0,0), [line_to(p(0,40)), line_to(p(30,0))]) |> rotateToXY |> linear_extrude(X+1), control3d(p(X, -10, -10)).";
        let mut db = database(src).unwrap();
        let (_, q) = parse_query("main.").unwrap();

        // 初回: overridesなし
        let (mut resolved, _) = execute(&mut db, q.clone()).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].var_names[0], Some("X".to_string()));
        assert_eq!(cps[0].x.value, 0.0); // Varフォールバック

        // 2回目: X=5.0でoverride → var_namesが保持されること
        let mut db2 = database(src).unwrap();
        let (_, q2) = parse_query("main.").unwrap();
        let (mut resolved2, _) = execute(&mut db2, q2).unwrap();
        let overrides = std::collections::HashMap::from([("X".to_string(), 5.0)]);
        let cps2 = extract_control_points(&mut resolved2, &overrides);
        assert_eq!(cps2.len(), 1);
        assert_eq!(cps2[0].var_names[0], Some("X".to_string()));
        assert_eq!(cps2[0].x.value, 5.0);
        // 残りのtermsでextrude(sketch(...), 6)になっていること
        assert_eq!(resolved2.len(), 1);
        let (mesh, _) = generate_mesh_and_tree_from_terms(&resolved2, &[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_control_is_builtin_functor() {
        assert!(crate::term_processor::is_builtin_functor("control2d"));
        assert!(crate::term_processor::is_builtin_functor("control3d"));
    }

    #[test]
    fn test_extract_control_2d() {
        // control2d(p(X, Y)) は z=0、var_names[2]=None の ControlPoint を生成。
        let cube: Term = struc(
            "cube".into(),
            vec![number_int(10), number_int(20), number_int(30)],
        );
        let cp = struc(
            "control2d".into(),
            vec![struc(
                "p".into(),
                vec![number_int(7), number_int(11)],
            )],
        );
        let mut terms = vec![cube, cp];
        let cps = extract_control_points(&mut terms, &Default::default());
        assert_eq!(terms.len(), 1);
        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].x.value, 7.0);
        assert_eq!(cps[0].y.value, 11.0);
        assert_eq!(cps[0].z.value, 0.0);
        assert!(cps[0].var_names[0].is_none());
        assert!(cps[0].var_names[1].is_none());
        assert!(cps[0].var_names[2].is_none());
    }

    #[test]
    fn test_extract_control_2d_with_var_and_name() {
        // control2d(p(X, Y), "anchor") で X 軸の Var 名が追跡されること
        let cp = struc(
            "control2d".into(),
            vec![
                struc(
                    "p".into(),
                    vec![var("X".into()), number_int(5)],
                ),
                string_lit("anchor".into()),
            ],
        );
        let mut terms = vec![cp];
        let cps = extract_control_points(&mut terms, &Default::default());
        assert_eq!(terms.len(), 0);
        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].name.as_deref(), Some("anchor"));
        assert_eq!(cps[0].x.value, 0.0); // Var はデフォルト 0
        assert_eq!(cps[0].y.value, 5.0);
        assert_eq!(cps[0].z.value, 0.0);
        assert_eq!(cps[0].var_names[0].as_deref(), Some("X"));
        assert!(cps[0].var_names[1].is_none());
        assert!(cps[0].var_names[2].is_none());
    }

    #[test]
    fn test_extract_control_2d_with_override() {
        // override が control2d の Var に効くこと
        let cp = struc(
            "control2d".into(),
            vec![struc(
                "p".into(),
                vec![var("X".into()), var("Y".into())],
            )],
        );
        let mut terms = vec![cp];
        let mut overrides = std::collections::HashMap::new();
        overrides.insert("X".to_string(), 3.0);
        overrides.insert("Y".to_string(), 7.5);
        let cps = extract_control_points(&mut terms, &overrides);
        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].x.value, 3.0);
        assert_eq!(cps[0].y.value, 7.5);
        assert_eq!(cps[0].z.value, 0.0);
    }

    /// `control2d(BareVar)` は per-axis Var に unify される。bare Var の annotation は fresh
    /// Var に伝搬し、range 注釈があれば axis_ranges に入る。
    #[test]
    fn test_control_2d_unify_bare_var_with_range() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- cube(1,1,1), control2d(-100<CENTER<100).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 1);
        // axis 名は `<bare>.<axis>#<scope>` 形式。スコープ ID 部分は内部実装に依存するので
        // prefix だけ確認する。
        assert!(cps[0].var_names[0].as_deref().unwrap().starts_with("CENTER.x#"));
        assert!(cps[0].var_names[1].as_deref().unwrap().starts_with("CENTER.y#"));
        assert_eq!(cps[0].axis_ranges[0], Some((-100.0, 100.0)));
        assert_eq!(cps[0].axis_ranges[1], Some((-100.0, 100.0)));
        // midpoint = 0
        assert_eq!(cps[0].x.value, 0.0);
        assert_eq!(cps[0].y.value, 0.0);
    }

    /// battery_case と同じく CENTER が複数箇所で参照されつつ control2d で range 付き。
    /// 同じ Var の bare 参照があっても range 情報が control2d 側で保持される。
    #[test]
    fn test_control_2d_unify_multi_reference_with_range() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- battery_box |> center3d(p(0,0,0)).
             battery_box :-
               (((sketch(p(0,0), [line_to(p(X@20,0)), line_to(p(X,Y@58)), line_to(p(0,Y))]) |> center2d(CENTER))
                 - inner(CENTER)) |> rotateToYZ |> linear_extrude(20))
                 + (inner(CENTER) |> rotateToYZ |> linear_extrude(2)),
               control2d(-100<CENTER<100).
             inner(CENTER) :- sketch(p(0,0), [line_to(p(XIN@14.6,0)), line_to(p(XIN,INNERLEN@49.8)), line_to(p(0,INNERLEN))])
                |> center2d(CENTER).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 1);
        assert_eq!(
            cps[0].axis_ranges[0],
            Some((-100.0, 100.0)),
            "control2d's range annotation should reach the ControlPoint even when CENTER \
             is referenced as bare Var elsewhere"
        );
        assert_eq!(cps[0].axis_ranges[1], Some((-100.0, 100.0)));
    }

    /// 上のテストと同じ multi-reference 構造のまま、bare-Var unify 後に Model3D へ変換
    /// できる (= user-defined predicate 経由で渡された CENTER も per-axis Var に置換され、
    /// center2d の引数として値を取り出せる) ことを保証する。
    #[test]
    fn test_control_2d_unify_multi_reference_mesh_generation() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- battery_box |> center3d(p(0,0,0)).
             battery_box :-
               (((sketch(p(0,0), [line_to(p(X@20,0)), line_to(p(X,Y@58)), line_to(p(0,Y))]) |> center2d(CENTER))
                 - inner(CENTER)) |> rotateToYZ |> linear_extrude(20))
                 + (inner(CENTER) |> rotateToYZ |> linear_extrude(2)),
               control2d(-100<CENTER<100).
             inner(CENTER) :- sketch(p(0,0), [line_to(p(XIN@14.6,0)), line_to(p(XIN,INNERLEN@49.8)), line_to(p(0,INNERLEN))])
                |> center2d(CENTER).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let _cps = extract_control_points(&mut resolved, &Default::default());
        let exprs: Vec<Model3D> = resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .collect::<Result<Vec<_>, _>>()
            .unwrap_or_else(|e| {
                panic!(
                    "Model3D conversion should not fail after control2d bare-Var unify: {:?}\n\
                     resolved terms: {:#?}",
                    e, resolved
                )
            });
        assert!(!exprs.is_empty());
    }

    /// 異なるスコープで同名 (POS) を別目的に使うケース。bare-Var unify の substitute は
    /// 同一名・同一スコープに限定されるべき (別スコープの POS を巻き込まない)。
    /// foo は POS@(0,0,0) を control3d で unify、bar は別の POS をそのまま translate に
    /// 渡す。bar の POS は明示的に別の値で bind される。
    #[test]
    fn test_control_3d_bare_unify_does_not_cross_scopes() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- foo + bar.
             foo :- cube(1, 1, 1), control3d(POS).
             bar :- cube(2, 2, 2) |> translate(p(0,0,0), POS),
               POS = p(5, 0, 0).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 1);
        // foo の control3d は POS を p(POS.x#<scope>, POS.y#<scope>, POS.z#<scope>) に unify
        assert!(cps[0].var_names[0].as_deref().unwrap().starts_with("POS.x#"));
        // bar 側の POS は CENTER = p(5, 0, 0) で別の値に bind されている。
        // 変換が通れば(エラーなく Model3D になれば) cross-scope 干渉していない。
        let _exprs: Vec<Model3D> = resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .collect::<Result<Vec<_>, _>>()
            .unwrap_or_else(|e| panic!("Model3D conversion should not fail: {:?}", e));
    }

    /// 別スコープで同名の CENTER がそれぞれ独立に bare-Var unify される場合、ControlPoint
    /// および axis var 名・axis_ranges がスコープごとに別であるべき。同名衝突で drag が
    /// 巻き込まれたり range が混在したりしてはいけない。
    #[test]
    fn test_control_2d_distinct_scopes_with_same_name() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- foo + bar.
             foo :- cube(1, 1, 1), control2d(-100<CENTER<100).
             bar :- cube(2, 2, 2), control2d(0<CENTER<50).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 2, "両方の control2d がそれぞれ独立に抽出されるべき");

        let ranges: Vec<_> = cps.iter().map(|c| c.axis_ranges[0]).collect();
        assert!(
            ranges.contains(&Some((-100.0, 100.0))),
            "foo's range should be present"
        );
        assert!(
            ranges.contains(&Some((0.0, 50.0))),
            "bar's range should be present"
        );

        // 各 ControlPoint の axis var 名はスコープ間で衝突してはならない。
        // 同名なら GUI override map のキー (例 "CENTER.x") が共有されて、片方の drag が
        // もう片方に伝播してしまう。
        let names: Vec<_> = cps
            .iter()
            .map(|c| c.var_names[0].clone().unwrap_or_default())
            .collect();
        assert_ne!(
            names[0], names[1],
            "axis var names must be distinct across scopes (got both '{}'). \
             Otherwise GUI override-map keys collide and dragging one CP also moves the other.",
            names[0]
        );
    }

    /// 明示形 `p(0<X<100, 50<Y<150)` で軸ごとに異なる range が axis_ranges に入る。
    #[test]
    fn test_control_2d_named_per_axis_ranges() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db =
            database("main :- cube(1,1,1), control2d(p(0<X<100, 50<Y<150)).").unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 1);
        // 各軸の midpoint
        assert_eq!(cps[0].x.value, 50.0);
        assert_eq!(cps[0].y.value, 100.0);
        // axis_ranges は各軸独立
        assert_eq!(cps[0].axis_ranges[0], Some((0.0, 100.0)));
        assert_eq!(cps[0].axis_ranges[1], Some((50.0, 150.0)));
        assert!(cps[0].axis_ranges[2].is_none());
    }

    /// 明示形 + 負の range
    #[test]
    fn test_control_3d_named_per_axis_negative_range() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- cube(1,1,1), control3d(p(-100<X<-50, -10<Y<10, 0<Z<200)).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].axis_ranges[0], Some((-100.0, -50.0)));
        assert_eq!(cps[0].axis_ranges[1], Some((-10.0, 10.0)));
        assert_eq!(cps[0].axis_ranges[2], Some((0.0, 200.0)));
    }

    #[test]
    fn test_resolved_var_names_after_execute() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        // クエリの変数名を確認
        let mut db =
            database("box(X) :- cube(X, X, X).\nmain :- box(10), box(20), control3d(p(X, 0, 0)).")
                .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        eprintln!("case1: {:?}", resolved);

        // 2つのcontrolが同じ変数名Xを使うケース
        let mut db2 = database(
            "main :- cube(X+Y, 20, 30), control3d(p(X, 0, 0)), control3d(p(Y, 0, 0)).",
        )
        .unwrap();
        let (_, q2) = parse_query("main.").unwrap();
        let (resolved2, _) = execute(&mut db2, q2).unwrap();
        eprintln!("case2: {:?}", resolved2);

        // ルール経由で同名変数が複数スコープに存在するケース
        let mut db3 = database(
            "helper(X) :- cube(X, X, X), control3d(p(X, 0, 0)).\nmain :- helper(10), helper(20).",
        )
        .unwrap();
        let (_, q3) = parse_query("main.").unwrap();
        let (resolved3, _) = execute(&mut db3, q3).unwrap();
        eprintln!("case3: {:?}", resolved3);
    }

    #[test]
    fn test_apply_var_overrides() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;
        use std::collections::HashMap;

        let mut db = database("main :- cube(X+10, 20, 30), control3d(p(X, 0, 0)).").unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();

        let mut overrides = HashMap::new();
        overrides.insert("X".to_string(), 5.0);
        apply_var_overrides(&mut resolved, &overrides);

        let cps = extract_control_points(&mut resolved, &Default::default());
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
        let mut db =
            database("box(X) :- cube(X, X, X).\nmain :- box(10), box(20), control3d(p(X, 0, 0)).")
                .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();

        let mut overrides = HashMap::new();
        overrides.insert("X".to_string(), 5.0);
        apply_var_overrides(&mut resolved, &overrides);

        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 1);
        // box(10)→cube(10,10,10), box(20)→cube(20,20,20) が残るはず
        assert_eq!(resolved.len(), 2);
        let (mesh, _) = generate_mesh_and_tree_from_terms(&resolved, &[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    fn make_path_term(start: (i64, i64), segments: Vec<Term>) -> Term {
        let start_point = struc("p".into(), vec![number_int(start.0), number_int(start.1)]);
        struc(
            "path".into(),
            vec![start_point, crate::parse::list(segments, None)],
        )
    }

    fn line_to_term(x: i64, y: i64) -> Term {
        struc(
            "line_to".into(),
            vec![struc("p".into(), vec![number_int(x), number_int(y)])],
        )
    }

    fn bezier_to_quad_term(cp: (i64, i64), end: (i64, i64)) -> Term {
        struc(
            "bezier_to".into(),
            vec![
                struc("p".into(), vec![number_int(cp.0), number_int(cp.1)]),
                struc("p".into(), vec![number_int(end.0), number_int(end.1)]),
            ],
        )
    }

    fn bezier_to_cubic_term(cp1: (i64, i64), cp2: (i64, i64), end: (i64, i64)) -> Term {
        struc(
            "bezier_to".into(),
            vec![
                struc("p".into(), vec![number_int(cp1.0), number_int(cp1.1)]),
                struc("p".into(), vec![number_int(cp2.0), number_int(cp2.1)]),
                struc("p".into(), vec![number_int(end.0), number_int(end.1)]),
            ],
        )
    }

    /// path を sweep_extrude の path 引数経由で構築できること & 期待した頂点列が
    /// 得られること。`Args::term_path` 経由でないと Path 値は得られないので、sweep_extrude
    /// 経由でテストする。
    #[test]
    fn test_path_line_to_only() {
        let profile = xy(make_polygon_term(vec![(0, 0), (1, 0), (1, 1)]));
        let path = make_path_term(
            (0, 0),
            vec![
                line_to_term(10, 0),
                line_to_term(10, 10),
                line_to_term(0, 10),
            ],
        );
        let term = struc("sweep_extrude".into(), vec![profile, path]);
        let expr = Model3D::from_term(&term).unwrap();
        match &expr {
            Model3D::SweepExtrude { path, .. } => {
                assert_eq!(path.points.len(), 4);
                assert_eq!(
                    path.points,
                    vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
                );
            }
            _ => panic!("Expected SweepExtrude"),
        }
    }

    #[test]
    fn test_path_quadratic_bezier() {
        let profile = xy(make_polygon_term(vec![(0, 0), (1, 0), (1, 1)]));
        let path = make_path_term((0, 0), vec![bezier_to_quad_term((5, 10), (10, 0))]);
        let term = struc("sweep_extrude".into(), vec![profile, path]);
        let expr = Model3D::from_term(&term).unwrap();
        match &expr {
            Model3D::SweepExtrude { path, .. } => {
                // start(1) + 16 bezier steps = 17 points
                assert_eq!(path.points.len(), 17);
                assert_eq!(path.points[0], (0.0, 0.0));
                assert_close!(path.points[16].0, 10.0);
                assert_close!(path.points[16].1, 0.0);
            }
            _ => panic!("Expected SweepExtrude"),
        }
    }

    #[test]
    fn test_path_cubic_bezier() {
        let profile = xy(make_polygon_term(vec![(0, 0), (1, 0), (1, 1)]));
        let path = make_path_term(
            (0, 0),
            vec![bezier_to_cubic_term((5, 10), (10, 10), (10, 0))],
        );
        let term = struc("sweep_extrude".into(), vec![profile, path]);
        let expr = Model3D::from_term(&term).unwrap();
        match &expr {
            Model3D::SweepExtrude { path, .. } => {
                assert_eq!(path.points.len(), 17);
                assert_close!(path.points[16].0, 10.0);
                assert_close!(path.points[16].1, 0.0);
            }
            _ => panic!("Expected SweepExtrude"),
        }
    }

    #[test]
    fn test_path_mixed_segments() {
        let profile = xy(make_polygon_term(vec![(0, 0), (1, 0), (1, 1)]));
        let path = make_path_term(
            (0, 0),
            vec![
                line_to_term(10, 0),
                bezier_to_quad_term((15, 5), (10, 10)),
                bezier_to_cubic_term((5, 15), (0, 10), (0, 0)),
            ],
        );
        let term = struc("sweep_extrude".into(), vec![profile, path]);
        let expr = Model3D::from_term(&term).unwrap();
        match &expr {
            Model3D::SweepExtrude { path, .. } => {
                // start(1) + line(1) + quad(16) + cubic(16) = 34 points
                assert_eq!(path.points.len(), 34);
            }
            _ => panic!("Expected SweepExtrude"),
        }
    }

    #[test]
    fn test_sweep_extrude_line() {
        let profile = xy(make_polygon_term(vec![(0, 0), (5, 0), (5, 5), (0, 5)]));
        let path = make_path_term((0, 0), vec![line_to_term(0, 20)]);
        let term = struc("sweep_extrude".into(), vec![profile, path]);
        let expr = Model3D::from_term(&term).unwrap();
        assert!(matches!(&expr, Model3D::SweepExtrude { .. }));
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_sweep_extrude_curve() {
        let profile = xy(make_polygon_term(vec![(0, 0), (3, 0), (0, 3)]));
        let path = make_path_term(
            (0, 0),
            vec![bezier_to_cubic_term((5, 0), (10, 10), (10, 20))],
        );
        let term = struc("sweep_extrude".into(), vec![profile, path]);
        let expr = Model3D::from_term(&term).unwrap();
        let mesh = expr.to_mesh(&[]).unwrap();
        assert!(mesh.vertices().len() > 0);
    }

    #[test]
    fn test_center3d_with_translate() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- cube(10,10,10) |> translate(p(0,0,0), p(5,0,0)) |> center3d(p(0,0,0)).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let exprs: Vec<Model3D> = resolved
            .iter()
            .filter_map(|t| Model3D::from_term(t).ok())
            .collect();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        assert_eq!(node.aabb_min[0], -5.0);
        assert_eq!(node.aabb_min[1], -5.0);
        assert_eq!(node.aabb_min[2], -5.0);
        assert_eq!(node.aabb_max[0], 5.0);
        assert_eq!(node.aabb_max[1], 5.0);
        assert_eq!(node.aabb_max[2], 5.0);
    }

    #[test]
    fn test_center3d_with_control() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- control3d(p(X@0, Y@0, Z@0)), cube(10,10,10) |> center3d(p(X, Y, Z)).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (mut resolved, _) = execute(&mut db, q).unwrap();
        let cps = extract_control_points(&mut resolved, &Default::default());
        assert_eq!(cps.len(), 1);
        assert_eq!(cps[0].x.value, 0.0);
        assert_eq!(cps[0].y.value, 0.0);
        assert_eq!(cps[0].z.value, 0.0);
        let exprs: Vec<Model3D> = resolved
            .iter()
            .filter_map(|t| Model3D::from_term(t).ok())
            .collect();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        assert_eq!(node.aabb_min[0], -5.0);
        assert_eq!(node.aabb_min[1], -5.0);
        assert_eq!(node.aabb_min[2], -5.0);
        assert_eq!(node.aabb_max[0], 5.0);
        assert_eq!(node.aabb_max[1], 5.0);
        assert_eq!(node.aabb_max[2], 5.0);
    }

    #[test]
    fn test_center2d_with_circle() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- circle(5) |> center2d(p(10, 20)) |> rotateToXY |> linear_extrude(2).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let exprs: Vec<Model3D> = resolved
            .iter()
            .filter_map(|t| Model3D::from_term(t).ok())
            .collect();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        let cx = (node.aabb_min[0] + node.aabb_max[0]) / 2.0;
        let cy = (node.aabb_min[1] + node.aabb_max[1]) / 2.0;
        assert_close!(cx, 10.0);
        assert_close!(cy, 20.0);
    }

    #[test]
    fn test_center2d_then_rotate_to_yz() {
        // center2d を先に行ってから rotateToYZ する新スタイル。
        // 旧 sketchYZ([...]) |> center2d(0,0) と同じ AABB になる。
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- sketch(p(10,20), [line_to(p(20,20)), line_to(p(20,30)), line_to(p(10,30))]) |> center2d(p(0, 0)) |> rotateToYZ |> linear_extrude(2).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let exprs: Vec<Model3D> = resolved
            .iter()
            .filter_map(|t| Model3D::from_term(t).ok())
            .collect();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        let cy = (node.aabb_min[1] + node.aabb_max[1]) / 2.0;
        let cz = (node.aabb_min[2] + node.aabb_max[2]) / 2.0;
        assert_close!(cy, 0.0);
        assert_close!(cz, 0.0);

        let x_extent = node.aabb_max[0] - node.aabb_min[0];
        let y_extent = node.aabb_max[1] - node.aabb_min[1];
        let z_extent = node.aabb_max[2] - node.aabb_min[2];
        assert_close!(x_extent, 2.0);
        assert_close!(y_extent, 10.0);
        assert_close!(z_extent, 10.0);
    }

    #[test]
    fn test_center2d_with_sketch() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- sketch(p(0,0), [line_to(p(10,0)), line_to(p(10,4)), line_to(p(0,4))]) |> center2d(p(0, 0)) |> rotateToXY |> linear_extrude(1).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let exprs: Vec<Model3D> = resolved
            .iter()
            .filter_map(|t| Model3D::from_term(t).ok())
            .collect();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        assert_close!(node.aabb_min[0], -5.0);
        assert_close!(node.aabb_max[0], 5.0);
        assert_close!(node.aabb_min[1], -2.0);
        assert_close!(node.aabb_max[1], 2.0);
    }

    /// 未束縛の bare Var を point 引数に渡すと TypeMismatch エラーになる。
    /// center2d/center3d/translate は `p(...)` 形 + 値が決まっていることを要求する。
    #[test]
    fn test_bare_var_as_point_errors_center2d() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "inner_part(CENTER) :- sketch(p(0,0), [line_to(p(10,0)), line_to(p(10,4)), line_to(p(0,4))]) |> center2d(CENTER).
             main :- inner_part(CENTER) |> rotateToXY |> linear_extrude(1).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let err = resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .find_map(|r| r.err())
            .expect("expected TypeMismatch for bare CENTER");
        assert!(matches!(err, ConversionError::TypeMismatch { .. }));
    }

    #[test]
    fn test_bare_var_as_point_errors_center3d() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- cube(10,10,10) |> translate(p(0,0,0), p(5,0,0)) |> center3d(P).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let err = resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .find_map(|r| r.err())
            .expect("expected TypeMismatch for bare P");
        assert!(matches!(err, ConversionError::TypeMismatch { .. }));
    }

    #[test]
    fn test_bare_var_as_point_errors_translate_src() {
        let cube_t = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let dst = struc(
            "p".into(),
            vec![number_int(7), number_int(0), number_int(0)],
        );
        let t = struc("translate".into(), vec![cube_t, var("SRC".into()), dst]);
        let result = Model3D::from_term(&t);
        assert!(matches!(result, Err(ConversionError::TypeMismatch { .. })));
    }

    /// CENTER を body 内で明示的に `p(...)` に bind すれば、center2d(CENTER) は
    /// engine の unification で展開されて値が伝播する。
    #[test]
    fn test_var_bound_to_point_works() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(
            "main :- sketch(p(0,0), [line_to(p(10,0)), line_to(p(10,4)), line_to(p(0,4))])
               |> center2d(CENTER) |> rotateToXY |> linear_extrude(1),
               CENTER = p(10, 20).",
        )
        .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let exprs: Vec<Model3D> = resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        // center2d(p(10, 20)) で中心が (10, 20) に移動 → 5 × 2 の矩形が中心 (10, 20)
        assert_close!(node.aabb_min[0], 5.0);
        assert_close!(node.aabb_max[0], 15.0);
        assert_close!(node.aabb_min[1], 18.0);
        assert_close!(node.aabb_max[1], 22.0);
    }

    #[test]
    fn test_bare_var_inside_p_errors() {
        // p(X, 0, 0) の X が bare Var: 値が決まらないのでエラー。
        let cube_t = struc(
            "cube".into(),
            vec![number_int(1), number_int(1), number_int(1)],
        );
        let src = struc(
            "p".into(),
            vec![number_int(0), number_int(0), number_int(0)],
        );
        let dst = struc(
            "p".into(),
            vec![var("X".into()), number_int(5), number_int(0)],
        );
        let t = struc("translate".into(), vec![cube_t, src, dst]);
        let result = Model3D::from_term(&t);
        assert!(matches!(result, Err(ConversionError::TypeMismatch { .. })));
    }

    // ============================================================
    // rotateToXY/YZ/XZ + seal-error tests
    // ============================================================

    fn run_main(src: &str) -> Result<Vec<Model3D>, ConversionError> {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(src).unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .collect::<Result<Vec<_>, _>>()
    }

    #[test]
    fn test_rotate_to_yz_basic() {
        // 10x10の正方形をrotateToYZしてZ方向(=push後の+X)に2押し出し
        let exprs = run_main(
            "main :- sketch(p(0,0), [line_to(p(10,0)), line_to(p(10,10)), line_to(p(0,10))]) |> rotateToYZ |> linear_extrude(2).",
        )
        .unwrap();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        let x_extent = node.aabb_max[0] - node.aabb_min[0];
        let y_extent = node.aabb_max[1] - node.aabb_min[1];
        let z_extent = node.aabb_max[2] - node.aabb_min[2];
        assert_close!(x_extent, 2.0);
        assert_close!(y_extent, 10.0);
        assert_close!(z_extent, 10.0);
    }

    #[test]
    fn test_rotate_to_xz_basic() {
        // 10x10の正方形をrotateToXZ → 押し出しは+Y方向に2
        let exprs = run_main(
            "main :- sketch(p(0,0), [line_to(p(10,0)), line_to(p(10,10)), line_to(p(0,10))]) |> rotateToXZ |> linear_extrude(2).",
        )
        .unwrap();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        let x_extent = node.aabb_max[0] - node.aabb_min[0];
        let y_extent = node.aabb_max[1] - node.aabb_min[1];
        let z_extent = node.aabb_max[2] - node.aabb_min[2];
        assert_close!(x_extent, 10.0);
        assert_close!(y_extent, 2.0);
        assert_close!(z_extent, 10.0);
    }

    /// メッシュの符号付き体積を計算する。法線が外向きなら正、反転していたら負になる。
    /// 三角形の v0·(v1×v2) を全三角形にわたり総和し、6で割る。
    fn signed_mesh_volume(node: &EvaluatedNode) -> f64 {
        // mesh_verts は num_props (=xyz+normal の 6) ごとに区切られたflatなf32配列。
        // xyzは先頭3要素。num_props は EvaluatedNode に保存されていないので、
        // mesh_verts.len() / vertex_count から推定する。
        let vertex_count = node
            .mesh_indices
            .iter()
            .max()
            .map(|m| (*m as usize) + 1)
            .unwrap_or(0);
        assert!(vertex_count > 0, "empty mesh");
        let stride = node.mesh_verts.len() / vertex_count;
        assert!(
            stride >= 3,
            "expected at least xyz per vertex, got stride={stride}"
        );
        let pos = |idx: u32| {
            let base = (idx as usize) * stride;
            [
                node.mesh_verts[base] as f64,
                node.mesh_verts[base + 1] as f64,
                node.mesh_verts[base + 2] as f64,
            ]
        };
        let mut volume = 0.0;
        for tri in node.mesh_indices.chunks_exact(3) {
            let v0 = pos(tri[0]);
            let v1 = pos(tri[1]);
            let v2 = pos(tri[2]);
            let cross = [
                v1[1] * v2[2] - v1[2] * v2[1],
                v1[2] * v2[0] - v1[0] * v2[2],
                v1[0] * v2[1] - v1[1] * v2[0],
            ];
            volume += v0[0] * cross[0] + v0[1] * cross[1] + v0[2] * cross[2];
        }
        volume / 6.0
    }

    #[test]
    fn test_rotate_to_xz_sketch_orientation() {
        // sketch 版でも同様に法線が外向きであること (回帰防止)。
        let exprs = run_main(
            "main :- sketch(p(0,0), [line_to(p(10,0)), line_to(p(10,10)), line_to(p(0,10))]) |> rotateToXZ |> linear_extrude(2).",
        )
        .unwrap();
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        let volume = signed_mesh_volume(&node);
        assert_close!(volume, 200.0);
    }

    #[test]
    fn test_rotate_to_xy_places_on_xy() {
        // rotateToXY を明示すると XY 平面の押し出し (押し出し方向は +Z)
        let exprs = run_main(
            "main :- sketch(p(0,0), [line_to(p(4,0)), line_to(p(4,3)), line_to(p(0,3))]) |> rotateToXY |> linear_extrude(2).",
        )
        .unwrap();
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        let x_extent = node.aabb_max[0] - node.aabb_min[0];
        let y_extent = node.aabb_max[1] - node.aabb_min[1];
        let z_extent = node.aabb_max[2] - node.aabb_min[2];
        assert_close!(x_extent, 4.0);
        assert_close!(y_extent, 3.0);
        assert_close!(z_extent, 2.0);
    }

    #[test]
    fn test_linear_extrude_requires_plane() {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db =
            database("main :- sketch(p(0,0), [line_to(p(4,0)), line_to(p(4,3)), line_to(p(0,3))]) |> linear_extrude(2).")
                .unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let err = resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .find_map(|r| r.err())
            .expect("expected MissingPlaneSpecification");
        assert!(matches!(
            err,
            ConversionError::MissingPlaneSpecification { ref functor } if functor == "linear_extrude"
        ));
    }

    #[test]
    fn test_boolean_then_rotate_to_yz() {
        // (a - b) |> rotateToYZ |> linear_extrude
        // 今回の battery_case のバグ再現ケースが正しく動くこと
        let exprs = run_main(
            "main :- (sketch(p(0,0), [line_to(p(20,0)), line_to(p(20,58)), line_to(p(0,58))]) - sketch(p(2.7,4.1), [line_to(p(17.3,4.1)), line_to(p(17.3,53.9)), line_to(p(2.7,53.9))])) |> rotateToYZ |> linear_extrude(20).",
        )
        .unwrap();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        // YZ平面の押し出しなので押し出し方向は+X
        let x_extent = node.aabb_max[0] - node.aabb_min[0];
        let y_extent = node.aabb_max[1] - node.aabb_min[1];
        let z_extent = node.aabb_max[2] - node.aabb_min[2];
        assert_close!(x_extent, 20.0);
        assert_close!(y_extent, 20.0);
        assert_close!(z_extent, 58.0);
    }

    #[test]
    fn test_circle_rotate_to_yz() {
        // circle(r) |> rotateToYZ |> linear_extrude が YZ 平面の円柱になる
        let exprs = run_main("main :- circle(5) |> rotateToYZ |> linear_extrude(3).").unwrap();
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        let x_extent = node.aabb_max[0] - node.aabb_min[0];
        let y_extent = node.aabb_max[1] - node.aabb_min[1];
        let z_extent = node.aabb_max[2] - node.aabb_min[2];
        assert_close!(x_extent, 3.0);
        // 円の直径=10, 32分割で僅かに小さくなるので緩めに
        assert!((y_extent - 10.0).abs() < 0.2);
        assert!((z_extent - 10.0).abs() < 0.2);
    }

    fn expect_sealed(src: &str, expected_op: &str) {
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let mut db = database(src).unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let err = resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .find_map(|r| r.err())
            .unwrap_or_else(|| panic!("expected RotatedSketchSealed error for src: {}", src));
        match err {
            ConversionError::RotatedSketchSealed { op } => {
                assert_eq!(op, expected_op, "src: {}", src);
            }
            other => panic!("expected RotatedSketchSealed, got {:?}", other),
        }
    }

    #[test]
    fn test_boolean_after_rotate_errors() {
        expect_sealed(
            "main :- (sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ) + sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> linear_extrude(1).",
            "+",
        );
        expect_sealed(
            "main :- sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) + (sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ) |> linear_extrude(1).",
            "+",
        );
        expect_sealed(
            "main :- (sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ) + (sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ) |> linear_extrude(1).",
            "+",
        );
        expect_sealed(
            "main :- (sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ) + (sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToXZ) |> linear_extrude(1).",
            "+",
        );
    }

    #[test]
    fn test_difference_after_rotate_errors() {
        expect_sealed(
            "main :- (sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ) - sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> linear_extrude(1).",
            "-",
        );
    }

    #[test]
    fn test_center2d_after_rotate_errors() {
        expect_sealed(
            "main :- sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ |> center2d(p(0, 0)) |> linear_extrude(1).",
            "center2d",
        );
    }

    #[test]
    fn test_battery_case_style_with_user_defined_inner() {
        // ユーザープロジェクト battery_case と同等の構造:
        // - 外側 boolean に user-defined predicate (honi) が含まれている
        // - boolean の結果を rotateToYZ で平面に置いてから linear_extrude
        // rotateTo* が is_builtin_functor として正しく登録され、term_rewrite で
        // 既定ファンクタとして扱われることをこの統合テストで確認する。
        use crate::parse::{database, query as parse_query};
        use crate::term_rewrite::execute;

        let src = "\
main :- battery_box.
battery_box :-
  (sketch(p(0,0), [line_to(p(20,0)), line_to(p(20,58)), line_to(p(0,58))]) |> center2d(p(0,0)))
  - honi |> rotateToYZ |> linear_extrude(20).
honi :- sketch(p(0,0), [line_to(p(14.6,0)), line_to(p(14.6,49.8)), line_to(p(0,49.8))]) |> center2d(p(0,0)).
";
        let mut db = database(src).unwrap();
        let (_, q) = parse_query("main.").unwrap();
        let (resolved, _) = execute(&mut db, q).unwrap();
        let exprs: Vec<Model3D> = resolved
            .iter()
            .map(|t| Model3D::from_term(t))
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(exprs.len(), 1);
        let node = build_evaluated_node(&exprs[0], &[]).unwrap();
        let x_extent = node.aabb_max[0] - node.aabb_min[0];
        let y_extent = node.aabb_max[1] - node.aabb_min[1];
        let z_extent = node.aabb_max[2] - node.aabb_min[2];
        // YZ平面に押し出した薄板形状: 押し出し方向は+X
        assert_close!(x_extent, 20.0);
        assert_close!(y_extent, 20.0);
        assert_close!(z_extent, 58.0);
    }

    #[test]
    fn test_double_rotate_errors() {
        expect_sealed(
            "main :- sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ |> rotateToXZ |> linear_extrude(1).",
            "rotateToXZ",
        );
        expect_sealed(
            "main :- sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ |> rotateToYZ |> linear_extrude(1).",
            "rotateToYZ",
        );
        expect_sealed(
            "main :- sketch(p(0,0), [line_to(p(1,0)), line_to(p(1,1)), line_to(p(0,1))]) |> rotateToYZ |> rotateToXY |> linear_extrude(1).",
            "rotateToXY",
        );
    }
}
