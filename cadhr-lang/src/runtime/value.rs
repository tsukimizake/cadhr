//! 実行時値 (`Value`)。
//!
//! - 数値 / 文字列 / 真偽値 / リスト / レコード / Range
//! - ユーザー定義 ADT の Constructor (`tag` + `args`)
//! - 関数値 (Closure / Builtin)
//! - 3D 幾何 (`Shape3D`) は宣言的な `Model3D` ツリーとして保持
//!   (manifold-rs への evaluate は GUI 側で行う)

use crate::syntax::ast::Pattern;
use std::collections::HashMap;
use std::fmt;
use std::rc::Rc;

/// 評価器が扱うすべての値。
#[derive(Clone, Debug)]
pub enum Value {
    Int(i64),
    Float(f64),
    String(String),
    Bool(bool),
    List(Vec<Value>),
    Record(Vec<(String, Value)>),
    /// `lo..hi` の範囲。slider 解決時に評価する。
    Range {
        lo: f64,
        hi: f64,
        is_int: bool,
    },
    /// ユーザー定義 ADT のコンストラクタ。
    Ctor { tag: String, args: Vec<Value> },
    /// ユーザー定義関数 (closure)。
    Closure {
        params: Vec<Pattern>,
        body: ClosureBody,
        env: Rc<Env>,
    },
    /// builtin 関数。引数累積方式 (curried)。
    Builtin {
        name: String,
        arity: usize,
        args: Vec<Value>,
    },
    /// 3D 幾何形状 (宣言ツリー)。`manifold_bridge::evaluate` で manifold-rs に橋渡しする。
    Shape3D(Model3D),
    /// 2D 幾何形状 (extrude / revolve の入力)。
    Shape2D(Model2D),
    /// 名前付きの不透明値 (control point ハンドルなど)。
    Opaque(String, Vec<Value>),
}

#[derive(Clone, Debug)]
pub enum ClosureBody {
    Expr(Rc<crate::syntax::ast::Expr>),
}

/// 評価環境 (識別子 → 値)。`Rc` で共有可能、`HashMap` で immutable update。
#[derive(Clone, Debug, Default)]
pub struct Env {
    bindings: HashMap<String, Value>,
}

impl Env {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn extend(&self, name: &str, value: Value) -> Self {
        let mut new = self.clone();
        new.bindings.insert(name.to_string(), value);
        new
    }

    pub fn extend_many<I, S>(&self, iter: I) -> Self
    where
        I: IntoIterator<Item = (S, Value)>,
        S: Into<String>,
    {
        let mut new = self.clone();
        for (n, v) in iter {
            new.bindings.insert(n.into(), v);
        }
        new
    }

    pub fn lookup(&self, name: &str) -> Option<&Value> {
        self.bindings.get(name)
    }
}

/// 3D 幾何形状を表す宣言ツリー (manifold-rs に渡す前の中間表現)。
/// `manifold_bridge::evaluate` で実体化する。
#[derive(Clone, Debug, PartialEq)]
pub enum Model3D {
    Cube {
        x: f64,
        y: f64,
        z: f64,
    },
    Sphere(f64),
    Cylinder {
        r: f64,
        h: f64,
    },
    Tetrahedron,
    Translate {
        shape: Box<Model3D>,
        src: (f64, f64, f64),
        dst: (f64, f64, f64),
    },
    Scale {
        shape: Box<Model3D>,
        factor: (f64, f64, f64),
    },
    Rotate {
        shape: Box<Model3D>,
        angles: (f64, f64, f64),
    },
    Union(Box<Model3D>, Box<Model3D>),
    Diff(Box<Model3D>, Box<Model3D>),
    Intersect(Box<Model3D>, Box<Model3D>),
    Hull(Box<Model3D>, Box<Model3D>),
    /// 2D 形状を指定平面上に置いて高さ方向へ押し出す。
    LinearExtrude {
        profile: Model2D,
        plane: Plane3D,
        height: f64,
    },
    /// 2D 形状を twist (deg) + scale (sx, sy) 付きで押し出す。
    ComplexExtrude {
        profile: Model2D,
        plane: Plane3D,
        height: f64,
        twist: f64,
        scale_x: f64,
        scale_y: f64,
    },
    /// 2D 形状を回転軸まわりに回して 3D に。`degrees` 360 で全周、180 で半周。
    Revolve {
        profile: Model2D,
        plane: Plane3D,
        degrees: f64,
    },
    /// STL ファイルから読み込んだ Mesh。`search_paths` で見つかるパスを期待する。
    Stl {
        path: String,
    },
    /// 2D profile を 3D path (連続 Point3D) に沿って sweep。
    SweepExtrude {
        profile: Model2D,
        plane: Plane3D,
        path: Vec<(f64, f64, f64)>,
    },
    /// BBox 中心を `target` に持ってくる。bridge 内で AABB を計測してから translate する。
    Center3D {
        shape: Box<Model3D>,
        target: (f64, f64, f64),
    },
    Empty,
}

/// 2D 形状の宣言ツリー (manifold-rs の polygon ring に変換する前段)。
#[derive(Clone, Debug, PartialEq)]
pub enum Model2D {
    /// 単一閉路ポリゴン。先頭点と終点が同じでも違ってもよく、bridge 側で閉じる。
    Polygon(Vec<(f64, f64)>),
    /// 2D CSG ノード。
    Union2D(Box<Model2D>, Box<Model2D>),
    Diff2D(Box<Model2D>, Box<Model2D>),
    Intersect2D(Box<Model2D>, Box<Model2D>),
    /// BBox 中心移動。`target` (cx, cy) に bbox 中心を持ってくる。
    Center2D {
        shape: Box<Model2D>,
        target: (f64, f64),
    },
    Empty2D,
}

/// Shape2D を 3D に持ち上げるときの基準平面。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Plane3D {
    XY,
    YZ,
    XZ,
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Int(n) => write!(f, "{n}"),
            Value::Float(x) => write!(f, "{x}"),
            Value::String(s) => write!(f, "\"{s}\""),
            Value::Bool(b) => write!(f, "{}", if *b { "True" } else { "False" }),
            Value::List(items) => {
                write!(f, "[")?;
                for (i, v) in items.iter().enumerate() {
                    if i > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{v}")?;
                }
                write!(f, "]")
            }
            Value::Record(fields) => {
                write!(f, "{{ ")?;
                for (i, (n, v)) in fields.iter().enumerate() {
                    if i > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{n} = {v}")?;
                }
                write!(f, " }}")
            }
            Value::Range { lo, hi, .. } => write!(f, "{lo}..{hi}"),
            Value::Ctor { tag, args } => {
                write!(f, "{tag}")?;
                for a in args {
                    write!(f, " ")?;
                    match a {
                        Value::Ctor { args: aa, .. } if !aa.is_empty() => write!(f, "({a})")?,
                        _ => write!(f, "{a}")?,
                    }
                }
                Ok(())
            }
            Value::Closure { params, .. } => {
                write!(f, "<closure with {} params>", params.len())
            }
            Value::Builtin { name, arity, args } => {
                write!(f, "<builtin {name} ({}/{})>", args.len(), arity)
            }
            Value::Shape3D(m) => write!(f, "<Shape3D {m:?}>"),
            Value::Shape2D(m) => write!(f, "<Shape2D {m:?}>"),
            Value::Opaque(tag, _) => write!(f, "<{tag}>"),
        }
    }
}
