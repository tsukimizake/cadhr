//! 実行時値 (`Value`)。
//!
//! Phase 3 の実装範囲:
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
    /// 3D 幾何形状 (宣言ツリー)。Phase 4 で manifold-rs に橋渡しする。
    Shape3D(Model3D),
    /// 2D 形状 / 平面など。Phase 5 以降で具体化。
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
/// Phase 4 で `manifold_bridge::evaluate` に渡す。
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
    Empty,
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
            Value::Opaque(tag, _) => write!(f, "<{tag}>"),
        }
    }
}
