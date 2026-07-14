//! Elm-like cadhr-lang の AST。
//!
//! 仕様は `LANG_SPEC.md` §3 を参照。設計指針:
//! - 全ノードに `Span` を持たせて診断・LSP で再利用する
//! - Pattern / Expr は値構文木とだけ向き合う。型構文木は `TypeExpr` に分ける
//! - record / ADT / case は全部一級。タプルは採用しない (record で代替)

use crate::diagnostic::Span;

/// 1 つの `.cadhr` ファイルに対応する AST ルート。
#[derive(Clone, Debug)]
pub struct Module {
    /// `module Foo exposing (...)` ヘッダ。省略時は `None` (匿名モジュール扱い)。
    pub header: Option<ModuleHeader>,
    pub imports: Vec<Import>,
    pub decls: Vec<Decl>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct ModuleHeader {
    pub name: ModuleName,
    pub exposing: Exposing,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct ModuleName {
    /// `Std.Gears` なら `["Std", "Gears"]`。
    pub segments: Vec<String>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub enum Exposing {
    /// `exposing (..)`
    All(Span),
    /// `exposing (a, b, Type, Type(..), Type(C1, C2))`
    Some(Vec<ExposingItem>, Span),
}

#[derive(Clone, Debug)]
pub struct ExposingItem {
    pub name: String,
    /// 大文字始まりの場合のみ意味を持つ (型/型エイリアス)。
    pub variant: ExposingVariant,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub enum ExposingVariant {
    /// 値 (小文字始まり) または 型名のみ (`Type`)。
    Bare,
    /// 型のコンストラクタを全部公開 (`Type(..)`)。
    AllCtors,
    /// 型のコンストラクタを指定して公開 (`Type(C1, C2)`)。
    SomeCtors(Vec<String>),
}

#[derive(Clone, Debug)]
pub struct Import {
    pub module: ModuleName,
    pub alias: Option<String>,
    /// import 句に `exposing (...)` が無い場合は `None`。
    pub exposing: Option<Exposing>,
    pub span: Span,
}

/// トップレベル宣言。
#[derive(Clone, Debug)]
pub enum Decl {
    /// `f : T -> U` の型シグネチャだけの行。
    Signature(SignatureDecl),
    /// `f x y = body`
    Value(ValueDecl),
    /// `type T a b = C1 a | C2 b`
    Type(TypeDecl),
    /// `type alias R = { f : T }`
    TypeAlias(TypeAliasDecl),
    /// `slider binding.param = expr`
    Slider(SliderDecl),
}

#[derive(Clone, Debug)]
pub struct SignatureDecl {
    pub name: String,
    pub ty: TypeExpr,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct ValueDecl {
    pub name: String,
    /// 関数定義なら引数パターン、定数定義なら空。
    pub params: Vec<Pattern>,
    pub body: Expr,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct TypeDecl {
    pub name: String,
    pub params: Vec<String>,
    pub constructors: Vec<Constructor>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct Constructor {
    pub name: String,
    pub args: Vec<TypeExpr>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct TypeAliasDecl {
    pub name: String,
    pub params: Vec<String>,
    pub body: TypeExpr,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct SliderDecl {
    /// `slider binding.param = ...` の binding 名。未修飾 `slider param = ...` なら
    /// `None` (パースは通すが `extract_sliders` がエラー報告する)。
    pub binding: Option<String>,
    pub param: String,
    pub body: Expr,
    pub span: Span,
}

/// 値式。
#[derive(Clone, Debug)]
pub enum Expr {
    /// 識別子 (小文字始まり)、または qualified (`Std.cube`)。
    /// segments の長さが 1 なら module 名なしの裸変数。
    Var {
        module: Option<ModuleName>,
        name: String,
        span: Span,
    },
    /// データコンストラクタ (大文字始まり)。
    Ctor {
        module: Option<ModuleName>,
        name: String,
        span: Span,
    },
    Lit(Lit, Span),
    List(Vec<Expr>, Span),
    /// `{ a = e1, b = e2 }` の record literal。
    Record(Vec<RecordField>, Span),
    /// `{ r | a = e1, b = e2 }` の record update。
    RecordUpdate {
        base: Box<Expr>,
        updates: Vec<RecordField>,
        span: Span,
    },
    /// `r.field` の field access。`r` 自体は任意の式。
    Field {
        receiver: Box<Expr>,
        name: String,
        span: Span,
    },
    /// 関数適用 (curried)。`f x y` は `App(App(f, x), y)` 相当。
    App {
        func: Box<Expr>,
        arg: Box<Expr>,
        span: Span,
    },
    /// `\x y -> body`
    Lambda {
        params: Vec<Pattern>,
        body: Box<Expr>,
        span: Span,
    },
    /// `let x = e1; y = e2 in body`
    Let {
        bindings: Vec<ValueDecl>,
        body: Box<Expr>,
        span: Span,
    },
    /// `sketch <bindings> in body end` の sketch DSL ブロック。
    /// binding は逐次スコープ (前方参照不可)。制約は `sema::sketch` が検査する。
    Sketch {
        bindings: Vec<SketchBinding>,
        body: Box<Expr>,
        span: Span,
    },
    /// `if cond then a else b`
    If {
        cond: Box<Expr>,
        then_branch: Box<Expr>,
        else_branch: Box<Expr>,
        span: Span,
    },
    /// `case scrutinee of`
    Case {
        scrutinee: Box<Expr>,
        arms: Vec<CaseArm>,
        span: Span,
    },
    /// 二項演算子 (パイプ / 比較 / 算術 / リスト等)。
    BinOp {
        op: BinOp,
        left: Box<Expr>,
        right: Box<Expr>,
        span: Span,
    },
    /// `-expr` の単項マイナス。Elm に倣ってリテラル以外でも許す。
    Negate(Box<Expr>, Span),
    /// `lo..hi` の Range リテラル。
    Range {
        lo: Box<Expr>,
        hi: Box<Expr>,
        span: Span,
    },
    /// パースエラー回復用のプレースホルダ。typecheck では「エラー型」になる。
    Error(Span),
}

#[derive(Clone, Debug)]
pub struct RecordField {
    pub name: String,
    pub value: Expr,
    pub span: Span,
}

/// sketch ブロック内の 1 束縛。params は取れない。
#[derive(Clone, Debug)]
pub struct SketchBinding {
    pub kind: SketchBindKind,
    pub name: String,
    /// binder 名そのものの span (LSP rename / GUI 書き戻しで使う)。
    pub name_span: Span,
    pub body: Expr,
    pub span: Span,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SketchBindKind {
    /// `var x = <Floatリテラル>`: 逆評価 (GUI ドラッグ) の書き込み対象。
    Var,
    /// `let x = <スカラー式>`: 読み取り専用スカラー。
    Let,
    /// キーワードなし (`poly1 = polygon [...]`): 幾何 / 点 / 線分の束縛。
    Bare,
}

#[derive(Clone, Debug)]
pub struct CaseArm {
    pub pattern: Pattern,
    pub guard: Option<Expr>,
    pub body: Expr,
    pub span: Span,
}

/// パターンマッチで使うパターン。
#[derive(Clone, Debug)]
pub enum Pattern {
    Var(String, Span),
    /// `_`
    Wildcard(Span),
    Lit(Lit, Span),
    /// `Cube x y z` のような ADT コンストラクタパターン。
    Ctor {
        module: Option<ModuleName>,
        name: String,
        args: Vec<Pattern>,
        span: Span,
    },
    /// `[a, b, c]`
    List(Vec<Pattern>, Span),
    /// `a :: rest`
    Cons {
        head: Box<Pattern>,
        tail: Box<Pattern>,
        span: Span,
    },
    /// `{ a, b }` で record のフィールドを取り出すパターン。
    Record(Vec<String>, Span),
    /// `name as inner` のエイリアスパターン (Elm 互換)。
    As {
        inner: Box<Pattern>,
        name: String,
        span: Span,
    },
}

impl Expr {
    pub fn span(&self) -> Span {
        match self {
            Expr::Var { span, .. }
            | Expr::Ctor { span, .. }
            | Expr::Lit(_, span)
            | Expr::List(_, span)
            | Expr::Record(_, span)
            | Expr::RecordUpdate { span, .. }
            | Expr::Field { span, .. }
            | Expr::App { span, .. }
            | Expr::Lambda { span, .. }
            | Expr::Let { span, .. }
            | Expr::Sketch { span, .. }
            | Expr::If { span, .. }
            | Expr::Case { span, .. }
            | Expr::BinOp { span, .. }
            | Expr::Negate(_, span)
            | Expr::Range { span, .. }
            | Expr::Error(span) => *span,
        }
    }
}

/// カリー化された適用 `f a b ...` を (head 名, 引数列) に展開する。
/// head が裸の小文字識別子でなければ head は `None`。
pub fn app_spine(e: &Expr) -> (Option<&str>, Vec<&Expr>) {
    let mut args: Vec<&Expr> = Vec::new();
    let mut cur = e;
    while let Expr::App { func, arg, .. } = cur {
        args.push(arg);
        cur = func;
    }
    args.reverse();
    match cur {
        Expr::Var {
            module: None, name, ..
        } => (Some(name.as_str()), args),
        _ => (None, args),
    }
}

/// 数値・文字列・真偽値リテラル。Range は `Expr::Range` で別に持つ。
#[derive(Clone, Debug, PartialEq)]
pub enum Lit {
    Int(i64),
    Float(f64),
    String(String),
    Bool(bool),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BinOp {
    // 算術
    Add,
    Sub,
    Mul,
    Div,
    // 比較
    Eq,
    NotEq,
    Lt,
    Le,
    Gt,
    Ge,
    // 論理
    And,
    Or,
    // リスト
    Cons,
    Append,
    // 関数合成
    Compose,
    ComposeR,
    // 関数適用
    ApplyL,
    ApplyR,
}

/// 型構文木。typecheck で `Type` に解決する前段階。
#[derive(Clone, Debug)]
pub enum TypeExpr {
    /// 小文字始まりの型変数。
    Var(String, Span),
    /// 大文字始まりの型コンストラクタ。`Int`, `List`, `Range`, `Shape3D` など。
    /// 引数を伴う場合 (`List Int`, `Range Float`) は `args` に置く。
    Con {
        module: Option<ModuleName>,
        name: String,
        args: Vec<TypeExpr>,
        span: Span,
    },
    /// `a -> b` の関数型。
    Arrow {
        from: Box<TypeExpr>,
        to: Box<TypeExpr>,
        span: Span,
    },
    /// `{ a : Int, b : String }` の record 型。
    Record(Vec<RecordTypeField>, Span),
}

#[derive(Clone, Debug)]
pub struct RecordTypeField {
    pub name: String,
    pub ty: TypeExpr,
    pub span: Span,
}
