//! 統一 Diagnostic 型と Span。
//!
//! parser / typecheck / evaluator はすべて `Diagnostic` を返し、LSP / GUI 側で
//! 表示用に変換する。spans はソース文字列のバイトオフセット (chumsky の
//! `SimpleSpan` と互換) で持つ。

use std::ops::Range;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Span {
    pub start: usize,
    pub end: usize,
}

impl Span {
    pub const fn new(start: usize, end: usize) -> Self {
        Self { start, end }
    }

    pub const fn empty() -> Self {
        Self { start: 0, end: 0 }
    }

    pub fn range(self) -> Range<usize> {
        self.start..self.end
    }

    pub fn merge(self, other: Span) -> Span {
        Span {
            start: self.start.min(other.start),
            end: self.end.max(other.end),
        }
    }
}

impl From<Range<usize>> for Span {
    fn from(r: Range<usize>) -> Self {
        Span {
            start: r.start,
            end: r.end,
        }
    }
}

impl From<Span> for Range<usize> {
    fn from(s: Span) -> Self {
        s.start..s.end
    }
}

impl From<chumsky::span::SimpleSpan> for Span {
    fn from(s: chumsky::span::SimpleSpan) -> Self {
        Span {
            start: s.start,
            end: s.end,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    Error,
    Warning,
    Info,
}

#[derive(Clone, Debug)]
pub struct RelatedInfo {
    pub span: Span,
    pub message: String,
}

/// すべての診断の種別。compile 由来 (字句/構文/モジュール/型/slider/網羅性) は
/// 種別ごとに variant を持ち、実行時エラーは `Runtime` に集約する。
/// `span()` / `severity()` / `message()` / `related()` で表示用情報を取り出す。
#[derive(Clone, Debug)]
pub enum Diagnostic {
    // ---- 字句 / 構文 ----
    Lex {
        span: Span,
        message: String,
    },
    Syntax {
        span: Span,
        message: String,
    },
    /// パースエラー由来の式に型推論が到達した。
    ParseErrorExpr {
        span: Span,
    },

    // ---- モジュール解決 ----
    ModuleNameConflict {
        name: String,
    },
    MainModuleMissing,
    ModuleCycle {
        span: Span,
        stack: String,
        qname: String,
    },
    ModuleNameMismatch {
        span: Span,
        importer: String,
        declared: String,
    },
    ModuleLoadFailed {
        span: Span,
        qname: String,
        error: String,
    },
    ModuleNotFound {
        span: Span,
        qname: String,
        search_paths: String,
    },
    /// import 先の型情報が未ロード (resolver bug)。
    ImportTypeNotLoaded {
        span: Span,
        qname: String,
    },

    // ---- 型推論 ----
    TypeAliasArity {
        span: Span,
        name: String,
        expected: usize,
        got: usize,
    },
    CtorUndefinedInPattern {
        span: Span,
        name: String,
    },
    /// 未定義の変数。確実に実行時クラッシュするので fatal。
    UndefinedVar {
        span: Span,
        name: String,
    },
    /// 未定義のコンストラクタ。fatal。
    UndefinedCtor {
        span: Span,
        name: String,
    },
    RecordNoField {
        span: Span,
        field: String,
    },
    AliasNoField {
        span: Span,
        alias: String,
        field: String,
    },
    NotARecord {
        span: Span,
        field: String,
        ty: String,
    },
    TypeMismatch {
        span: Span,
        context: String,
        left: String,
        right: String,
    },
    InfiniteType {
        span: Span,
        context: String,
        ty: String,
    },
    RecordFieldMismatch {
        span: Span,
        context: String,
        left_fields: Vec<String>,
        right_fields: Vec<String>,
    },
    /// 型クラスのインスタンスが存在しない (例: `Num String`)。
    NoInstance {
        span: Span,
        context: String,
        class_name: String,
        ty: String,
    },
    /// 型クラス制約が残ったまま型変数を default で解決できなかった。
    /// (現状はあらゆる class に default を持たせているので発生は稀。)
    AmbiguousConstraint {
        span: Span,
        context: String,
        class_name: String,
    },

    // ---- slider ----
    SliderNotRange {
        span: Span,
        name: String,
        got: String,
    },
    SliderNotConst {
        span: Span,
        name: String,
        related: Vec<RelatedInfo>,
    },

    // ---- 網羅性 (warning) ----
    NonExhaustiveAllGuarded {
        span: Span,
    },
    NonExhaustiveMissing {
        span: Span,
        missing: String,
    },

    // ---- 実行時 (集約) ----
    Runtime {
        span: Span,
        message: String,
    },
}

impl Diagnostic {
    /// 実行時エラーは種別を細分しないのでこのヘルパで作る。
    pub fn runtime(span: Span, message: impl Into<String>) -> Self {
        Diagnostic::Runtime {
            span,
            message: message.into(),
        }
    }

    pub fn span(&self) -> Span {
        use Diagnostic::*;
        match self {
            MainModuleMissing | ModuleNameConflict { .. } => Span::empty(),
            Lex { span, .. }
            | Syntax { span, .. }
            | ParseErrorExpr { span }
            | ModuleCycle { span, .. }
            | ModuleNameMismatch { span, .. }
            | ModuleLoadFailed { span, .. }
            | ModuleNotFound { span, .. }
            | ImportTypeNotLoaded { span, .. }
            | TypeAliasArity { span, .. }
            | CtorUndefinedInPattern { span, .. }
            | UndefinedVar { span, .. }
            | UndefinedCtor { span, .. }
            | RecordNoField { span, .. }
            | AliasNoField { span, .. }
            | NotARecord { span, .. }
            | TypeMismatch { span, .. }
            | InfiniteType { span, .. }
            | RecordFieldMismatch { span, .. }
            | NoInstance { span, .. }
            | AmbiguousConstraint { span, .. }
            | SliderNotRange { span, .. }
            | SliderNotConst { span, .. }
            | NonExhaustiveAllGuarded { span }
            | NonExhaustiveMissing { span, .. }
            | Runtime { span, .. } => *span,
        }
    }

    pub fn severity(&self) -> Severity {
        match self {
            Diagnostic::NonExhaustiveAllGuarded { .. }
            | Diagnostic::NonExhaustiveMissing { .. } => Severity::Warning,
            _ => Severity::Error,
        }
    }

    pub fn related(&self) -> &[RelatedInfo] {
        match self {
            Diagnostic::SliderNotConst { related, .. } => related,
            _ => &[],
        }
    }

    pub fn message(&self) -> String {
        use Diagnostic::*;
        match self {
            Lex { message, .. } => format!("字句エラー: {message}"),
            Syntax { message, .. } => format!("構文エラー: {message}"),
            ParseErrorExpr { .. } => "パースエラーがあります".to_string(),
            ModuleNameConflict { name } => {
                format!("モジュール名衝突: `{name}` が複数定義されています")
            }
            MainModuleMissing => "main module が解決結果に含まれない (resolver bug)".to_string(),
            ModuleCycle { stack, qname, .. } => format!("モジュール循環: {stack} → {qname}"),
            ModuleNameMismatch {
                importer, declared, ..
            } => format!(
                "モジュール名不一致: import 側は `{importer}` ですが、ファイルは `{declared}` を宣言しています"
            ),
            ModuleLoadFailed { qname, error, .. } => {
                format!("モジュール `{qname}` の読み込みに失敗: {error}")
            }
            ModuleNotFound {
                qname,
                search_paths,
                ..
            } => {
                format!("モジュール `{qname}` が見つかりません (search_paths: {search_paths})")
            }
            ImportTypeNotLoaded { qname, .. } => {
                format!("import 先 `{qname}` の型情報がまだロードされていません")
            }
            TypeAliasArity {
                name,
                expected,
                got,
                ..
            } => format!("型エイリアス `{name}` は {expected} 個の引数を取りますが {got} 個でした"),
            CtorUndefinedInPattern { name, .. } => format!("コンストラクタ `{name}` は未定義"),
            UndefinedVar { name, .. } => format!("未定義の変数 `{name}`"),
            UndefinedCtor { name, .. } => format!("未定義のコンストラクタ `{name}`"),
            RecordNoField { field, .. } => format!("レコードにフィールド `{field}` がありません"),
            AliasNoField { alias, field, .. } => {
                format!("`{alias}` にフィールド `{field}` がありません")
            }
            NotARecord { field, ty, .. } => {
                format!("レコードでないものから `.{field}` を取れません: {ty}")
            }
            TypeMismatch {
                context,
                left,
                right,
                ..
            } => {
                format!("{context}: 型 `{left}` と `{right}` が一致しません")
            }
            InfiniteType { context, ty, .. } => {
                format!("{context}: 循環型 `{ty}` を作ろうとしています")
            }
            RecordFieldMismatch {
                context,
                left_fields,
                right_fields,
                ..
            } => format!(
                "{context}: レコードのフィールドが一致しません ({left_fields:?} vs {right_fields:?})"
            ),
            NoInstance {
                context,
                class_name,
                ty,
                ..
            } => format!(
                "{context}: 型 `{ty}` は `{class_name}` のインスタンスではありません"
            ),
            AmbiguousConstraint {
                context,
                class_name,
                ..
            } => format!(
                "{context}: 型クラス `{class_name}` 制約が解決できません (型を明示してください)"
            ),
            SliderNotRange { name, got, .. } => {
                format!("slider `{name}`: 右辺が Range 型でない値 ({got}) になっています")
            }
            SliderNotConst { name, .. } => format!(
                "slider `{name}`: 右辺をコンパイル時に評価できません (main の引数や別 slider に依存している可能性)"
            ),
            NonExhaustiveAllGuarded { .. } => {
                "case が非網羅です: 全ての arm に guard があり catch-all が無い".to_string()
            }
            NonExhaustiveMissing { missing, .. } => {
                format!("case が非網羅です: {missing} が漏れています")
            }
            Runtime { message, .. } => message.clone(),
        }
    }
}
