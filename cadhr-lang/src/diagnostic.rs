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
pub struct Diagnostic {
    pub severity: Severity,
    pub span: Span,
    pub code: Option<&'static str>,
    pub message: String,
    pub related: Vec<RelatedInfo>,
}

#[derive(Clone, Debug)]
pub struct RelatedInfo {
    pub span: Span,
    pub message: String,
}

impl Diagnostic {
    pub fn error(span: Span, message: impl Into<String>) -> Self {
        Self {
            severity: Severity::Error,
            span,
            code: None,
            message: message.into(),
            related: Vec::new(),
        }
    }

    pub fn warning(span: Span, message: impl Into<String>) -> Self {
        Self {
            severity: Severity::Warning,
            span,
            code: None,
            message: message.into(),
            related: Vec::new(),
        }
    }

    pub fn with_code(mut self, code: &'static str) -> Self {
        self.code = Some(code);
        self
    }

    pub fn with_related(mut self, related: RelatedInfo) -> Self {
        self.related.push(related);
        self
    }
}
