use cadhr_lang::parse::SrcSpan;
use iced::advanced::text::highlighter;
use iced::advanced::text::Highlighter;
use std::ops::Range;

#[derive(Clone, PartialEq)]
pub struct Settings {
    pub error_span: Option<SrcSpan>,
    pub has_error: bool,
}

pub struct SpanHighlighter {
    settings: Settings,
    current_line: usize,
    byte_offset: usize,
}

#[derive(Debug, Clone, Copy)]
pub enum Highlight {
    Normal,
    Error,
    Ok,
}

impl Highlighter for SpanHighlighter {
    type Settings = Settings;
    type Highlight = Highlight;
    type Iterator<'a> = std::vec::IntoIter<(Range<usize>, Highlight)>;

    fn new(settings: &Self::Settings) -> Self {
        Self {
            settings: settings.clone(),
            current_line: 0,
            byte_offset: 0,
        }
    }

    fn update(&mut self, new_settings: &Self::Settings) {
        if self.settings != *new_settings {
            self.settings = new_settings.clone();
            self.current_line = 0;
            self.byte_offset = 0;
        }
    }

    fn change_line(&mut self, line: usize) {
        if line < self.current_line {
            self.current_line = 0;
            self.byte_offset = 0;
        }
    }

    fn highlight_line(&mut self, line: &str) -> Self::Iterator<'_> {
        let line_start = self.byte_offset;
        let line_end = line_start + line.len();
        self.byte_offset = line_end + 1; // +1 for newline
        self.current_line += 1;

        if line.is_empty() {
            return vec![(0..0, Highlight::Normal)].into_iter();
        }

        let spans = if let Some(span) = self.settings.error_span {
            let span_start = span.start.max(line_start);
            let span_end = span.end.min(line_end);

            if span_start < span_end {
                let local_start = span_start - line_start;
                let local_end = span_end - line_start;
                let mut result = Vec::new();

                if local_start > 0 {
                    result.push((0..local_start, Highlight::Normal));
                }
                result.push((local_start..local_end, Highlight::Error));
                if local_end < line.len() {
                    result.push((local_end..line.len(), Highlight::Normal));
                }
                result
            } else {
                vec![(0..line.len(), Highlight::Normal)]
            }
        } else if !self.settings.has_error {
            vec![(0..line.len(), Highlight::Ok)]
        } else {
            vec![(0..line.len(), Highlight::Normal)]
        };

        spans.into_iter()
    }

    fn current_line(&self) -> usize {
        self.current_line
    }
}

pub fn format(
    highlight: &Highlight,
    _theme: &iced::Theme,
) -> highlighter::Format<iced::Font> {
    use iced::Color;
    match highlight {
        Highlight::Normal => highlighter::Format::default(),
        Highlight::Error => highlighter::Format {
            color: Some(Color::from_rgb(1.0, 0.3, 0.3)),
            font: None,
        },
        Highlight::Ok => highlighter::Format {
            color: Some(Color::from_rgb(0.4, 0.9, 0.4)),
            font: None,
        },
    }
}
