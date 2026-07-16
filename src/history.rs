//! エディタ全文スナップショットによる undo / redo 履歴。
//!
//! ソースは小さいため全文スナップショットで持つ (sketch のドラッグも
//! 全文置換で書き戻される)。記録単位:
//! - キー入力は直前の入力から [`TYPING_MERGE_WINDOW`] 以内なら 1 単位に合流
//! - sketch のドラッグは 1 ジェスチャ (DragEnd まで) が 1 単位
//! - 構造編集や外部リロードなどの単発操作は常に独立した 1 単位

use std::time::{Duration, Instant};

use iced::widget::text_editor::Cursor;

/// 連続キー入力を 1 undo 単位にまとめる時間幅。
const TYPING_MERGE_WINDOW: Duration = Duration::from_millis(1000);

/// undo スタックの上限。全文スナップショットなのでメモリを抑える。
const MAX_ENTRIES: usize = 200;

/// 記録する編集の種別。合流判定に使う。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EditKind {
    /// エディタへのキー入力。
    Typing,
    /// sketch ハンドルドラッグの 1 ステップ。
    Drag,
    /// 合流しない単発の書き換え (構造編集・外部リロードなど)。
    Oneshot,
}

/// 編集適用前のエディタ状態 1 つ分。
#[derive(Clone, Debug)]
pub struct Snapshot {
    pub text: String,
    pub cursor: Cursor,
}

#[derive(Default)]
pub struct History {
    undo: Vec<Snapshot>,
    redo: Vec<Snapshot>,
    /// 直前に record した編集の種別と時刻。
    group: Option<(EditKind, Instant)>,
}

impl History {
    /// 編集を適用する **直前** の状態を記録する。直前の記録と同一グループに
    /// 合流する場合はスナップショットを積まない (グループ開始時点のものが
    /// 既に積まれている)。いずれの場合も redo は捨てる。
    pub fn record(&mut self, before: Snapshot, kind: EditKind) {
        self.record_at(before, kind, Instant::now());
    }

    fn record_at(&mut self, before: Snapshot, kind: EditKind, now: Instant) {
        self.redo.clear();
        let merge = match (kind, self.group) {
            (EditKind::Typing, Some((EditKind::Typing, last))) => {
                now.duration_since(last) < TYPING_MERGE_WINDOW
            }
            (EditKind::Drag, Some((EditKind::Drag, _))) => true,
            _ => false,
        };
        // 直前のスナップショットと同文なら積まない (テキスト無変化の編集など)
        let duplicate = self.undo.last().is_some_and(|s| s.text == before.text);
        if !merge && !duplicate {
            if self.undo.len() >= MAX_ENTRIES {
                self.undo.remove(0);
            }
            self.undo.push(before);
        }
        self.group = Some((kind, now));
    }

    /// グループ合流を打ち切る (カーソル移動、ドラッグ終了など)。
    pub fn break_group(&mut self) {
        self.group = None;
    }

    /// `current` を redo へ積み、直前のスナップショットを返す。
    pub fn undo(&mut self, current: Snapshot) -> Option<Snapshot> {
        let snap = self.undo.pop()?;
        self.redo.push(current);
        self.group = None;
        Some(snap)
    }

    /// `current` を undo へ積み、取り消した状態を返す。
    pub fn redo(&mut self, current: Snapshot) -> Option<Snapshot> {
        let snap = self.redo.pop()?;
        if self.undo.len() >= MAX_ENTRIES {
            self.undo.remove(0);
        }
        self.undo.push(current);
        self.group = None;
        Some(snap)
    }

    /// セッション切り替え時に履歴を破棄する。
    pub fn clear(&mut self) {
        self.undo.clear();
        self.redo.clear();
        self.group = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use iced::widget::text_editor::Position;

    fn snap(text: &str) -> Snapshot {
        Snapshot {
            text: text.to_string(),
            cursor: Cursor {
                position: Position { line: 0, column: 0 },
                selection: None,
            },
        }
    }

    #[test]
    fn typing_merges_within_window() {
        let mut h = History::default();
        let t0 = Instant::now();
        h.record_at(snap(""), EditKind::Typing, t0);
        h.record_at(snap("a"), EditKind::Typing, t0 + Duration::from_millis(100));
        h.record_at(snap("ab"), EditKind::Typing, t0 + Duration::from_millis(200));
        assert_eq!(h.undo.len(), 1);
        let restored = h.undo(snap("abc")).expect("undo");
        assert_eq!(restored.text, "");
    }

    #[test]
    fn typing_after_window_starts_new_group() {
        let mut h = History::default();
        let t0 = Instant::now();
        h.record_at(snap(""), EditKind::Typing, t0);
        h.record_at(snap("abc"), EditKind::Typing, t0 + TYPING_MERGE_WINDOW * 2);
        assert_eq!(h.undo.len(), 2);
    }

    #[test]
    fn break_group_splits_typing() {
        let mut h = History::default();
        let t0 = Instant::now();
        h.record_at(snap(""), EditKind::Typing, t0);
        h.break_group();
        h.record_at(snap("a"), EditKind::Typing, t0 + Duration::from_millis(100));
        assert_eq!(h.undo.len(), 2);
    }

    #[test]
    fn drag_gesture_is_single_unit() {
        let mut h = History::default();
        let t0 = Instant::now();
        h.record_at(snap("v0"), EditKind::Drag, t0);
        h.record_at(snap("v1"), EditKind::Drag, t0 + Duration::from_secs(5));
        h.record_at(snap("v2"), EditKind::Drag, t0 + Duration::from_secs(10));
        assert_eq!(h.undo.len(), 1, "1 ジェスチャは 1 単位");
        h.break_group();
        h.record_at(snap("v3"), EditKind::Drag, t0 + Duration::from_secs(11));
        assert_eq!(h.undo.len(), 2, "DragEnd 後は別単位");
        let restored = h.undo(snap("v4")).expect("undo");
        assert_eq!(restored.text, "v3");
        let restored = h.undo(snap("v3")).expect("undo");
        assert_eq!(restored.text, "v0");
    }

    #[test]
    fn oneshot_never_merges() {
        let mut h = History::default();
        let t0 = Instant::now();
        h.record_at(snap("a"), EditKind::Oneshot, t0);
        h.record_at(snap("b"), EditKind::Oneshot, t0);
        assert_eq!(h.undo.len(), 2);
    }

    #[test]
    fn undo_redo_roundtrip() {
        let mut h = History::default();
        h.record(snap("v1"), EditKind::Oneshot);
        let restored = h.undo(snap("v2")).expect("undo");
        assert_eq!(restored.text, "v1");
        let restored = h.redo(snap("v1")).expect("redo");
        assert_eq!(restored.text, "v2");
        // redo で undo 側に戻っているのでもう一度 undo できる
        let restored = h.undo(snap("v2")).expect("undo again");
        assert_eq!(restored.text, "v1");
    }

    #[test]
    fn new_record_clears_redo() {
        let mut h = History::default();
        h.record(snap("v1"), EditKind::Oneshot);
        h.undo(snap("v2")).expect("undo");
        h.record(snap("v1"), EditKind::Oneshot);
        assert!(h.redo(snap("x")).is_none());
    }

    #[test]
    fn duplicate_snapshot_not_pushed() {
        let mut h = History::default();
        let t0 = Instant::now();
        h.record_at(snap("same"), EditKind::Oneshot, t0);
        h.record_at(snap("same"), EditKind::Oneshot, t0 + Duration::from_secs(5));
        assert_eq!(h.undo.len(), 1);
    }

    #[test]
    fn empty_history_returns_none() {
        let mut h = History::default();
        assert!(h.undo(snap("x")).is_none());
        assert!(h.redo(snap("x")).is_none());
    }
}
