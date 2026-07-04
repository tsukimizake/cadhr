//! Workspace = 3D プレビュー / 2D スケッチ の混在リスト。
//!
//! 各 workspace は共通の u64 id を持ち、順序は `Vec<Workspace>` 上の位置で表す。

use cadhr_lang::BindingSignature;
use iced::widget::{column, scrollable, text};
use iced::{Element, Fill};

use crate::ui::preview::{self, Preview, PreviewMsg};
use crate::ui::sketch::{self, Sketch, SketchMsg};

pub enum Workspace {
    Preview(Preview),
    Sketch(Sketch),
}

impl Workspace {
    pub fn id(&self) -> u64 {
        match self {
            Workspace::Preview(p) => p.id,
            Workspace::Sketch(s) => s.id,
        }
    }

    pub fn as_preview(&self) -> Option<&Preview> {
        match self {
            Workspace::Preview(p) => Some(p),
            Workspace::Sketch(_) => None,
        }
    }

    pub fn as_preview_mut(&mut self) -> Option<&mut Preview> {
        match self {
            Workspace::Preview(p) => Some(p),
            Workspace::Sketch(_) => None,
        }
    }
}

#[derive(Debug, Clone)]
pub enum WorkspaceMsg {
    Preview(PreviewMsg),
    Sketch(SketchMsg),
}

/// workspace 単体では完結しない、Model レベルの後処理要求。
/// `Workspace::update` が状態遷移を適用した上でこれを返す。
#[derive(Debug)]
pub enum WorkspaceEvent {
    None,
    /// 状態が変わった (セッション未保存化)。
    Edited,
    /// 再評価が必要。`edited` はセッション未保存化も伴うかどうか。
    EvalNeeded { edited: bool },
    /// target binding が変わった。signature デフォルト適用と再評価が必要。
    TargetChanged,
    /// 3MF export 要求。None はエクスポート対象なし。
    ExportRequested(Option<Vec<u8>>),
    /// 生成コードをクリップボードへ。
    CopyRequested(String),
    Close,
    MoveUp,
    MoveDown,
}

impl Workspace {
    pub fn update(&mut self, msg: WorkspaceMsg) -> WorkspaceEvent {
        match (self, msg) {
            (Workspace::Preview(p), WorkspaceMsg::Preview(m)) => p.update(m),
            (Workspace::Sketch(s), WorkspaceMsg::Sketch(m)) => s.update(m),
            // id は型をまたいで再利用されないため、型違いの配送は起きない
            (w, m) => unreachable!("workspace #{} received mismatched message {m:?}", w.id()),
        }
    }
}

pub fn list_view<'a>(
    workspaces: &'a [Workspace],
    candidate_signatures: &'a [BindingSignature],
) -> Element<'a, (u64, WorkspaceMsg)> {
    if workspaces.is_empty() {
        return text("No workspaces. Press \"+ Add Workspace\" to create one.").into();
    }
    let total = workspaces.len();
    let items: Vec<Element<'a, (u64, WorkspaceMsg)>> = workspaces
        .iter()
        .enumerate()
        .map(|(i, w)| {
            let id = w.id();
            match w {
                Workspace::Preview(p) => preview::view(p, i, total, candidate_signatures)
                    .map(move |m| (id, WorkspaceMsg::Preview(m))),
                Workspace::Sketch(s) => {
                    sketch::view(s, i, total).map(move |m| (id, WorkspaceMsg::Sketch(m)))
                }
            }
        })
        .collect();
    scrollable(column(items).spacing(8)).height(Fill).into()
}
