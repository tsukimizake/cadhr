//! セッション (db.cadhr + previews.json) の保存・読み込み。
//!
//! セッションはディレクトリ単位:
//!   - `db.cadhr` — ソースコード
//!   - `previews.json` — プレビュー (slider 値・カメラ視点など) の永続化
//!
//! `~/.local/share/cadhr/last_session_path` に最後のセッションパスを記録し、
//! 起動時に自動復元する。

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

fn default_target_name() -> String {
    "main".to_string()
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SessionPreview {
    pub preview_id: u64,
    #[serde(default)]
    pub order: usize,
    /// 評価する top-level binding の名前。欠落時は `"main"`。
    #[serde(default = "default_target_name")]
    pub target_name: String,
    /// 引数名 → 値 (旧 main slider 値もここに残る — target_name=main で扱えば互換)。
    #[serde(default)]
    pub slider_values: HashMap<String, f64>,
    /// control point 名 → (x, y, z) override。CP ドラッグ後のスナップ位置を保存。
    #[serde(default)]
    pub control_point_overrides: HashMap<String, [f64; 3]>,
    #[serde(default)]
    pub view_at_object_center: bool,
    #[serde(default)]
    pub minimized: bool,
    /// 衝突チェックプレビューかどうか。
    #[serde(default)]
    pub is_collision: bool,
}

fn default_zoom() -> f32 {
    20.0
}

/// 2D sketch workspace の永続化。座標はすべて格子交点スナップ済みの整数
/// (grid 単位)。
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SessionSketch {
    pub sketch_id: u64,
    #[serde(default)]
    pub order: usize,
    #[serde(default)]
    pub minimized: bool,
    /// 1 grid 単位あたりのピクセル数。
    #[serde(default = "default_zoom")]
    pub zoom: f32,
    /// 参照表示する Shape2D binding の名前。空なら紐付けなし。
    #[serde(default)]
    pub target: String,
    #[serde(default)]
    pub shapes: Vec<crate::ui::sketch::SketchShape>,
}

/// SketchV2 workspace (sketch DSL binding との双方向編集) の永続化。
/// 図形はコード側が真なので、紐付け先 binding 名とビュー状態だけ持つ。
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SessionSketchV2 {
    pub sketch_id: u64,
    #[serde(default)]
    pub order: usize,
    #[serde(default)]
    pub minimized: bool,
    /// 1 grid 単位あたりのピクセル数。
    #[serde(default = "default_zoom")]
    pub zoom: f32,
    /// 紐付ける sketch binding の名前。空なら未選択。
    #[serde(default)]
    pub binding: String,
}

#[derive(Debug, Serialize, Deserialize, Clone, Default)]
pub struct SessionPreviews {
    pub previews: Vec<SessionPreview>,
    #[serde(default)]
    pub sketches: Vec<SessionSketch>,
    #[serde(default)]
    pub sketches_v2: Vec<SessionSketchV2>,
}

pub fn save_session(
    dir: &Path,
    editor_text: &str,
    session: &SessionPreviews,
) -> Result<(), String> {
    let _ = std::fs::remove_file(dir);
    std::fs::create_dir_all(dir).map_err(|e| format!("Failed to create directory: {e}"))?;

    let db_path = dir.join("db.cadhr");
    std::fs::write(&db_path, editor_text)
        .map_err(|e| format!("Failed to save db file: {e}"))?;

    let json = serde_json::to_string_pretty(&session)
        .map_err(|e| format!("Failed to serialize: {e}"))?;
    let previews_path = dir.join("previews.json");
    std::fs::write(&previews_path, json)
        .map_err(|e| format!("Failed to save previews: {e}"))?;

    Ok(())
}

pub fn load_session(dir: &Path) -> Option<(String, SessionPreviews)> {
    let db_path = dir.join("db.cadhr");
    let db_content = std::fs::read_to_string(&db_path).ok()?;
    let previews = match std::fs::read_to_string(dir.join("previews.json")) {
        Ok(json) => serde_json::from_str(&json).unwrap_or_default(),
        Err(_) => SessionPreviews::default(),
    };
    Some((db_content, previews))
}

fn last_session_path_file() -> Option<PathBuf> {
    dirs::data_local_dir().map(|d| d.join("cadhr").join("last_session_path"))
}

pub fn save_last_session_path(path: &Path) {
    if let Some(file) = last_session_path_file() {
        if let Some(parent) = file.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let _ = std::fs::write(&file, path.to_string_lossy().as_bytes());
    }
}

pub fn restore_last_session_path() -> Option<PathBuf> {
    let file = last_session_path_file()?;
    let content = std::fs::read_to_string(&file).ok()?;
    let path = PathBuf::from(content.trim());
    if path.join("db.cadhr").exists() {
        Some(path)
    } else {
        None
    }
}
