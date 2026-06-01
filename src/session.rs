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

#[derive(Debug, Serialize, Deserialize, Clone, Default)]
pub struct SessionPreview {
    pub preview_id: u64,
    #[serde(default)]
    pub order: usize,
    /// slider 名 → 値。
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

#[derive(Debug, Serialize, Deserialize, Clone, Default)]
pub struct SessionPreviews {
    pub previews: Vec<SessionPreview>,
}

pub fn save_session(
    dir: &Path,
    editor_text: &str,
    previews: &[SessionPreview],
) -> Result<(), String> {
    let _ = std::fs::remove_file(dir);
    std::fs::create_dir_all(dir).map_err(|e| format!("Failed to create directory: {e}"))?;

    let db_path = dir.join("db.cadhr");
    std::fs::write(&db_path, editor_text)
        .map_err(|e| format!("Failed to save db file: {e}"))?;

    let session = SessionPreviews {
        previews: previews.to_vec(),
    };
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
