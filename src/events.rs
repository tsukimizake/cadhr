use bevy::prelude::*;
use cadhr_lang::manifold_bridge::{ControlPoint, EvaluatedNode};
use std::collections::HashMap;
use std::path::PathBuf;

// UI -> CadhrLang: request to generate a preview mesh
#[derive(Message, Clone)]
pub struct GeneratePreviewRequest {
    pub preview_id: u64,
    pub database: String,
    pub query: String,
    pub include_paths: Vec<PathBuf>,
    pub control_point_overrides: HashMap<String, f64>,
}

// CadhrLang -> UI: mesh has been generated for a request
#[derive(Message)]
pub struct PreviewGenerated {
    pub preview_id: u64,
    pub query: String,
    pub mesh: Mesh,
    pub evaluated_nodes: Vec<EvaluatedNode>,
    pub control_points: Vec<ControlPoint>,
}

// CadhrLang -> UI: error or log message from cadhr-lang execution
#[derive(Message)]
pub struct CadhrLangOutput {
    pub message: String,
    pub is_error: bool,
}
