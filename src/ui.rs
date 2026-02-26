use bevy::prelude::*;
use bevy_file_dialog::prelude::*;
use cadhr_lang::manifold_bridge::EvaluatedNode;
use derived_deref::{Deref, DerefMut};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;

pub mod setup;
pub mod update;

pub struct UiPlugin;

pub struct ThreeMfFileContents;

pub struct SessionSaveContents;
pub struct SessionLoadContents;

#[derive(Serialize, Deserialize, Clone)]
pub struct PreviewState {
    #[serde(default)]
    pub preview_id: Option<u64>,
    pub query: String,
    pub zoom: f32,
    pub rotate_x: f64,
    pub rotate_y: f64,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct SessionPreviews {
    pub previews: Vec<PreviewState>,
}

/// Preview target data stored as a component on the root entity.
/// When this entity is despawned, all children are automatically removed.
#[derive(Component, Clone)]
pub struct PreviewTarget {
    pub preview_id: u64,
    pub render_layer: usize,
    pub mesh_handle: Handle<Mesh>,
    pub rt_image: Handle<Image>,
    pub rt_size: UVec2,
    pub camera_entity: Entity,
    pub base_camera_distance: f32,
    pub zoom: f32,
    pub rotate_x: f64,
    pub rotate_y: f64,
    pub query: String,
    pub evaluated_nodes: Vec<EvaluatedNode>,
}

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
        use setup::*;
        use update::{
            egui_ui, handle_cadhr_lang_output, on_preview_generated, restore_last_session,
            session_loaded, session_saved, threemf_saved, update_preview_transforms,
        };

        app.add_plugins(EguiPlugin::default())
            .add_plugins(
                FileDialogPlugin::new()
                    .with_save_file::<SessionSaveContents>()
                    .with_pick_directory::<SessionLoadContents>()
                    .with_save_file::<ThreeMfFileContents>(),
            )
            .add_systems(Startup, (setup, restore_last_session))
            .add_systems(EguiPrimaryContextPass, setup_fonts.run_if(run_once))
            .add_systems(EguiPrimaryContextPass, egui_ui)
            .add_systems(
                Update,
                (
                    on_preview_generated,
                    update_preview_transforms,
                    handle_cadhr_lang_output,
                ),
            )
            .add_systems(Update, (session_saved, session_loaded, threemf_saved))
            .insert_resource(EditorText("main :- cube(10, 20, 30).".to_string()))
            .insert_resource(NextPreviewId::default())
            .insert_resource(FreeRenderLayers::default())
            .insert_resource(ErrorMessage::default())
            .insert_resource(CurrentFilePath::default())
            .insert_resource(PendingPreviewStates::default())
            .insert_resource(EditableVars::default());
    }
}

#[derive(Resource, Default, Clone, Deref, DerefMut)]
struct EditorText(pub String);

#[derive(Resource, Default, Clone, Deref, DerefMut)]
pub struct EditableVars(pub Vec<cadhr_lang::parse::VarInfo>);

#[derive(Resource, Default, Deref, DerefMut)]
pub struct NextPreviewId(u64);

#[derive(Resource, Clone, Deref, DerefMut)]
pub struct FreeRenderLayers(pub Vec<usize>);

impl Default for FreeRenderLayers {
    fn default() -> Self {
        Self((1..32).rev().collect())
    }
}

#[derive(Resource, Default, Clone, Deref, DerefMut)]
pub struct ErrorMessage(pub String);

#[derive(Resource, Default, Clone, Deref, DerefMut)]
pub struct CurrentFilePath(pub Option<PathBuf>);

#[derive(Resource, Default, Clone, Deref, DerefMut)]
pub struct PendingPreviewStates(pub HashMap<u64, PreviewState>);
