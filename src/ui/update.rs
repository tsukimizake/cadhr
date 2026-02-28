use crate::events::{CadhrLangOutput, GeneratePreviewRequest, PreviewGenerated};
use crate::ui::{
    CurrentFilePath, EditableVars, EditorText, ErrorMessage, FreeRenderLayers, NextPreviewId,
    PendingPreviewStates, PreviewState, PreviewTarget, SelectedControlPoint, SessionLoadContents,
    SessionPreviews, SessionSaveContents, ThreeMfFileContents,
};
use bevy::asset::RenderAssetUsages;
use bevy::camera::RenderTarget;
use bevy::camera::primitives::MeshAabb;
use bevy::camera::visibility::RenderLayers;
use bevy::mesh::{Indices, VertexAttributeValues};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
use bevy_egui::{EguiContexts, egui};
use bevy_file_dialog::prelude::*;
#[allow(unused_imports)]
use cadhr_lang::manifold_bridge::{EvaluatedNode, collect_tracked_spans_from_expr};
use cadhr_lang::parse::SrcSpan;
use std::io::Cursor;
use std::path::Path;

const MAX_CAMERA_PITCH: f64 = std::f64::consts::FRAC_PI_2 - 0.001;
const MIN_CAMERA_PITCH: f64 = -MAX_CAMERA_PITCH;
const MAX_CAMERA_YAW: f64 = std::f64::consts::PI - 0.001;
const MIN_CAMERA_YAW: f64 = -MAX_CAMERA_YAW;
const DEFAULT_ZOOM: f32 = 10.0;
const AUTO_ZOOM_SENTINEL: f32 = 0.0;
const MIN_ZOOM: f32 = 1.0;
const MAX_ZOOM: f32 = 100.0;
const CAMERA_DISTANCE_FACTOR: f32 = 2.4 * 3.0;

fn refresh_editable_vars(editor_text: &str, editable_vars: &mut EditableVars) {
    if let Ok(clauses) = cadhr_lang::parse::database(editor_text) {
        **editable_vars = cadhr_lang::parse::collect_editable_vars(&clauses);
    }
}
const MIN_CAMERA_DISTANCE: f32 = 5.0;

fn camera_distance_and_zoom_from_mesh(mesh: &Mesh) -> (f32, f32) {
    if let Some(aabb) = mesh.compute_aabb() {
        let half_extents = aabb.half_extents;
        let max_extent = half_extents.x.max(half_extents.y).max(half_extents.z);
        let natural_distance = max_extent * CAMERA_DISTANCE_FACTOR;
        let camera_distance = natural_distance.max(MIN_CAMERA_DISTANCE);
        let auto_zoom = if natural_distance > f32::EPSILON {
            (DEFAULT_ZOOM * camera_distance / natural_distance).clamp(MIN_ZOOM, MAX_ZOOM)
        } else {
            DEFAULT_ZOOM
        };
        (camera_distance, auto_zoom)
    } else {
        (MIN_CAMERA_DISTANCE, DEFAULT_ZOOM)
    }
}

// egui UI: add previews dynamically and render all existing previews
pub(super) fn egui_ui(
    mut commands: Commands,
    mut contexts: EguiContexts,
    mut preview_query: Query<(Entity, &mut PreviewTarget)>,
    mut editor_text: ResMut<EditorText>,
    mut next_preview_id: ResMut<NextPreviewId>,
    mut pending_states: ResMut<PendingPreviewStates>,
    mut free_render_layers: ResMut<FreeRenderLayers>,
    mut ev_generate: MessageWriter<GeneratePreviewRequest>,
    mut editable_vars: ResMut<EditableVars>,
    mut selected_cp: ResMut<SelectedControlPoint>,
    error_message: Res<ErrorMessage>,
    current_file_path: Res<CurrentFilePath>,
    meshes: Res<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Parse editable vars if not yet populated (e.g. initial load, session load)
    if editable_vars.is_empty() && !editor_text.is_empty() {
        refresh_editable_vars(&editor_text, &mut editable_vars);
    }

    // Collect preview targets into a Vec for indexed access
    let mut preview_targets: Vec<(Entity, Mut<PreviewTarget>)> = preview_query.iter_mut().collect();
    // Toolbar: add a new preview or reload existing
    if let Ok(ctx) = contexts.ctx_mut() {
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                // Session operations
                if ui.button("Open Session").clicked() {
                    commands
                        .dialog()
                        .pick_directory_path::<SessionLoadContents>();
                }
                if ui.button("Save Session").clicked() {
                    if let Some(ref path) = **current_file_path {
                        save_session(
                            path,
                            &editor_text,
                            preview_targets.iter().map(|(_, t)| t.as_ref()),
                        );
                    } else {
                        commands
                            .dialog()
                            .set_file_name("untitled")
                            .save_file::<SessionSaveContents>(vec![]);
                    }
                }
                if ui.button("Save Session As").clicked() {
                    let file_name = current_file_path
                        .as_ref()
                        .as_ref()
                        .and_then(|p| p.file_name())
                        .and_then(|n| n.to_str())
                        .unwrap_or("untitled");
                    commands
                        .dialog()
                        .set_file_name(file_name)
                        .save_file::<SessionSaveContents>(vec![]);
                }

                ui.separator();

                if ui.button("Add Preview").clicked() {
                    let preview_id = **next_preview_id;
                    **next_preview_id += 1;
                    let query_text = "main.".to_string();
                    pending_states.insert(
                        preview_id,
                        PreviewState {
                            preview_id: Some(preview_id),
                            query: query_text.clone(),
                            zoom: AUTO_ZOOM_SENTINEL,
                            rotate_x: 0.0,
                            rotate_y: 0.0,
                            control_point_overrides: Default::default(),
                        },
                    );
                    ev_generate.write(GeneratePreviewRequest {
                        preview_id,
                        database: (**editor_text).clone(),
                        query: query_text,
                        include_paths: (**current_file_path).iter().cloned().collect(),
                        control_point_overrides: Default::default(),
                    });
                }
                if ui.button("Update Previews").clicked() {
                    for (_, target) in preview_targets.iter() {
                        ev_generate.write(GeneratePreviewRequest {
                            preview_id: target.preview_id,
                            database: (**editor_text).clone(),
                            query: target.query.clone(),
                            include_paths: (**current_file_path).iter().cloned().collect(),
                            control_point_overrides: target.control_point_overrides.clone(),
                        });
                    }
                }
                if ui.button("Reload and Preview").clicked() {
                    if let Some(ref path) = **current_file_path {
                        let db_path = path.join("db.cadhr");
                        if let Ok(content) = std::fs::read_to_string(&db_path) {
                            **editor_text = content;
                        }
                    }
                    refresh_editable_vars(&editor_text, &mut editable_vars);
                    for (_, target) in preview_targets.iter() {
                        ev_generate.write(GeneratePreviewRequest {
                            preview_id: target.preview_id,
                            database: (**editor_text).clone(),
                            query: target.query.clone(),
                            include_paths: (**current_file_path).iter().cloned().collect(),
                            control_point_overrides: target.control_point_overrides.clone(),
                        });
                    }
                }
            });
        });
    }

    // Precompute egui texture ids for each preview's offscreen image
    let preview_images: Vec<(egui::TextureId, UVec2)> = preview_targets
        .iter()
        .map(|(_, t)| {
            let id = contexts.image_id(&t.rt_image).unwrap_or_else(|| {
                contexts.add_image(bevy_egui::EguiTextureHandle::Strong(t.rt_image.clone()))
            });
            (id, t.rt_size)
        })
        .collect();

    // Error message panel at the bottom
    if let Ok(ctx) = contexts.ctx_mut() {
        if !error_message.is_empty() {
            egui::TopBottomPanel::bottom("error_panel")
                .min_height(24.0)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.colored_label(egui::Color32::from_rgb(255, 100, 100), &**error_message);
                    });
                });
        }
    }

    // Main split view: left = large text area, right = previews list and controls
    if let Ok(ctx) = contexts.ctx_mut() {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.columns(2, |columns| {
                // Left half: text area + editable vars panel
                let left = &mut columns[0];

                // Editable vars sliders at the top of left panel
                let mut global_var_edits: Vec<(f64, SrcSpan)> = Vec::new();
                if !editable_vars.is_empty() {
                    egui::Frame::default()
                        .stroke(egui::Stroke::new(1.0, egui::Color32::from_gray(80)))
                        .corner_radius(egui::CornerRadius::same(4))
                        .inner_margin(egui::Margin::symmetric(6, 4))
                        .show(left, |ui| {
                            ui.label("Parameters");
                            for var_info in editable_vars.iter_mut() {
                                ui.horizontal(|ui| {
                                    ui.label(format!("{}:", var_info.name));
                                    let min_f =
                                        var_info.min.map(|b| b.value.to_f64()).unwrap_or(-10000.0);
                                    let max_f =
                                        var_info.max.map(|b| b.value.to_f64()).unwrap_or(10000.0);
                                    let mut val = var_info.value.to_f64();
                                    let changed = ui
                                        .add(
                                            egui::DragValue::new(&mut val)
                                                .speed(0.5)
                                                .range(min_f..=max_f),
                                        )
                                        .changed();
                                    if changed {
                                        global_var_edits.push((val, var_info.span));
                                    }
                                });
                            }
                        });
                    left.add_space(4.0);
                }

                // Text editor
                let size_left = left.available_size();
                let text_response = left.add_sized(
                    size_left,
                    egui::TextEdit::multiline(&mut **editor_text)
                        .hint_text("ここにテキストを入力してください"),
                );

                // Refresh editable vars when text changes
                if text_response.changed() {
                    refresh_editable_vars(&editor_text, &mut editable_vars);
                }

                // Right half: show and edit previews
                let right = &mut columns[1];
                // Collect actions to process after iterating
                let mut updates_to_send: Vec<(u64, String)> = Vec::new();
                let mut exports_to_send: Vec<usize> = Vec::new();
                let mut closes_to_send: Vec<(Entity, usize)> = Vec::new();
                let mut cp_override_regenerate: Vec<u64> = Vec::new();
                egui::ScrollArea::vertical()
                    .auto_shrink([false; 2])
                    .show(right, |ui| {
                        if preview_targets.is_empty() {
                            ui.label(
                                "プレビューはまだありません。上の『Add Preview』を押してください。",
                            );
                        } else {
                            for (i, (entity, target)) in preview_targets.iter_mut().enumerate() {
                                if let Some((tex_id, size)) = preview_images.get(i) {
                                    match preview_target_ui(ui, i, target, *tex_id, *size, &mut selected_cp, &mut cp_override_regenerate) {
                                        PreviewAction::Update => {
                                            updates_to_send
                                                .push((target.preview_id, target.query.clone()));
                                        }
                                        PreviewAction::Export3MF => {
                                            exports_to_send.push(i);
                                        }
                                        PreviewAction::Close => {
                                            closes_to_send.push((*entity, target.render_layer));
                                        }
                                        PreviewAction::None => {}
                                    }
                                }
                                ui.add_space(6.0);
                            }
                        }
                    });
                // Update control sphere colors based on selection
                for (_, target) in preview_targets.iter() {
                    let is_selected_preview = selected_cp.preview_id == Some(target.preview_id);
                    for (ci, entity) in target.control_sphere_entities.iter().enumerate() {
                        let selected = is_selected_preview && selected_cp.index == ci;
                        let color = if selected {
                            Color::srgb(0.0, 1.0, 0.5)
                        } else {
                            Color::srgb(1.0, 0.9, 0.0)
                        };
                        let mat = materials.add(StandardMaterial {
                            base_color: color,
                            unlit: true,
                            ..default()
                        });
                        commands.entity(*entity).insert(MeshMaterial3d(mat));
                    }
                }

                // Send update requests
                for (preview_id, query) in updates_to_send {
                    let overrides = preview_targets
                        .iter()
                        .find(|(_, t)| t.preview_id == preview_id)
                        .map(|(_, t)| t.control_point_overrides.clone())
                        .unwrap_or_default();
                    ev_generate.write(GeneratePreviewRequest {
                        preview_id,
                        database: (**editor_text).clone(),
                        query,
                        include_paths: (**current_file_path).iter().cloned().collect(),
                        control_point_overrides: overrides,
                    });
                }
                // Regenerate previews that had control point overrides changed
                for preview_id in cp_override_regenerate {
                    if let Some((_, target)) = preview_targets.iter().find(|(_, t)| t.preview_id == preview_id) {
                        ev_generate.write(GeneratePreviewRequest {
                            preview_id,
                            database: (**editor_text).clone(),
                            query: target.query.clone(),
                            include_paths: (**current_file_path).iter().cloned().collect(),
                            control_point_overrides: target.control_point_overrides.clone(),
                        });
                    }
                }
                // Handle export requests
                for idx in exports_to_send {
                    if let Some(target) = preview_targets.get(idx) {
                        if let Some(mesh) = meshes.get(&target.1.mesh_handle) {
                            if let Some(threemf_data) = bevy_mesh_to_threemf(mesh) {
                                let file_name = export_3mf_file_name(
                                    current_file_path.as_ref().as_ref().map(|p| p.as_path()),
                                    &target.1.query,
                                );
                                commands
                                    .dialog()
                                    .add_filter("3MF", &["3mf"])
                                    .set_file_name(file_name)
                                    .save_file::<ThreeMfFileContents>(threemf_data);
                            }
                        }
                    }
                }
                for (entity, render_layer) in closes_to_send {
                    free_render_layers.push(render_layer);
                    commands.entity(entity).despawn();
                }

                // Apply source text edits from parameters panel
                if !global_var_edits.is_empty() {
                    global_var_edits.sort_by(|a, b| b.1.start.cmp(&a.1.start));
                    for (new_val, span) in &global_var_edits {
                        let new_val_str = format!("{}", new_val);
                        let src = &mut **editor_text;
                        if span.end <= src.len() {
                            src.replace_range(span.start..span.end, &new_val_str);
                        }
                    }
                    refresh_editable_vars(&editor_text, &mut editable_vars);
                    for (_, target) in preview_targets.iter() {
                        ev_generate.write(GeneratePreviewRequest {
                            preview_id: target.preview_id,
                            database: (**editor_text).clone(),
                            query: target.query.clone(),
                            include_paths: (**current_file_path).iter().cloned().collect(),
                            control_point_overrides: target.control_point_overrides.clone(),
                        });
                    }
                }
            });
        });
    }
}

fn update_control_spheres(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    parent_entity: Entity,
    target: &mut PreviewTarget,
    both_layers: &RenderLayers,
) {
    let cp_count = target.control_points.len();
    let existing_count = target.control_sphere_entities.len();

    // Update positions of existing spheres
    for (i, cp) in target.control_points.iter().enumerate() {
        if i < existing_count {
            commands
                .entity(target.control_sphere_entities[i])
                .insert(Transform::from_xyz(
                    cp.x.value as f32,
                    cp.y.value as f32,
                    cp.z.value as f32,
                ));
        }
    }

    // Spawn new spheres if count increased
    let sphere_mesh = meshes.add(Sphere::new(0.5));
    let sphere_material = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.9, 0.0),
        unlit: true,
        ..default()
    });

    for i in existing_count..cp_count {
        let cp = &target.control_points[i];
        let child_id = commands
            .spawn((
                Mesh3d(sphere_mesh.clone()),
                MeshMaterial3d(sphere_material.clone()),
                Transform::from_xyz(cp.x.value as f32, cp.y.value as f32, cp.z.value as f32),
                both_layers.clone(),
            ))
            .id();
        commands.entity(parent_entity).add_child(child_id);
        target.control_sphere_entities.push(child_id);
    }

    // Despawn extras if count decreased
    for i in cp_count..existing_count {
        commands
            .entity(target.control_sphere_entities[i])
            .despawn();
    }
    target.control_sphere_entities.truncate(cp_count);
}

// Handle generated previews: spawn entities and track UI state
pub(super) fn on_preview_generated(
    mut ev_generated: MessageReader<PreviewGenerated>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut preview_query: Query<(Entity, &mut PreviewTarget)>,
    mut pending_states: ResMut<PendingPreviewStates>,
    mut free_render_layers: ResMut<FreeRenderLayers>,
) {
    for ev in ev_generated.read() {
        // Update existing preview if it is still alive.
        let mut updated_existing = false;
        for (entity, mut target) in preview_query.iter_mut() {
            if target.preview_id == ev.preview_id {
                if let Some(mesh_asset) = meshes.get_mut(&target.mesh_handle) {
                    *mesh_asset = ev.mesh.clone();
                }
                target.evaluated_nodes = ev.evaluated_nodes.clone();
                target.control_points = ev.control_points.clone();
                let (camera_distance, auto_zoom) =
                    camera_distance_and_zoom_from_mesh(&ev.mesh);
                target.base_camera_distance = camera_distance;
                target.zoom = auto_zoom;

                // Update control sphere entities
                let render_layer = target.render_layer;
                let both_layers = RenderLayers::from_layers(&[0, render_layer]);
                update_control_spheres(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    entity,
                    &mut target,
                    &both_layers,
                );

                updated_existing = true;
                break;
            }
        }
        if updated_existing {
            continue;
        }

        // Spawn only when this ID is still pending creation.
        let Some(pending_state) = pending_states.remove(&ev.preview_id) else {
            continue;
        };

        // New preview: spawn entities
        let mesh_handle = meshes.add(ev.mesh.clone());

        // Choose a simple material
        let material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.2, 0.2),
            ..default()
        });

        // Create an offscreen render target
        let rt_size = UVec2::new(512, 384);
        let size = Extent3d {
            width: rt_size.x,
            height: rt_size.y,
            depth_or_array_layers: 1,
        };
        let mut image = Image::new_fill(
            size,
            TextureDimension::D2,
            &[0, 0, 0, 0],
            TextureFormat::Rgba8UnormSrgb,
            RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
        );
        image.texture_descriptor.usage = TextureUsages::RENDER_ATTACHMENT
            | TextureUsages::TEXTURE_BINDING
            | TextureUsages::COPY_SRC;
        let rt_image = images.add(image);

        // Unique render layer per preview
        let Some(render_layer) = free_render_layers.pop() else {
            bevy::log::error!(
                "No free render layer available for preview {}",
                ev.preview_id
            );
            continue;
        };
        let layer_only = RenderLayers::layer(render_layer);

        let (camera_distance, auto_zoom) = camera_distance_and_zoom_from_mesh(&ev.mesh);
        let initial_zoom = if pending_state.zoom > AUTO_ZOOM_SENTINEL {
            pending_state.zoom
        } else {
            auto_zoom
        };
        let cam_pos = Vec3::new(
            camera_distance * 0.5,
            camera_distance,
            camera_distance * 0.5,
        );

        // Make the mesh visible to both default (0) and offscreen layer
        let both_layers = RenderLayers::from_layers(&[0, render_layer]);

        // Spawn XYZ axis indicators
        let axis_length = 20.0;
        let axis_radius = 0.1;
        let axis_cylinder = meshes.add(Cylinder::new(axis_radius, axis_length));

        let x_material = materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.0, 0.0),
            unlit: true,
            ..default()
        });
        let y_material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.0, 1.0, 0.0),
            unlit: true,
            ..default()
        });
        let z_material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.0, 0.0, 1.0),
            unlit: true,
            ..default()
        });

        // Control point sphere material
        let cp_sphere_mesh = meshes.add(Sphere::new(0.5));
        let cp_material = materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.9, 0.0),
            unlit: true,
            ..default()
        });

        // Spawn root entity with all children
        let mut camera_entity = Entity::PLACEHOLDER;
        let mut control_sphere_entities: Vec<Entity> = Vec::new();
        commands
            .spawn((Transform::default(), Visibility::default()))
            .with_children(|parent| {
                // Mesh
                parent.spawn((
                    Mesh3d(mesh_handle.clone()),
                    MeshMaterial3d(material),
                    Transform::from_xyz(0.0, 0.0, 0.0),
                    both_layers.clone(),
                ));

                // Camera
                camera_entity = parent
                    .spawn((
                        Camera3d::default(),
                        Camera::default(),
                        RenderTarget::Image(rt_image.clone().into()),
                        Transform::from_xyz(cam_pos.x, cam_pos.y, cam_pos.z)
                            .looking_at(Vec3::ZERO, Vec3::Z),
                        layer_only.clone(),
                    ))
                    .id();

                // Light
                parent.spawn((
                    DirectionalLight::default(),
                    Transform::from_xyz(4.0, 4.0, 8.0).looking_at(Vec3::ZERO, Vec3::Z),
                    layer_only.clone(),
                ));

                // X axis (red)
                parent.spawn((
                    Mesh3d(axis_cylinder.clone()),
                    MeshMaterial3d(x_material),
                    Transform::from_xyz(axis_length / 2.0, 0.0, 0.0)
                        .with_rotation(Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2)),
                    both_layers.clone(),
                ));

                // Y axis (green)
                parent.spawn((
                    Mesh3d(axis_cylinder.clone()),
                    MeshMaterial3d(y_material),
                    Transform::from_xyz(0.0, axis_length / 2.0, 0.0),
                    both_layers.clone(),
                ));

                // Z axis (blue)
                parent.spawn((
                    Mesh3d(axis_cylinder.clone()),
                    MeshMaterial3d(z_material),
                    Transform::from_xyz(0.0, 0.0, axis_length / 2.0)
                        .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
                    both_layers.clone(),
                ));

                // Control point spheres
                for cp in &ev.control_points {
                    let id = parent
                        .spawn((
                            Mesh3d(cp_sphere_mesh.clone()),
                            MeshMaterial3d(cp_material.clone()),
                            Transform::from_xyz(
                                cp.x.value as f32,
                                cp.y.value as f32,
                                cp.z.value as f32,
                            ),
                            both_layers.clone(),
                        ))
                        .id();
                    control_sphere_entities.push(id);
                }
            })
            .insert({
                PreviewTarget {
                    preview_id: ev.preview_id,
                    render_layer,
                    mesh_handle: mesh_handle.clone(),
                    rt_image: rt_image.clone(),
                    rt_size,
                    camera_entity,
                    base_camera_distance: camera_distance,
                    zoom: initial_zoom,
                    rotate_x: pending_state.rotate_x,
                    rotate_y: pending_state.rotate_y,
                    query: ev.query.clone(),
                    evaluated_nodes: ev.evaluated_nodes.clone(),
                    control_points: ev.control_points.clone(),
                    control_sphere_entities,
                    control_point_overrides: pending_state.control_point_overrides.clone(),
                }
            });
    }
}

// Pending previews and polling system are no longer needed with bevy-async-ecs

/// UI action returned from preview_target_ui
enum PreviewAction {
    None,
    Update,
    Export3MF,
    Close,
}

// ============================================================
// Raycast: click-to-source infrastructure
// ============================================================

/// Ray-triangle intersection using Moller-Trumbore algorithm.
/// Returns distance t if hit (t > 0).
#[allow(unused)]
fn ray_triangle_intersect(
    ray_origin: &[f64; 3],
    ray_dir: &[f64; 3],
    v0: &[f64; 3],
    v1: &[f64; 3],
    v2: &[f64; 3],
) -> Option<f64> {
    let edge1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
    let edge2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
    let h = cross(ray_dir, &edge2);
    let a = dot(&edge1, &h);
    if a.abs() < 1e-10 {
        return None;
    }
    let f = 1.0 / a;
    let s = [
        ray_origin[0] - v0[0],
        ray_origin[1] - v0[1],
        ray_origin[2] - v0[2],
    ];
    let u = f * dot(&s, &h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(&s, &edge1);
    let v = f * dot(ray_dir, &q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * dot(&edge2, &q);
    if t > 1e-10 { Some(t) } else { None }
}

#[allow(unused)]
fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[allow(unused)]
fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Test ray against AABB, returns true if ray intersects the box.
#[allow(unused)]
fn ray_aabb_intersect(
    ray_origin: &[f64; 3],
    ray_dir: &[f64; 3],
    aabb_min: &[f64; 3],
    aabb_max: &[f64; 3],
) -> bool {
    let mut tmin = f64::NEG_INFINITY;
    let mut tmax = f64::INFINITY;
    for i in 0..3 {
        if ray_dir[i].abs() < 1e-12 {
            if ray_origin[i] < aabb_min[i] || ray_origin[i] > aabb_max[i] {
                return false;
            }
        } else {
            let inv_d = 1.0 / ray_dir[i];
            let mut t1 = (aabb_min[i] - ray_origin[i]) * inv_d;
            let mut t2 = (aabb_max[i] - ray_origin[i]) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            tmin = tmin.max(t1);
            tmax = tmax.min(t2);
            if tmin > tmax {
                return false;
            }
        }
    }
    tmax >= 0.0
}

/// Raycast against an EvaluatedNode tree. Returns (distance, node_ref) of closest hit.
#[allow(unused)]
fn raycast_evaluated_nodes<'a>(
    ray_origin: &[f64; 3],
    ray_dir: &[f64; 3],
    nodes: &'a [EvaluatedNode],
) -> Option<(f64, &'a EvaluatedNode)> {
    let mut best: Option<(f64, &'a EvaluatedNode)> = None;
    for node in nodes {
        if let Some((t, n)) = raycast_node(ray_origin, ray_dir, node) {
            if best.is_none() || t < best.unwrap().0 {
                best = Some((t, n));
            }
        }
    }
    best
}

#[allow(unused)]
fn raycast_node<'a>(
    ray_origin: &[f64; 3],
    ray_dir: &[f64; 3],
    node: &'a EvaluatedNode,
) -> Option<(f64, &'a EvaluatedNode)> {
    if !ray_aabb_intersect(ray_origin, ray_dir, &node.aabb_min, &node.aabb_max) {
        return None;
    }

    // First try children (more specific nodes)
    let mut best: Option<(f64, &'a EvaluatedNode)> = None;
    for child in &node.children {
        if let Some((t, n)) = raycast_node(ray_origin, ray_dir, child) {
            if best.is_none() || t < best.unwrap().0 {
                best = Some((t, n));
            }
        }
    }
    if best.is_some() {
        return best;
    }

    // No child hit: test this node's mesh triangles
    let num_props = 6usize; // xyz + normals after calculate_normals
    let verts = &node.mesh_verts;
    let indices = &node.mesh_indices;
    let mut closest_t = f64::INFINITY;
    for tri in indices.chunks(3) {
        if tri.len() < 3 {
            continue;
        }
        let i0 = tri[0] as usize * num_props;
        let i1 = tri[1] as usize * num_props;
        let i2 = tri[2] as usize * num_props;
        if i0 + 2 >= verts.len() || i1 + 2 >= verts.len() || i2 + 2 >= verts.len() {
            continue;
        }
        let v0 = [verts[i0] as f64, verts[i0 + 1] as f64, verts[i0 + 2] as f64];
        let v1 = [verts[i1] as f64, verts[i1 + 1] as f64, verts[i1 + 2] as f64];
        let v2 = [verts[i2] as f64, verts[i2 + 1] as f64, verts[i2 + 2] as f64];
        if let Some(t) = ray_triangle_intersect(ray_origin, ray_dir, &v0, &v1, &v2) {
            if t < closest_t {
                closest_t = t;
            }
        }
    }
    if closest_t < f64::INFINITY {
        Some((closest_t, node))
    } else {
        None
    }
}

/// Generate a ray from UV coordinates (0..1) in the preview image, given camera params.
#[allow(unused)]
fn generate_ray_from_uv(u: f32, v: f32, target: &PreviewTarget) -> ([f64; 3], [f64; 3]) {
    let rx = target.rotate_x.clamp(MIN_CAMERA_PITCH, MAX_CAMERA_PITCH) as f32;
    let ry = target.rotate_y.clamp(MIN_CAMERA_YAW, MAX_CAMERA_YAW) as f32;
    let zoom = target.zoom.clamp(MIN_ZOOM, MAX_ZOOM);
    let dist = target.base_camera_distance * (20.0 / zoom);

    let cam_x = dist * ry.sin() * rx.cos();
    let cam_y = dist * ry.cos() * rx.cos();
    let cam_z = dist * rx.sin();

    let cam_pos = Vec3::new(cam_x, cam_y, cam_z);
    let cam_transform = Transform::from_translation(cam_pos).looking_at(Vec3::ZERO, Vec3::Z);

    // Default perspective FOV (Bevy Camera3d default is 45 degrees vertical)
    let fov_y: f32 = std::f32::consts::FRAC_PI_4;
    let aspect = target.rt_size.x as f32 / target.rt_size.y as f32;
    let half_h = (fov_y * 0.5).tan();
    let half_w = half_h * aspect;

    // Convert UV (0..1, 0..1) to NDC (-1..1, -1..1), with V flipped (egui top-left origin)
    let ndc_x = u * 2.0 - 1.0;
    let ndc_y = 1.0 - v * 2.0;

    // Ray direction in camera local space
    let local_dir = Vec3::new(ndc_x * half_w, ndc_y * half_h, -1.0).normalize();

    // Transform to world space
    let world_dir = cam_transform.rotation * local_dir;

    let origin = [cam_pos.x as f64, cam_pos.y as f64, cam_pos.z as f64];
    let dir = [world_dir.x as f64, world_dir.y as f64, world_dir.z as f64];
    (origin, dir)
}

/// Returns the action requested by the user
fn preview_target_ui(
    ui: &mut egui::Ui,
    index: usize,
    target: &mut PreviewTarget,
    tex_id: egui::TextureId,
    size: UVec2,
    selected_cp: &mut SelectedControlPoint,
    cp_override_regenerate: &mut Vec<u64>,
) -> PreviewAction {
    let mut action = PreviewAction::None;
    egui::Frame::default()
        .stroke(egui::Stroke::new(1.0, egui::Color32::from_gray(120)))
        .corner_radius(egui::CornerRadius::same(6))
        .inner_margin(egui::Margin::symmetric(8, 8))
        .show(ui, |ui| {
            ui.label(format!("Preview {}", index + 1));
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("?-");
                ui.text_edit_singleline(&mut target.query);
                if ui.button("Update Preview").clicked() {
                    action = PreviewAction::Update;
                }
                if ui.button("Export 3MF").clicked() {
                    action = PreviewAction::Export3MF;
                }
                if ui.button("Close").clicked() {
                    action = PreviewAction::Close;
                }
            });
            ui.add_space(4.0);
            // Camera controls (orbit and zoom)
            ui.horizontal(|ui| {
                target.rotate_x = target.rotate_x.clamp(MIN_CAMERA_PITCH, MAX_CAMERA_PITCH);
                target.rotate_y = target.rotate_y.clamp(MIN_CAMERA_YAW, MAX_CAMERA_YAW);
                ui.label("Rotate X:");
                ui.add(
                    egui::DragValue::new(&mut target.rotate_x)
                        .speed(0.01)
                        .range(MIN_CAMERA_PITCH..=MAX_CAMERA_PITCH),
                );
                ui.label("Rotate Y:");
                ui.add(
                    egui::DragValue::new(&mut target.rotate_y)
                        .speed(0.01)
                        .range(MIN_CAMERA_YAW..=MAX_CAMERA_YAW),
                );
                ui.label("Zoom:");
                ui.add(
                    egui::DragValue::new(&mut target.zoom)
                        .speed(0.1)
                        .range(MIN_ZOOM..=MAX_ZOOM),
                );
            });
            ui.add_space(6.0);

            // Show the offscreen render under controls (with click detection)
            let avail_w = ui.available_width();
            let aspect = size.y as f32 / size.x as f32;
            let w = avail_w;
            let h = w * aspect;
            let image_response = ui.add(
                egui::Image::from_texture((tex_id, egui::vec2(w, h)))
                    .sense(egui::Sense::click()),
            );

            // Click-to-select control point
            if image_response.clicked() && !target.control_points.is_empty() {
                if let Some(pos) = image_response.interact_pointer_pos() {
                    let rect = image_response.rect;
                    let u = (pos.x - rect.min.x) / rect.width();
                    let v = (pos.y - rect.min.y) / rect.height();
                    let (ray_origin, ray_dir) = generate_ray_from_uv(u, v, target);

                    // Ray-sphere intersection with larger radius for easier clicking
                    let sphere_radius = 1.5_f64;
                    let mut best_hit: Option<(f64, usize)> = None;
                    for (ci, cp) in target.control_points.iter().enumerate() {
                        let center = [cp.x.value, cp.y.value, cp.z.value];
                        let oc = [
                            ray_origin[0] - center[0],
                            ray_origin[1] - center[1],
                            ray_origin[2] - center[2],
                        ];
                        let a = dot(&ray_dir, &ray_dir);
                        let b = 2.0 * dot(&oc, &ray_dir);
                        let c = dot(&oc, &oc) - sphere_radius * sphere_radius;
                        let discriminant = b * b - 4.0 * a * c;
                        if discriminant >= 0.0 {
                            let t = (-b - discriminant.sqrt()) / (2.0 * a);
                            let t = if t > 0.0 {
                                t
                            } else {
                                (-b + discriminant.sqrt()) / (2.0 * a)
                            };
                            if t > 0.0 {
                                if best_hit.is_none() || t < best_hit.unwrap().0 {
                                    best_hit = Some((t, ci));
                                }
                            }
                        }
                    }

                    if let Some((_t, ci)) = best_hit {
                        selected_cp.preview_id = Some(target.preview_id);
                        selected_cp.index = ci;
                    } else {
                        // Click on empty space deselects
                        if selected_cp.preview_id == Some(target.preview_id) {
                            selected_cp.preview_id = None;
                        }
                    }
                }
            }

            // Control point DragValue UI
            if selected_cp.preview_id == Some(target.preview_id) {
                if let Some(cp) = target.control_points.get_mut(selected_cp.index) {
                    ui.add_space(4.0);
                    let label = cp
                        .name
                        .as_deref()
                        .map(|n| format!("Control: {}", n))
                        .unwrap_or_else(|| format!("Control {}", selected_cp.index));
                    ui.label(label);
                    ui.horizontal(|ui| {
                        for (axis_idx, (axis_label, tracked)) in [
                            ("X:", &mut cp.x),
                            ("Y:", &mut cp.y),
                            ("Z:", &mut cp.z),
                        ].iter_mut().enumerate() {
                            ui.label(*axis_label);
                            let mut val = tracked.value;
                            if ui
                                .add(egui::DragValue::new(&mut val).speed(0.5))
                                .changed()
                            {
                                tracked.value = val;
                                if let Some(ref vname) = cp.var_names[axis_idx] {
                                    target.control_point_overrides.insert(vname.clone(), val);
                                    cp_override_regenerate.push(target.preview_id);
                                }
                            }
                        }
                    });
                }
            }
        });
    action
}

fn export_3mf_file_name(current_path: Option<&Path>, query: &str) -> String {
    let base_name = current_path
        .and_then(|p| p.file_stem())
        .and_then(|n| n.to_str())
        .filter(|s| !s.is_empty());
    let query_suffix = sanitize_query_for_filename(query);

    if let Some(base) = base_name {
        format!("{base}_{query_suffix}.3mf")
    } else {
        format!("{query_suffix}.3mf")
    }
}

fn sanitize_query_for_filename(query: &str) -> String {
    query
        .chars()
        .map(|ch| if ch.is_ascii_alphanumeric() { ch } else { '_' })
        .collect::<String>()
        .trim_matches('_')
        .to_string()
}

// Keep spawned preview entity rotations in sync with UI values
pub(super) fn update_preview_transforms(
    preview_query: Query<&PreviewTarget>,
    mut camera_query: Query<&mut Transform, With<Camera3d>>,
) {
    for target in preview_query.iter() {
        if let Ok(mut transform) = camera_query.get_mut(target.camera_entity) {
            let rx = target.rotate_x.clamp(MIN_CAMERA_PITCH, MAX_CAMERA_PITCH) as f32;
            let ry = target.rotate_y.clamp(MIN_CAMERA_YAW, MAX_CAMERA_YAW) as f32;
            let zoom = target.zoom.clamp(MIN_ZOOM, MAX_ZOOM);
            let dist = target.base_camera_distance * (20.0 / zoom);

            // Orbit camera around origin
            let x = dist * ry.sin() * rx.cos();
            let y = dist * ry.cos() * rx.cos();
            let z = dist * rx.sin();

            transform.translation = Vec3::new(x, y, z);
            *transform = transform.looking_at(Vec3::ZERO, Vec3::Z);
        }
    }
}

// Handle cadhr-lang output messages and update error display
pub(super) fn handle_cadhr_lang_output(
    mut ev_output: MessageReader<CadhrLangOutput>,
    mut error_message: ResMut<ErrorMessage>,
) {
    for output in ev_output.read() {
        if output.is_error {
            **error_message = output.message.clone();
        } else {
            // For non-error logs, we could display them differently or just log
            bevy::log::info!("CadhrLang: {}", output.message);
            // Clear error message on successful execution
            **error_message = String::new();
        }
    }
}

const LAST_SESSION_PATH_FILE: &str = "/tmp/cadhr_last_session_path";

fn save_last_session_path(path: &Path) {
    if let Err(e) = std::fs::write(LAST_SESSION_PATH_FILE, path.to_string_lossy().as_bytes()) {
        bevy::log::warn!("Failed to save last session path: {:?}", e);
    }
}

fn save_session<'a>(
    dir: &std::path::Path,
    editor_text: &EditorText,
    preview_targets: impl Iterator<Item = &'a PreviewTarget>,
) {
    // Remove the marker file if it exists (from save_file dialog)
    let _ = std::fs::remove_file(dir);

    if let Err(e) = std::fs::create_dir_all(dir) {
        bevy::log::error!("Failed to create session directory: {:?}", e);
        return;
    }

    let db_path = dir.join("db.cadhr");
    let previews_path = dir.join("previews.json");

    if let Err(e) = std::fs::write(&db_path, &**editor_text) {
        bevy::log::error!("Failed to save db file: {:?}", e);
        return;
    }

    let previews = SessionPreviews {
        previews: preview_targets
            .map(|t| PreviewState {
                preview_id: Some(t.preview_id),
                query: t.query.clone(),
                zoom: t.zoom,
                rotate_x: t.rotate_x,
                rotate_y: t.rotate_y,
                control_point_overrides: t.control_point_overrides.clone(),
            })
            .collect(),
    };

    match serde_json::to_string_pretty(&previews) {
        Ok(json) => {
            if let Err(e) = std::fs::write(&previews_path, json) {
                bevy::log::error!("Failed to save previews.json: {:?}", e);
            }
        }
        Err(e) => bevy::log::error!("Failed to serialize previews: {:?}", e),
    }
}

fn load_session(dir: &std::path::Path) -> Option<(String, SessionPreviews)> {
    let db_path = dir.join("db.cadhr");
    let previews_path = dir.join("previews.json");

    let db_content = std::fs::read_to_string(&db_path).ok()?;
    let previews_json = std::fs::read_to_string(&previews_path).ok()?;
    let previews: SessionPreviews = serde_json::from_str(&previews_json).ok()?;

    Some((db_content, previews))
}

pub(super) fn session_saved(
    mut ev_saved: MessageReader<DialogFileSaved<SessionSaveContents>>,
    mut current_file_path: ResMut<CurrentFilePath>,
    editor_text: Res<EditorText>,
    preview_query: Query<&PreviewTarget>,
) {
    for ev in ev_saved.read() {
        if ev.result.is_ok() {
            save_session(&ev.path, &editor_text, preview_query.iter());
            **current_file_path = Some(ev.path.clone());
            save_last_session_path(&ev.path);
        }
    }
}

pub(super) fn session_loaded(
    mut commands: Commands,
    mut ev_picked: MessageReader<DialogDirectoryPicked<SessionLoadContents>>,
    mut editor_text: ResMut<EditorText>,
    mut current_file_path: ResMut<CurrentFilePath>,
    preview_query: Query<(Entity, &PreviewTarget)>,
    mut ev_generate: MessageWriter<GeneratePreviewRequest>,
    mut next_preview_id: ResMut<NextPreviewId>,
    mut pending_states: ResMut<PendingPreviewStates>,
    mut free_render_layers: ResMut<FreeRenderLayers>,
    mut editable_vars: ResMut<EditableVars>,
) {
    for ev in ev_picked.read() {
        if let Some((db_content, previews)) = load_session(&ev.path) {
            **editor_text = db_content;
            **current_file_path = Some(ev.path.clone());
            save_last_session_path(&ev.path);

            refresh_editable_vars(&editor_text, &mut editable_vars);

            // Despawn all preview root entities (children are automatically removed)
            for (entity, target) in preview_query.iter() {
                free_render_layers.push(target.render_layer);
                commands.entity(entity).despawn();
            }

            pending_states.clear();

            for mut preview_state in previews.previews {
                let preview_id = if let Some(saved_id) = preview_state.preview_id {
                    if saved_id >= **next_preview_id {
                        **next_preview_id = saved_id.saturating_add(1);
                    }
                    saved_id
                } else {
                    let generated_id = **next_preview_id;
                    **next_preview_id += 1;
                    generated_id
                };

                preview_state.preview_id = Some(preview_id);
                let query = preview_state.query.clone();
                let overrides = preview_state.control_point_overrides.clone();
                pending_states.insert(preview_id, preview_state);

                ev_generate.write(GeneratePreviewRequest {
                    preview_id,
                    database: (**editor_text).clone(),
                    query,
                    include_paths: (**current_file_path).iter().cloned().collect(),
                    control_point_overrides: overrides,
                });
            }
        } else {
            bevy::log::error!("Failed to load session from {:?}", ev.path);
        }
    }
}

pub(super) fn threemf_saved(mut ev_saved: MessageReader<DialogFileSaved<ThreeMfFileContents>>) {
    for ev in ev_saved.read() {
        if ev.result.is_ok() {
            bevy::log::info!("3MF file saved to: {:?}", ev.path);
        } else {
            bevy::log::error!("Failed to save 3MF file: {:?}", ev.result);
        }
    }
}

pub(super) fn restore_last_session(
    mut editor_text: ResMut<EditorText>,
    mut current_file_path: ResMut<CurrentFilePath>,
    mut ev_generate: MessageWriter<GeneratePreviewRequest>,
    mut next_preview_id: ResMut<NextPreviewId>,
    mut pending_states: ResMut<PendingPreviewStates>,
    mut editable_vars: ResMut<EditableVars>,
) {
    let path_str = match std::fs::read_to_string(LAST_SESSION_PATH_FILE) {
        Ok(s) => s,
        Err(_) => return,
    };
    let path = std::path::PathBuf::from(path_str.trim());
    if !path.is_dir() {
        return;
    }
    if let Some((db_content, previews)) = load_session(&path) {
        **editor_text = db_content;
        **current_file_path = Some(path);

        refresh_editable_vars(&editor_text, &mut editable_vars);

        for mut preview_state in previews.previews {
            let preview_id = if let Some(saved_id) = preview_state.preview_id {
                if saved_id >= **next_preview_id {
                    **next_preview_id = saved_id.saturating_add(1);
                }
                saved_id
            } else {
                let generated_id = **next_preview_id;
                **next_preview_id += 1;
                generated_id
            };

            preview_state.preview_id = Some(preview_id);
            let query = preview_state.query.clone();
            let overrides = preview_state.control_point_overrides.clone();
            pending_states.insert(preview_id, preview_state);

            ev_generate.write(GeneratePreviewRequest {
                preview_id,
                database: (**editor_text).clone(),
                query,
                include_paths: (**current_file_path).iter().cloned().collect(),
                control_point_overrides: overrides,
            });
        }
    }
}

/// Convert a Bevy Mesh to 3MF file bytes
fn bevy_mesh_to_threemf(mesh: &Mesh) -> Option<Vec<u8>> {
    // Get positions
    let positions = match mesh.attribute(Mesh::ATTRIBUTE_POSITION)? {
        VertexAttributeValues::Float32x3(pos) => pos,
        _ => return None,
    };

    // Get indices
    let indices = match mesh.indices()? {
        Indices::U32(idx) => idx.clone(),
        Indices::U16(idx) => idx.iter().map(|&i| i as u32).collect(),
    };

    // Convert to threemf types
    let vertices: Vec<threemf::model::Vertex> = positions
        .iter()
        .map(|[x, y, z]| threemf::model::Vertex {
            x: *x as f64,
            y: *y as f64,
            z: *z as f64,
        })
        .collect();

    let triangles: Vec<threemf::model::Triangle> = indices
        .chunks(3)
        .map(|tri| threemf::model::Triangle {
            v1: tri[0] as usize,
            v2: tri[1] as usize,
            v3: tri[2] as usize,
        })
        .collect();

    let threemf_mesh = threemf::model::Mesh {
        vertices: threemf::model::Vertices { vertex: vertices },
        triangles: threemf::model::Triangles {
            triangle: triangles,
        },
    };

    // Write to buffer
    let mut buffer = Cursor::new(Vec::new());
    if threemf::write(&mut buffer, threemf_mesh).is_err() {
        return None;
    }

    Some(buffer.into_inner())
}
