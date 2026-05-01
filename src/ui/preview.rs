use crate::debug_log;
use std::collections::HashMap;
use std::path::PathBuf;

use cadhr_lang::bom::BomEntry;
use cadhr_lang::manifold_bridge::ControlPoint;
use cadhr_lang::parse::{QueryParam, SrcSpan};
use iced::widget::{column, row, shader, slider, text, text_input};
use iced::{Element, Fill, Task};
use interpreter::{CollisionJobParams, CollisionJobResult, MeshJobParams, MeshJobResult};
use preview::Scene;
use session::SessionPreview;

use crate::export;
use crate::interpreter;
use crate::preview;
use crate::ui::parts;
use crate::session;

#[derive(Clone, PartialEq)]
pub enum PreviewKind {
    Normal,
    Collision {
        part_count: usize,
        collision_count: usize,
    },
}

pub struct Preview {
    pub id: u64,
    pub kind: PreviewKind,
    pub query: String,
    pub scene: Scene,
    pub control_points: Vec<ControlPoint>,
    pub control_point_overrides: HashMap<String, f64>,
    pub query_params: Vec<QueryParam>,
    pub query_param_overrides: HashMap<String, f64>,
    pub last_vertices: Vec<preview::pipeline::Vertex>,
    pub last_indices: Vec<u32>,
    pub bom_entries: Vec<BomEntry>,
    /// false: 注視点は原点 / true: bbox 中心
    pub view_at_object_center: bool,
}

pub struct PreviewModel {
    pub previews: Vec<Preview>,
    pub next_preview_id: u64,
    pub selected_cp: Option<(u64, usize)>,
}

impl PreviewModel {
    pub fn new() -> Self {
        Self {
            previews: vec![],
            next_preview_id: 0,
            selected_cp: None,
        }
    }

    pub fn add_from_session(&mut self, sp: &SessionPreview) -> u64 {
        let id = sp.preview_id;
        if id >= self.next_preview_id {
            self.next_preview_id = id + 1;
        }
        let mut scene = Scene::new();
        scene.view_at_object_center = sp.view_at_object_center;
        self.previews.push(Preview {
            id,
            kind: PreviewKind::Normal,
            query: sp.query.clone(),
            scene,
            control_points: vec![],
            control_point_overrides: sp.control_point_overrides.clone(),
            query_params: vec![],
            query_param_overrides: sp.query_param_overrides.clone(),
            last_vertices: vec![],
            last_indices: vec![],
            bom_entries: vec![],
            view_at_object_center: sp.view_at_object_center,
        });
        id
    }
}

#[derive(Clone)]
pub struct Context {
    pub editor_text: String,
    pub include_paths: Vec<PathBuf>,
    pub base_name: String,
}

pub struct Outcome {
    pub mark_unsaved: bool,
    pub error: Option<(String, Option<SrcSpan>)>,
    pub source_edit: Option<String>,
}

impl Outcome {
    fn none() -> (Task<Msg>, Self) {
        (Task::none(), Self {
            mark_unsaved: false,
            error: None,
            source_edit: None,
        })
    }
}

#[derive(Debug, Clone)]
pub enum Msg {
    AddPreview,
    AddCollisionCheck,
    UpdatePreviews,
    UpdatePreview(u64),
    ClosePreview(u64),
    MovePreviewUp(u64),
    MovePreviewDown(u64),
    QueryChanged(u64, String),
    ToggleViewCenter(u64),
    PreviewGenerated(u64, MeshJobResult),
    CollisionGenerated(u64, CollisionJobResult),
    CpOverrideChanged(u64, String, f64),
    QpOverrideChanged(u64, String, f64),
    PreviewClicked {
        preview_id: u64,
        u: f32,
        v: f32,
        rotate_x: f64,
        rotate_y: f64,
        zoom: f32,
        aspect: f32,
    },
    Export3MF(u64),
    ExportBOM(u64),
    ExportFinished(Result<(), String>),
    CpSourceEdit(u64, String, SrcSpan),
}

pub fn update(model: &mut PreviewModel, msg: Msg, ctx: Context) -> (Task<Msg>, Outcome) {
    match msg {
        Msg::AddPreview => {
            let id = model.next_preview_id;
            model.next_preview_id += 1;
            model.previews.push(Preview {
                id,
                kind: PreviewKind::Normal,
                query: "main.".to_string(),
                scene: Scene::new(),
                control_points: vec![],
                control_point_overrides: Default::default(),
                query_params: vec![],
                query_param_overrides: Default::default(),
                last_vertices: vec![],
                last_indices: vec![],
                bom_entries: vec![],
                view_at_object_center: false,
            });
            (
                generate(model, id, ctx),
                Outcome {
                    mark_unsaved: true,
                    error: None,
                    source_edit: None,
                },
            )
        }
        Msg::AddCollisionCheck => {
            let id = model.next_preview_id;
            model.next_preview_id += 1;
            let mut scene = Scene::new();
            scene.color = [0.4, 0.5, 0.7, 0.7];
            model.previews.push(Preview {
                id,
                kind: PreviewKind::Collision {
                    part_count: 0,
                    collision_count: 0,
                },
                query: "main.".to_string(),
                scene,
                control_points: vec![],
                control_point_overrides: Default::default(),
                query_params: vec![],
                query_param_overrides: Default::default(),
                last_vertices: vec![],
                last_indices: vec![],
                bom_entries: vec![],
                view_at_object_center: false,
            });
            (
                generate(model, id, ctx),
                Outcome {
                    mark_unsaved: true,
                    error: None,
                    source_edit: None,
                },
            )
        }
        Msg::UpdatePreviews => {
            let ids: Vec<u64> = model.previews.iter().map(|p| p.id).collect();
            let tasks: Vec<Task<Msg>> =
                ids.iter().map(|&id| generate(model, id, ctx.clone())).collect();
            (Task::batch(tasks), Outcome {
                mark_unsaved: false,
                error: None,
                source_edit: None,
            })
        }
        Msg::UpdatePreview(id) => (generate(model, id, ctx), Outcome {
            mark_unsaved: false,
            error: None,
            source_edit: None,
        }),
        Msg::ClosePreview(id) => {
            model.previews.retain(|p| p.id != id);
            Outcome::none()
        }
        Msg::MovePreviewUp(id) => {
            if let Some(i) = model.previews.iter().position(|p| p.id == id) {
                if i > 0 {
                    model.previews.swap(i - 1, i);
                }
            }
            let (task, _) = Outcome::none();
            (task, Outcome {
                mark_unsaved: true,
                ..Default::default()
            })
        }
        Msg::MovePreviewDown(id) => {
            if let Some(i) = model.previews.iter().position(|p| p.id == id) {
                if i + 1 < model.previews.len() {
                    model.previews.swap(i, i + 1);
                }
            }
            let (task, _) = Outcome::none();
            (task, Outcome {
                mark_unsaved: true,
                ..Default::default()
            })
        }
        Msg::QueryChanged(id, query) => {
            if let Some(p) = model.previews.iter_mut().find(|p| p.id == id) {
                p.query = query;
            }
            Outcome::none()
        }
        Msg::ToggleViewCenter(id) => {
            if let Some(p) = model.previews.iter_mut().find(|p| p.id == id) {
                p.view_at_object_center = !p.view_at_object_center;
                p.scene.view_at_object_center = p.view_at_object_center;
                // 注視点の切り替えだけでは shader の再描画が走らないので
                // 現メッシュを再アップロードしてバージョンを進める
                let vertices = p.last_vertices.clone();
                let indices = p.last_indices.clone();
                if !vertices.is_empty() {
                    let selected = model
                        .selected_cp
                        .and_then(|(pid, ci)| if pid == id { Some(ci) } else { None });
                    if let Some(p) = model.previews.iter_mut().find(|p| p.id == id) {
                        match &p.kind {
                            PreviewKind::Normal => {
                                p.scene.set_mesh_with_control_points(
                                    vertices,
                                    indices,
                                    &p.control_points,
                                    selected,
                                );
                            }
                            PreviewKind::Collision { .. } => {
                                p.scene.set_mesh(vertices, indices);
                            }
                        }
                    }
                }
            }
            (Task::none(), Outcome {
                mark_unsaved: true,
                error: None,
                source_edit: None,
            })
        }
        Msg::PreviewGenerated(id, result) => {
            debug_log!("MeshJob completed (id={}): {:?}", id, result);
            let mut outcome = Outcome {
                mark_unsaved: false,
                error: None,
                source_edit: None,
            };
            match result {
                MeshJobResult::Success { vertices, indices, control_points, query_params, bom_entries, resolved_terms_debug, .. } => {
                    debug_log!("MeshJob resolved terms:\n{}", resolved_terms_debug);
                    if let Some(p) = model.previews.iter_mut().find(|p| p.id == id) {
                        p.last_vertices = vertices.clone();
                        p.last_indices = indices.clone();
                        p.control_points = control_points;
                        p.query_params = query_params;
                        p.bom_entries = bom_entries;
                        let selected = model
                            .selected_cp
                            .and_then(|(pid, ci)| if pid == id { Some(ci) } else { None });
                        p.scene.set_mesh_with_control_points(
                            vertices,
                            indices,
                            &p.control_points,
                            selected,
                        );
                    }
                }
                MeshJobResult::Error { message, span } => {
                    outcome.error = Some((message, span));
                }
            }
            (Task::none(), outcome)
        }
        Msg::CollisionGenerated(id, result) => {
            debug_log!("CollisionJob completed (id={}): {:?}", id, result);
            let mut outcome = Outcome {
                mark_unsaved: false,
                error: None,
                source_edit: None,
            };
            match result {
                CollisionJobResult::Success { vertices, indices, part_count, collision_count } => {
                    if let Some(p) = model.previews.iter_mut().find(|p| p.id == id) {
                        p.last_vertices = vertices.clone();
                        p.last_indices = indices.clone();
                        p.kind = PreviewKind::Collision {
                            part_count,
                            collision_count,
                        };
                        p.scene.set_mesh(vertices, indices);
                    }
                }
                CollisionJobResult::Error { message, span } => {
                    outcome.error = Some((message, span));
                }
            }
            (Task::none(), outcome)
        }
        Msg::CpOverrideChanged(id, var_name, value) => {
            if let Some(p) = model.previews.iter_mut().find(|p| p.id == id) {
                p.control_point_overrides.insert(var_name, value);
            }
            (generate(model, id, ctx), Outcome {
                mark_unsaved: false,
                error: None,
                source_edit: None,
            })
        }
        Msg::QpOverrideChanged(id, name, value) => {
            if let Some(p) = model.previews.iter_mut().find(|p| p.id == id) {
                p.query_param_overrides.insert(name, value);
            }
            (generate(model, id, ctx), Outcome {
                mark_unsaved: false,
                error: None,
                source_edit: None,
            })
        }
        Msg::PreviewClicked {
            preview_id,
            u,
            v,
            rotate_x,
            rotate_y,
            zoom,
            aspect,
        } => {
            if let Some(p) = model.previews.iter().find(|p| p.id == preview_id) {
                if p.control_points.is_empty() {
                    model.selected_cp = None;
                    return (generate(model, preview_id, ctx), Outcome {
                        mark_unsaved: false,
                        error: None,
                        source_edit: None,
                    });
                }

                let cam = preview::CameraState::with_values(rotate_x, rotate_y, zoom);
                let (ray_origin, ray_dir) = preview::generate_ray_from_uv(
                    u,
                    v,
                    &cam,
                    p.scene.base_camera_distance(),
                    aspect,
                    p.scene.view_center(),
                );

                let aabb = compute_aabb_from_vertices(&p.last_vertices);
                let hit_radius = (aabb * 0.03).max(0.5) * 1.5;

                let mut best_hit: Option<(f64, usize)> = None;
                for (ci, cp) in p.control_points.iter().enumerate() {
                    let center = [cp.x.value, cp.y.value, cp.z.value];
                    if let Some(t) = preview::ray_sphere_intersect(
                        &ray_origin,
                        &ray_dir,
                        &center,
                        hit_radius as f64,
                    ) {
                        if best_hit.is_none() || t < best_hit.unwrap().0 {
                            best_hit = Some((t, ci));
                        }
                    }
                }

                if let Some((_, ci)) = best_hit {
                    model.selected_cp = Some((preview_id, ci));
                } else {
                    model.selected_cp = None;
                }

                return (generate(model, preview_id, ctx), Outcome {
                    mark_unsaved: false,
                    error: None,
                    source_edit: None,
                });
            }
            Outcome::none()
        }
        Msg::Export3MF(id) => {
            if let Some(p) = model.previews.iter().find(|p| p.id == id) {
                let vertices = p.last_vertices.clone();
                let indices = p.last_indices.clone();
                let query = p.query.clone();
                let base_name = ctx.base_name.to_string();
                let task = Task::perform(
                    async move {
                        let Some(data) = export::vertices_to_threemf(&vertices, &indices) else {
                            return Err("Nothing to export".to_string());
                        };
                        let file_name =
                            format!("{}_{}.3mf", base_name, sanitize_filename(&query));
                        let handle = rfd::AsyncFileDialog::new()
                            .set_title("Export 3MF")
                            .add_filter("3MF", &["3mf"])
                            .set_file_name(&file_name)
                            .save_file()
                            .await;
                        match handle {
                            Some(h) => std::fs::write(h.path(), data)
                                .map_err(|e| format!("Failed to write 3MF: {}", e)),
                            None => Ok(()),
                        }
                    },
                    Msg::ExportFinished,
                );
                (task, Outcome {
                    mark_unsaved: false,
                    error: None,
                    source_edit: None,
                })
            } else {
                Outcome::none()
            }
        }
        Msg::ExportBOM(id) => {
            if let Some(p) = model.previews.iter().find(|p| p.id == id) {
                if p.bom_entries.is_empty() {
                    return Outcome::none();
                }
                let json = cadhr_lang::bom::bom_entries_to_json(&p.bom_entries);
                let base_name = ctx.base_name.to_string();
                let task = Task::perform(
                    async move {
                        let handle = rfd::AsyncFileDialog::new()
                            .set_title("Export BOM")
                            .add_filter("JSON", &["json"])
                            .set_file_name(&format!("{}_bom.json", base_name))
                            .save_file()
                            .await;
                        match handle {
                            Some(h) => std::fs::write(h.path(), json.as_bytes())
                                .map_err(|e| format!("Failed to write BOM: {}", e)),
                            None => Ok(()),
                        }
                    },
                    Msg::ExportFinished,
                );
                (task, Outcome {
                    mark_unsaved: false,
                    error: None,
                    source_edit: None,
                })
            } else {
                Outcome::none()
            }
        }
        Msg::ExportFinished(result) => {
            let outcome = Outcome {
                mark_unsaved: false,
                error: result.err().map(|e| (e, None)),
                source_edit: None,
            };
            (Task::none(), outcome)
        }
        Msg::CpSourceEdit(preview_id, var_name, span) => {
            if span.file_id != 0 {
                return Outcome::none();
            }
            let value = model
                .previews
                .iter()
                .find(|p| p.id == preview_id)
                .and_then(|p| p.control_point_overrides.get(&var_name).copied());
            let Some(value) = value else {
                return Outcome::none();
            };
            let text = ctx.editor_text.clone();
            let start = span.start.min(text.len());
            let end = span.end.min(text.len());
            if !text.is_char_boundary(start) || !text.is_char_boundary(end) {
                return Outcome::none();
            }
            let new_text = format!(
                "{}{}{}",
                &text[..start],
                format_cp_value(value),
                &text[end..]
            );
            (
                generate(model, preview_id, ctx),
                Outcome {
                    mark_unsaved: true,
                    error: None,
                    source_edit: Some(new_text),
                },
            )
        }
    }
}

pub fn generate(model: &PreviewModel, id: u64, ctx: Context) -> Task<Msg> {
    let Some(preview) = model.previews.iter().find(|p| p.id == id) else {
        return Task::none();
    };
    let db = ctx.editor_text.clone();
    let query = preview.query.clone();
    let include_paths = ctx.include_paths.clone();

    match &preview.kind {
        PreviewKind::Normal => {
            let params = MeshJobParams {
                database: db,
                query,
                include_paths,
                control_point_overrides: preview.control_point_overrides.clone(),
                query_param_overrides: preview.query_param_overrides.clone(),
            };
            Task::perform(
                async move {
                    match std::thread::spawn(move || interpreter::run_mesh_job(params)).join() {
                        Ok(result) => result,
                        Err(_) => MeshJobResult::Error {
                            message: "Interpreter thread panicked".to_string(),
                            span: None,
                        },
                    }
                },
                move |result| Msg::PreviewGenerated(id, result),
            )
        }
        PreviewKind::Collision { .. } => {
            let params = CollisionJobParams {
                database: db,
                query,
                include_paths,
            };
            Task::perform(
                async move {
                    match std::thread::spawn(move || interpreter::run_collision_job(params)).join() {
                        Ok(result) => result,
                        Err(_) => CollisionJobResult::Error {
                            message: "Interpreter thread panicked".to_string(),
                            span: None,
                        },
                    }
                },
                move |result| Msg::CollisionGenerated(id, result),
            )
        }
    }
}

pub fn collect_session_previews(model: &PreviewModel) -> Vec<SessionPreview> {
    model
        .previews
        .iter()
        .enumerate()
        .map(|(i, p)| SessionPreview {
            preview_id: p.id,
            query: p.query.clone(),
            order: i,
            control_point_overrides: p.control_point_overrides.clone(),
            query_param_overrides: p.query_param_overrides.clone(),
            view_at_object_center: p.view_at_object_center,
        })
        .collect()
}

pub fn view(p: &Preview, index: usize, total: usize) -> Element<'_, Msg> {
    let id = p.id;

    let preview_label = match &p.kind {
        PreviewKind::Normal => format!("Preview {}", id),
        PreviewKind::Collision {
            part_count,
            collision_count,
        } => {
            if *collision_count > 0 {
                format!(
                    "Collision {} — {} parts, {} collision(s) ⚠",
                    id, part_count, collision_count
                )
            } else {
                format!("Collision {} — {} parts, no collisions ✓", id, part_count)
            }
        }
    };
    let up_btn = if index > 0 {
        parts::dark_button("↑").on_press(Msg::MovePreviewUp(id))
    } else {
        parts::dark_button("↑")
    };
    let down_btn = if index + 1 < total {
        parts::dark_button("↓").on_press(Msg::MovePreviewDown(id))
    } else {
        parts::dark_button("↓")
    };
    let view_label = if p.view_at_object_center {
        "View: Center"
    } else {
        "View: Origin"
    };
    let mut header = row![
        up_btn,
        down_btn,
        text(preview_label),
        parts::dark_button("Update").on_press(Msg::UpdatePreview(id)),
        parts::dark_button(view_label).on_press(Msg::ToggleViewCenter(id)),
        parts::dark_button("Export 3MF").on_press(Msg::Export3MF(id)),
    ]
    .spacing(4);
    if !p.bom_entries.is_empty() {
        header = header.push(parts::dark_button("Export BOM").on_press(Msg::ExportBOM(id)));
    }
    let header = header.push(parts::dark_button("Close").on_press(Msg::ClosePreview(id)));

    let query_row = row![
        text("?- "),
        text_input("query", &p.query)
            .on_input(move |q| Msg::QueryChanged(id, q))
            .on_submit(Msg::UpdatePreview(id)),
    ]
    .spacing(4);

    let shader_view: Element<'_, Msg> = Element::from(shader(&p.scene).width(Fill).height(300))
        .map(move |msg| match msg {
            preview::SceneMessage::Clicked {
                u,
                v,
                rotate_x,
                rotate_y,
                zoom,
                aspect,
            } => Msg::PreviewClicked {
                preview_id: id,
                u,
                v,
                rotate_x,
                rotate_y,
                zoom,
                aspect,
            },
        });

    let qp_items = p.query_params.iter().map(|qp| {
        let name = qp.name.clone();
        let (min_val, max_val) = query_param_range(qp);
        let current = p
            .query_param_overrides
            .get(&qp.name)
            .copied()
            .unwrap_or_else(|| {
                qp.default_value
                    .map(|dv| dv.to_f64())
                    .unwrap_or((min_val + max_val) / 2.0)
            });
        let qp_id = id;
        let qp_name = name.clone();
        row![
            text(format!("{}:", name)).width(80),
            slider(min_val..=max_val, current, move |v| {
                Msg::QpOverrideChanged(qp_id, qp_name.clone(), v)
            })
            .step(0.1)
            .width(Fill),
            text(format!("{:.1}", current)).width(60),
        ]
        .spacing(4)
        .into()
    });

    let cp_items = p.control_points.iter().enumerate().map(|(ci, cp)| {
        let label = cp
            .name
            .as_deref()
            .map(|n| format!("CP {}", n))
            .unwrap_or_else(|| format!("CP {}", ci));

        let axis_elements: Vec<Element<'_, Msg>> = [("X", &cp.x), ("Y", &cp.y), ("Z", &cp.z)]
            .iter()
            .enumerate()
            .flat_map(|(axis_idx, (axis_label, tracked))| {
                let val = cp.var_names[axis_idx]
                    .as_ref()
                    .and_then(|vn| p.control_point_overrides.get(vn).copied())
                    .unwrap_or(tracked.value);

                let slider_el: Option<Element<'_, Msg>> =
                    cp.var_names[axis_idx].as_ref().map(|var_name| {
                        let cp_id = id;
                        let vn = var_name.clone();
                        let source_span = tracked.source_span;
                        let range_half = (val.abs() + 50.0).max(50.0);
                        let mut sl =
                            slider((val - range_half)..=(val + range_half), val, move |v| {
                                Msg::CpOverrideChanged(cp_id, vn.clone(), v)
                            })
                            .step(0.5)
                            .width(80);
                        if let Some(span) = source_span.filter(|s| s.file_id == 0) {
                            let vn2 = var_name.clone();
                            sl = sl.on_release(Msg::CpSourceEdit(id, vn2, span));
                        }
                        sl.into()
                    });

                [
                    Some(text(*axis_label).into()),
                    slider_el,
                    Some(text(format!("{:.1}", val)).width(50).into()),
                ]
                .into_iter()
                .flatten()
            })
            .collect();

        let label_el: Element<'_, Msg> = text(label).width(80).into();
        row(std::iter::once(label_el).chain(axis_elements))
            .spacing(4)
            .into()
    });

    column(
        [header.into(), query_row.into(), shader_view.into()]
            .into_iter()
            .chain(qp_items)
            .chain(cp_items),
    )
    .spacing(4)
    .into()
}

fn compute_aabb_from_vertices(vertices: &[preview::pipeline::Vertex]) -> f32 {
    let mut max_extent: f32 = 0.0;
    for v in vertices {
        for &c in &v.position {
            max_extent = max_extent.max(c.abs());
        }
    }
    max_extent
}

fn query_param_range(qp: &QueryParam) -> (f64, f64) {
    let min = qp.min.as_ref().map(|b| b.value.to_f64()).unwrap_or(-100.0);
    let max = qp.max.as_ref().map(|b| b.value.to_f64()).unwrap_or(100.0);
    (min, max)
}

fn sanitize_filename(s: &str) -> String {
    s.chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect::<String>()
        .trim_matches('_')
        .to_string()
}

fn format_cp_value(value: f64) -> String {
    let s = format!("{:.6}", value);
    let s = s.trim_end_matches('0');
    let s = s.trim_end_matches('.');
    s.to_string()
}

impl Default for Outcome {
    fn default() -> Self {
        Self {
            mark_unsaved: false,
            error: None,
            source_edit: None,
        }
    }
}