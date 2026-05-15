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
use crate::session;
use crate::ui::parts;

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
    pub minimized: bool,
    pub selected_cp: Option<usize>,
}

impl Preview {
    pub fn new(id: u64) -> Self {
        Self {
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
            minimized: false,
            selected_cp: None,
        }
    }

    pub fn new_collision(id: u64) -> Self {
        let mut scene = Scene::new();
        scene.color = [0.4, 0.5, 0.7, 0.7];
        Self {
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
            minimized: false,
            selected_cp: None,
        }
    }

    pub fn from_session(sp: &SessionPreview) -> Self {
        let mut scene = Scene::new();
        scene.view_at_object_center = sp.view_at_object_center;
        Self {
            id: sp.preview_id,
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
            minimized: sp.minimized,
            selected_cp: None,
        }
    }

    pub fn to_session(&self, order: usize) -> SessionPreview {
        SessionPreview {
            preview_id: self.id,
            query: self.query.clone(),
            order,
            control_point_overrides: self.control_point_overrides.clone(),
            query_param_overrides: self.query_param_overrides.clone(),
            view_at_object_center: self.view_at_object_center,
            minimized: self.minimized,
        }
    }
}

#[derive(Clone)]
pub struct Context {
    pub editor_text: String,
    pub include_paths: Vec<PathBuf>,
    pub base_name: String,
}

#[derive(Default)]
pub struct Outcome {
    pub mark_unsaved: bool,
    pub error: Option<(String, Option<SrcSpan>)>,
    pub source_edit: Option<String>,
}

#[derive(Debug, Clone)]
pub enum Msg {
    Update,
    ToggleMinimize,
    ToggleViewCenter,
    QueryChanged(String),
    Generated(MeshJobResult),
    CollisionGenerated(CollisionJobResult),
    CpOverrideChanged(String, f64),
    QpOverrideChanged(String, f64),
    Clicked {
        u: f32,
        v: f32,
        rotate_x: f64,
        rotate_y: f64,
        zoom: f32,
        aspect: f32,
    },
    Export3MF,
    ExportBOM,
    ExportFinished(Result<(), String>),
    CpSourceEdit(String, SrcSpan),

    // 親が intercept する list 操作
    MoveUp,
    MoveDown,
    Close,
}

pub fn update(p: &mut Preview, msg: Msg, ctx: Context) -> (Task<Msg>, Outcome) {
    match msg {
        Msg::Update => (generate(p, ctx), Outcome::default()),
        Msg::ToggleMinimize => {
            p.minimized = !p.minimized;
            (
                Task::none(),
                Outcome {
                    mark_unsaved: true,
                    ..Default::default()
                },
            )
        }
        Msg::ToggleViewCenter => {
            p.view_at_object_center = !p.view_at_object_center;
            p.scene.view_at_object_center = p.view_at_object_center;
            // 注視点の切り替えだけでは shader の再描画が走らないので
            // 現メッシュを再アップロードしてバージョンを進める
            let vertices = p.last_vertices.clone();
            let indices = p.last_indices.clone();
            if !vertices.is_empty() {
                match &p.kind {
                    PreviewKind::Normal => {
                        p.scene.set_mesh_with_control_points(
                            vertices,
                            indices,
                            &p.control_points,
                            p.selected_cp,
                        );
                    }
                    PreviewKind::Collision { .. } => {
                        p.scene.set_mesh(vertices, indices);
                    }
                }
            }
            (
                Task::none(),
                Outcome {
                    mark_unsaved: true,
                    ..Default::default()
                },
            )
        }
        Msg::QueryChanged(q) => {
            p.query = q;
            (Task::none(), Outcome::default())
        }
        Msg::Generated(result) => {
            debug_log!("MeshJob completed (id={}): {:?}", p.id, result);
            let mut outcome = Outcome::default();
            match result {
                MeshJobResult::Success {
                    vertices,
                    indices,
                    control_points,
                    query_params,
                    bom_entries,
                    resolved_terms_debug,
                    ..
                } => {
                    debug_log!("MeshJob resolved terms:\n{}", resolved_terms_debug);
                    p.last_vertices = vertices.clone();
                    p.last_indices = indices.clone();
                    p.control_points = control_points;
                    p.query_params = query_params;
                    p.bom_entries = bom_entries;
                    p.scene.set_mesh_with_control_points(
                        vertices,
                        indices,
                        &p.control_points,
                        p.selected_cp,
                    );
                }
                MeshJobResult::Error { message, span } => {
                    outcome.error = Some((message, span));
                }
            }
            (Task::none(), outcome)
        }
        Msg::CollisionGenerated(result) => {
            debug_log!("CollisionJob completed (id={}): {:?}", p.id, result);
            let mut outcome = Outcome::default();
            match result {
                CollisionJobResult::Success {
                    vertices,
                    indices,
                    part_count,
                    collision_count,
                } => {
                    p.last_vertices = vertices.clone();
                    p.last_indices = indices.clone();
                    p.kind = PreviewKind::Collision {
                        part_count,
                        collision_count,
                    };
                    p.scene.set_mesh(vertices, indices);
                }
                CollisionJobResult::Error { message, span } => {
                    outcome.error = Some((message, span));
                }
            }
            (Task::none(), outcome)
        }
        Msg::CpOverrideChanged(var_name, value) => {
            p.control_point_overrides.insert(var_name, value);
            (generate(p, ctx), Outcome::default())
        }
        Msg::QpOverrideChanged(name, value) => {
            p.query_param_overrides.insert(name, value);
            (generate(p, ctx), Outcome::default())
        }
        Msg::Clicked {
            u,
            v,
            rotate_x,
            rotate_y,
            zoom,
            aspect,
        } => {
            if p.control_points.is_empty() {
                p.selected_cp = None;
                return (generate(p, ctx), Outcome::default());
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

            p.selected_cp = best_hit.map(|(_, ci)| ci);
            (generate(p, ctx), Outcome::default())
        }
        Msg::Export3MF => {
            let vertices = p.last_vertices.clone();
            let indices = p.last_indices.clone();
            let query = p.query.clone();
            let base_name = ctx.base_name.to_string();
            let task = Task::perform(
                async move {
                    let Some(data) = export::vertices_to_threemf(&vertices, &indices) else {
                        return Err("Nothing to export".to_string());
                    };
                    let file_name = format!("{}_{}.3mf", base_name, sanitize_filename(&query));
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
            (task, Outcome::default())
        }
        Msg::ExportBOM => {
            if p.bom_entries.is_empty() {
                return (Task::none(), Outcome::default());
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
            (task, Outcome::default())
        }
        Msg::ExportFinished(result) => {
            let outcome = Outcome {
                error: result.err().map(|e| (e, None)),
                ..Default::default()
            };
            (Task::none(), outcome)
        }
        Msg::CpSourceEdit(var_name, span) => {
            if span.file_id != 0 {
                return (Task::none(), Outcome::default());
            }
            let Some(value) = p.control_point_overrides.get(&var_name).copied() else {
                return (Task::none(), Outcome::default());
            };
            let text = ctx.editor_text.clone();
            let start = span.start.min(text.len());
            let end = span.end.min(text.len());
            if !text.is_char_boundary(start) || !text.is_char_boundary(end) {
                return (Task::none(), Outcome::default());
            }
            let new_text = format!(
                "{}{}{}",
                &text[..start],
                format_cp_value(value),
                &text[end..]
            );
            (
                generate(p, ctx),
                Outcome {
                    mark_unsaved: true,
                    source_edit: Some(new_text),
                    ..Default::default()
                },
            )
        }
        // 親が intercept するので preview::update には来ない
        Msg::MoveUp | Msg::MoveDown | Msg::Close => (Task::none(), Outcome::default()),
    }
}

pub fn generate(p: &Preview, ctx: Context) -> Task<Msg> {
    let db = ctx.editor_text.clone();
    let query = p.query.clone();
    let include_paths = ctx.include_paths.clone();

    match &p.kind {
        PreviewKind::Normal => {
            let params = MeshJobParams {
                database: db,
                query,
                include_paths,
                control_point_overrides: p.control_point_overrides.clone(),
                query_param_overrides: p.query_param_overrides.clone(),
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
                Msg::Generated,
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
                    match std::thread::spawn(move || interpreter::run_collision_job(params)).join()
                    {
                        Ok(result) => result,
                        Err(_) => CollisionJobResult::Error {
                            message: "Interpreter thread panicked".to_string(),
                            span: None,
                        },
                    }
                },
                Msg::CollisionGenerated,
            )
        }
    }
}

pub fn view(p: &Preview, index: usize, total: usize) -> Element<'_, Msg> {
    let preview_label = match &p.kind {
        PreviewKind::Normal => format!("Preview {}", p.id),
        PreviewKind::Collision {
            part_count,
            collision_count,
        } => {
            if *collision_count > 0 {
                format!(
                    "Collision {} — {} parts, {} collision(s) ⚠",
                    p.id, part_count, collision_count
                )
            } else {
                format!(
                    "Collision {} — {} parts, no collisions ✓",
                    p.id, part_count
                )
            }
        }
    };
    let up_btn = if index > 0 {
        parts::dark_button("↑").on_press(Msg::MoveUp)
    } else {
        parts::dark_button("↑")
    };
    let down_btn = if index + 1 < total {
        parts::dark_button("↓").on_press(Msg::MoveDown)
    } else {
        parts::dark_button("↓")
    };
    let view_label = if p.view_at_object_center {
        "View: Center"
    } else {
        "View: Origin"
    };
    let minimize_label = if p.minimized { "+" } else { "−" };
    let mut header = row![
        up_btn,
        down_btn,
        text(preview_label),
        parts::dark_button("Update").on_press(Msg::Update),
        parts::dark_button(view_label).on_press(Msg::ToggleViewCenter),
        parts::dark_button("Export 3MF").on_press(Msg::Export3MF),
    ]
    .spacing(4);
    if !p.bom_entries.is_empty() {
        header = header.push(parts::dark_button("Export BOM").on_press(Msg::ExportBOM));
    }
    let header = header
        .push(parts::dark_button(minimize_label).on_press(Msg::ToggleMinimize))
        .push(parts::dark_button("Close").on_press(Msg::Close));

    if p.minimized {
        return column![header].spacing(4).into();
    }

    let query_row = row![
        text("?- "),
        text_input("query", &p.query)
            .on_input(Msg::QueryChanged)
            .on_submit(Msg::Update),
    ]
    .spacing(4);

    let shader_view: Element<'_, Msg> = Element::from(shader(&p.scene).width(Fill).height(300))
        .map(|msg| match msg {
            preview::SceneMessage::Clicked {
                u,
                v,
                rotate_x,
                rotate_y,
                zoom,
                aspect,
            } => Msg::Clicked {
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
                    .as_ref()
                    .map(|dv| dv.to_f64())
                    .unwrap_or((min_val + max_val) / 2.0)
            });
        let qp_name = name.clone();
        row![
            text(format!("{}:", name)).width(80),
            slider(min_val..=max_val, current, move |v| {
                Msg::QpOverrideChanged(qp_name.clone(), v)
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
                        let vn = var_name.clone();
                        let source_span = tracked.source_span;
                        // 軸 Var に range 注釈があればそれを使う(`0<X<100` などで指定された範囲)。
                        // 無ければ広めの固定範囲 -1000..1000 を使う(よくある CAD スケールを
                        // カバーしつつ、注釈が無いユーザにも一定の操作性を提供)。
                        let (slider_lo, slider_hi) =
                            cp.axis_ranges[axis_idx].unwrap_or((-1000.0, 1000.0));
                        let mut sl = slider(slider_lo..=slider_hi, val, move |v| {
                            Msg::CpOverrideChanged(vn.clone(), v)
                        })
                        .step(0.5)
                        .width(80);
                        if let Some(span) = source_span.filter(|s| s.file_id == 0) {
                            let vn2 = var_name.clone();
                            sl = sl.on_release(Msg::CpSourceEdit(vn2, span));
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
