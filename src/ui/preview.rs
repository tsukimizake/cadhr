//! 個別プレビュー (Vec<Preview>) の管理。
//!
//! 各 preview は target binding 名・独自の slider 値・scene・表示オプションを持つ。
//! `CompiledProgram` は共有し、preview ごとに `run_binding` を投げる。

use std::collections::HashMap;
use std::path::PathBuf;

use cadhr_lang::{BindingSignature, CompiledProgram, Span};
use iced::widget::{column, combo_box, container, row, shader, slider, text, text_input};
use iced::{Element, Fill, Length};

use crate::interpreter::{EvalJobParams, EvalJobResult};
use crate::preview::Scene;
use crate::preview::pipeline::Vertex;
use crate::session::SessionPreview;
use crate::ui::parts;
use crate::ui::workspace::WorkspaceEvent;

pub struct Preview {
    pub id: u64,
    /// 評価する top-level binding の名前。`"main"` がデフォルト。
    pub target: String,
    /// combo_box widget の内部状態。candidate 一覧が変わった時に作り直す。
    pub target_state: combo_box::State<String>,
    pub scene: Scene,
    pub slider_values: HashMap<String, f64>,
    pub bom: Vec<String>,
    pub view_at_object_center: bool,
    pub minimized: bool,
    pub is_collision: bool,
    pub error: Option<(String, Option<Span>)>,
    /// `control3d` で記録された名前付き 3D 点 (descriptor)。
    pub control_points: Vec<(String, [f64; 3])>,
    /// ユーザがドラッグして上書きした control point の現在位置。
    pub control_overrides: HashMap<String, [f64; 3]>,
    /// 選択中の control point index (UI 用)。
    pub selected_cp: Option<usize>,
    /// 直近の eval で得た mesh。3MF export 用 (Scene の mesh は control point の sphere が混入する)。
    pub last_vertices: Vec<Vertex>,
    pub last_indices: Vec<u32>,
}

impl Preview {
    pub fn new(id: u64) -> Self {
        Self {
            id,
            target: "main".to_string(),
            target_state: combo_box::State::new(Vec::new()),
            scene: Scene::new(),
            slider_values: HashMap::new(),
            bom: Vec::new(),
            view_at_object_center: false,
            minimized: false,
            is_collision: false,
            error: None,
            control_points: Vec::new(),
            control_overrides: HashMap::new(),
            selected_cp: None,
            last_vertices: Vec::new(),
            last_indices: Vec::new(),
        }
    }

    pub fn new_collision(id: u64) -> Self {
        let mut p = Self::new(id);
        p.is_collision = true;
        p.scene.color = [0.4, 0.5, 0.7, 0.7];
        p
    }

    pub fn from_session(sp: &SessionPreview) -> Self {
        let mut p = if sp.is_collision {
            Self::new_collision(sp.preview_id)
        } else {
            Self::new(sp.preview_id)
        };
        p.target = sp.target_name.clone();
        p.slider_values = sp.slider_values.clone();
        p.control_overrides = sp.control_point_overrides.clone();
        p.view_at_object_center = sp.view_at_object_center;
        p.minimized = sp.minimized;
        p.scene.view_at_object_center = sp.view_at_object_center;
        p
    }

    pub fn to_session(&self, order: usize) -> SessionPreview {
        SessionPreview {
            preview_id: self.id,
            order,
            target_name: self.target.clone(),
            slider_values: self.slider_values.clone(),
            control_point_overrides: self.control_overrides.clone(),
            view_at_object_center: self.view_at_object_center,
            minimized: self.minimized,
            is_collision: self.is_collision,
        }
    }

    pub fn apply_eval_result(&mut self, result: EvalJobResult) {
        match result {
            EvalJobResult::Success {
                vertices,
                indices,
                bom,
                control_points,
            } => {
                self.control_points = control_points;
                self.last_vertices = vertices.clone();
                self.last_indices = indices.clone();
                self.scene.set_mesh_with_control_points(
                    vertices,
                    indices,
                    &self.control_points,
                    self.selected_cp,
                );
                self.bom = bom;
                self.error = None;
            }
            EvalJobResult::Error { message, span } => {
                self.error = Some((message, span));
            }
        }
    }

    pub fn build_eval_params(
        &self,
        program: &CompiledProgram,
        search_paths: Vec<PathBuf>,
    ) -> EvalJobParams {
        EvalJobParams {
            program: program.clone(),
            slider_values: self.slider_values.clone(),
            search_paths,
            control_overrides: self.control_overrides.clone(),
            target: self.target.clone(),
        }
    }

    /// candidate 一覧が変わった時に呼ぶ。combo_box state を作り直して、現在 target
    /// を維持する。
    pub fn refresh_candidates(&mut self, names: &[String]) {
        self.target_state = combo_box::State::new(names.to_vec());
    }

    /// preview 内で完結する状態遷移を適用し、Model レベルの後処理を返す。
    pub fn update(&mut self, msg: PreviewMsg) -> WorkspaceEvent {
        match msg {
            PreviewMsg::SceneIgnored => WorkspaceEvent::None,
            PreviewMsg::Close => WorkspaceEvent::Close,
            PreviewMsg::MoveUp => WorkspaceEvent::MoveUp,
            PreviewMsg::MoveDown => WorkspaceEvent::MoveDown,
            PreviewMsg::Minimize => {
                self.minimized = !self.minimized;
                WorkspaceEvent::Edited
            }
            PreviewMsg::ToggleViewCenter => {
                self.view_at_object_center = !self.view_at_object_center;
                self.scene.view_at_object_center = self.view_at_object_center;
                WorkspaceEvent::Edited
            }
            PreviewMsg::SelectControlPoint(i) => {
                self.selected_cp = Some(i);
                WorkspaceEvent::EvalNeeded { edited: false }
            }
            PreviewMsg::ControlPointEdited(name, axis, value_str) => {
                let Ok(v) = value_str.parse::<f64>() else {
                    return WorkspaceEvent::None;
                };
                // 既存値がなければ control_points から default を引いて初期化
                let entry = self.control_overrides.entry(name.clone()).or_insert_with(|| {
                    self.control_points
                        .iter()
                        .find(|(n, _)| n == &name)
                        .map(|(_, pos)| *pos)
                        .unwrap_or([0.0, 0.0, 0.0])
                });
                if axis < 3 {
                    entry[axis] = v;
                }
                WorkspaceEvent::EvalNeeded { edited: true }
            }
            PreviewMsg::Export3MF => WorkspaceEvent::ExportRequested(
                crate::export::vertices_to_threemf(&self.last_vertices, &self.last_indices),
            ),
            PreviewMsg::TargetChanged(new_target) => {
                if self.target == new_target {
                    return WorkspaceEvent::None;
                }
                self.target = new_target;
                WorkspaceEvent::TargetChanged
            }
            PreviewMsg::ArgChanged(name, v) => {
                self.slider_values.insert(name, v);
                WorkspaceEvent::EvalNeeded { edited: true }
            }
        }
    }
}

#[derive(Debug, Clone)]
pub enum PreviewMsg {
    /// Scene が出す SceneMessage (カメラ操作) を preview レイヤーで握りつぶすための no-op。
    SceneIgnored,
    ToggleViewCenter,
    Minimize,
    Close,
    MoveUp,
    MoveDown,
    SelectControlPoint(usize),
    ControlPointEdited(String, usize, String),
    Export3MF,
    /// combo_box で binding 名が確定された (selection or text submit)。
    TargetChanged(String),
    /// preview 固有の引数 slider/text 値変更 (引数名、新値)。
    ArgChanged(String, f64),
}

pub fn view<'a>(
    p: &'a Preview,
    index: usize,
    total: usize,
    candidate_signatures: &'a [BindingSignature],
) -> Element<'a, PreviewMsg> {
    let current_sig: Option<&BindingSignature> =
        candidate_signatures.iter().find(|s| s.name == p.target);
    let label = if p.is_collision {
        format!("collision #{} ({}/{})", p.id, index + 1, total)
    } else {
        format!("#{} ({}/{})", p.id, index + 1, total)
    };
    let mut header = row![
        parts::dark_button("↑").on_press(PreviewMsg::MoveUp),
        parts::dark_button("↓").on_press(PreviewMsg::MoveDown),
        parts::dark_button(if p.minimized { "▶" } else { "▼" }).on_press(PreviewMsg::Minimize),
        text(label),
        parts::dark_button(if p.view_at_object_center {
            "center"
        } else {
            "origin"
        })
        .on_press(PreviewMsg::ToggleViewCenter),
    ]
    .spacing(2);

    if !p.is_collision {
        // combo_box で target binding を選択。on_input でテキスト変更を、on_selected
        // で候補選択を拾い、どちらも TargetChanged にまとめる。
        let cb = combo_box(
            &p.target_state,
            "binding",
            Some(&p.target),
            PreviewMsg::TargetChanged,
        )
        .on_input(PreviewMsg::TargetChanged)
        .width(Length::Fixed(160.0))
        .size(13.0);
        header = header.push(cb);
    }

    // collision preview は表示専用なので 3MF export は出さない
    if !p.is_collision && !p.last_vertices.is_empty() {
        header = header.push(parts::dark_button("Export 3MF").on_press(PreviewMsg::Export3MF));
    }
    let header = header.push(parts::dark_button("×").on_press(PreviewMsg::Close));

    if p.minimized {
        return container(header).padding(4).into();
    }

    let widget: Element<'a, crate::preview::SceneMessage> = shader(&p.scene)
        .width(Fill)
        .height(Length::Fixed(220.0))
        .into();
    let preview_widget: Element<'a, PreviewMsg> = widget.map(|_| PreviewMsg::SceneIgnored);

    let mut col = column![header, preview_widget].spacing(4);

    // collision 以外は当該 binding の引数 widget を出す
    if !p.is_collision {
        if let Some(sig) = current_sig {
            if !sig.params.is_empty() {
                let mut s_col = column![].spacing(4);
                for param in &sig.params {
                    let name = param.name.clone();
                    let cur = p
                        .slider_values
                        .get(&name)
                        .copied()
                        .or_else(|| param.range.as_ref().map(|r| (r.lo + r.hi) / 2.0))
                        .unwrap_or(0.0);
                    let label = text(format!("{}: {:.2}", name, cur));
                    let widget_el: Element<'a, PreviewMsg> = if let Some(r) = &param.range {
                        let lo = r.lo as f32;
                        let hi = r.hi as f32;
                        let name_for_slider = name.clone();
                        slider(lo..=hi, cur as f32, move |v| {
                            PreviewMsg::ArgChanged(name_for_slider.clone(), v as f64)
                        })
                        .into()
                    } else {
                        let s = format!("{cur:.2}");
                        let name_for_input = name.clone();
                        text_input("", &s)
                            .on_input(move |new| {
                                let v = new.parse::<f64>().unwrap_or(0.0);
                                PreviewMsg::ArgChanged(name_for_input.clone(), v)
                            })
                            .width(Length::Fixed(80.0))
                            .into()
                    };
                    s_col = s_col.push(row![label, widget_el].spacing(8));
                }
                col = col.push(s_col);
            }
        } else if !p.target.is_empty()
            && !candidate_signatures.iter().any(|s| s.name == p.target)
        {
            col = col.push(
                text(format!("warning: binding `{}` not previewable", p.target))
                    .color(iced::Color::from_rgb(1.0, 0.7, 0.4)),
            );
        }
    }

    if !p.bom.is_empty() {
        let mut b_col = column![text("BOM:").size(13)].spacing(2);
        for b in &p.bom {
            b_col = b_col.push(text(format!("  • {b}")));
        }
        col = col.push(b_col);
    }
    if !p.control_points.is_empty() {
        let mut c_col = column![text("Control Points:").size(13)].spacing(2);
        for (i, (name, pos)) in p.control_points.iter().enumerate() {
            let selected_label = if Some(i) == p.selected_cp {
                "●"
            } else {
                "○"
            };
            let select_btn =
                parts::dark_button(selected_label).on_press(PreviewMsg::SelectControlPoint(i));
            let label = text(format!("  {name}"));
            let x_input = numeric_input(name.clone(), 0, pos[0]);
            let y_input = numeric_input(name.clone(), 1, pos[1]);
            let z_input = numeric_input(name.clone(), 2, pos[2]);
            c_col = c_col.push(row![select_btn, label, x_input, y_input, z_input].spacing(4));
        }
        col = col.push(c_col);
    }
    if let Some((msg, _)) = &p.error {
        col = col.push(text(format!("error: {msg}")).color(iced::Color::from_rgb(1.0, 0.4, 0.4)));
    }
    container(col).padding(4).into()
}

fn numeric_input<'a>(name: String, axis: usize, value: f64) -> Element<'a, PreviewMsg> {
    let s = format!("{value:.2}");
    text_input("", &s)
        .on_input(move |new| PreviewMsg::ControlPointEdited(name.clone(), axis, new))
        .width(Length::Fixed(60.0))
        .into()
}

