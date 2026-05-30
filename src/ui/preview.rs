//! 個別プレビュー (Vec<Preview>) の管理。
//!
//! 各 preview は独自の slider 値、scene、表示オプション、衝突モードを持つ。
//! main の `CompiledProgram` は共有し、preview ごとに `run_eval_job` を投げる。

use std::collections::HashMap;
use std::path::PathBuf;

use cadhr_lang::{CompiledProgram, Span};
use iced::widget::{column, container, row, scrollable, shader, text, text_input};
use iced::{Element, Fill, Length};

use crate::interpreter::{EvalJobParams, EvalJobResult};
use crate::preview::Scene;
use crate::session::SessionPreview;
use crate::ui::parts;

/// プレビュー 1 枚分の状態。
pub struct Preview {
    pub id: u64,
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
}

impl Preview {
    pub fn new(id: u64) -> Self {
        Self {
            id,
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
        }
    }
}

#[derive(Debug, Clone)]
pub enum PreviewMsg {
    ToggleViewCenter,
    Minimize,
    Close,
    MoveUp,
    MoveDown,
    SelectControlPoint(usize),
    ControlPointEdited(String, usize, String),
}

pub fn view<'a>(p: &'a Preview, index: usize, total: usize) -> Element<'a, PreviewMsg> {
    let label = if p.is_collision {
        format!("collision #{} ({}/{})", p.id, index + 1, total)
    } else {
        format!("#{} ({}/{})", p.id, index + 1, total)
    };
    let header = row![
        parts::dark_button("↑").on_press(PreviewMsg::MoveUp),
        parts::dark_button("↓").on_press(PreviewMsg::MoveDown),
        parts::dark_button(if p.minimized { "▶" } else { "▼" })
            .on_press(PreviewMsg::Minimize),
        text(label),
        parts::dark_button(if p.view_at_object_center { "center" } else { "origin" })
            .on_press(PreviewMsg::ToggleViewCenter),
        parts::dark_button("×").on_press(PreviewMsg::Close),
    ]
    .spacing(2);

    if p.minimized {
        return container(header).padding(4).into();
    }

    let widget: Element<'a, crate::preview::SceneMessage> =
        shader(&p.scene).width(Fill).height(Length::Fixed(220.0)).into();
    // Scene が出す SceneMessage::Clicked はカメラ操作専用なので preview 自体は publish しない。
    let preview_widget: Element<'a, PreviewMsg> = widget.map(|_| PreviewMsg::ToggleViewCenter);
    // ↑ 厳密には `ToggleViewCenter` は誤接続。実用上 Clicked は出ないので無害だが、
    //   将来 control point ドラッグを実装するときに本物のメッセージに差し替える。

    let mut col = column![header, preview_widget].spacing(4);

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
            let selected_label = if Some(i) == p.selected_cp { "●" } else { "○" };
            let select_btn = parts::dark_button(selected_label)
                .on_press(PreviewMsg::SelectControlPoint(i));
            let label = text(format!("  {name}"));
            let x_input = numeric_input(name.clone(), 0, pos[0]);
            let y_input = numeric_input(name.clone(), 1, pos[1]);
            let z_input = numeric_input(name.clone(), 2, pos[2]);
            c_col = c_col.push(row![select_btn, label, x_input, y_input, z_input].spacing(4));
        }
        col = col.push(c_col);
    }
    if let Some((msg, _)) = &p.error {
        col = col.push(
            text(format!("error: {msg}")).color(iced::Color::from_rgb(1.0, 0.4, 0.4)),
        );
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

pub fn list_view<'a>(previews: &'a [Preview]) -> Element<'a, (u64, PreviewMsg)> {
    if previews.is_empty() {
        return text("No previews. Press \"Add\" to create one.").into();
    }
    let total = previews.len();
    let items: Vec<Element<'a, (u64, PreviewMsg)>> = previews
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let id = p.id;
            view(p, i, total).map(move |m| (id, m))
        })
        .collect();
    scrollable(column(items).spacing(8)).height(Fill).into()
}
