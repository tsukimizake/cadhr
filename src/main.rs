mod debug;
mod export;
mod highlight;
mod interpreter;
mod preview;
mod session;
mod ui;

use std::path::PathBuf;

use cadhr_lang::{CompiledProgram, MainSignature, Span};
use iced::widget::{
    column, container, pane_grid, pick_list, row, scrollable, slider, text, text_editor, toggler,
};
use iced::{Element, Fill, Length, Subscription, Task};
use interpreter::{CompileJobParams, CompileJobResult, EvalJobResult};
use ui::parts;
use ui::preview::{Preview, PreviewMsg};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PaneKind {
    Editor,
    Preview,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AddKind {
    Preview,
    Collision,
}

impl AddKind {
    const ALL: [AddKind; 2] = [AddKind::Preview, AddKind::Collision];
}

impl std::fmt::Display for AddKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AddKind::Preview => write!(f, "Preview"),
            AddKind::Collision => write!(f, "Collision Check"),
        }
    }
}

const DEFAULT_EDITOR_TEXT: &str =
    "main length =\n    cube 10.0 10.0 length\n\nslider length = 6.0 .. 80.0\n";

trait DialogHandler {
    fn open_session(&self) -> Task<Msg>;
    fn save_session_as(
        &self,
        editor_text: String,
        previews: Vec<session::SessionPreview>,
    ) -> Task<Msg>;
}

struct RfdDialogs;

impl DialogHandler for RfdDialogs {
    fn open_session(&self) -> Task<Msg> {
        Task::perform(
            async {
                let handle = rfd::AsyncFileDialog::new()
                    .set_title("Open Session Directory")
                    .pick_folder()
                    .await?;
                let path = handle.path().to_path_buf();
                let (db, previews) = session::load_session(&path)?;
                Some((path, db, previews))
            },
            Msg::SessionOpened,
        )
    }

    fn save_session_as(&self, text: String, previews: Vec<session::SessionPreview>) -> Task<Msg> {
        Task::perform(
            async move {
                let handle = rfd::AsyncFileDialog::new()
                    .set_title("Save Session Directory")
                    .set_file_name("untitled")
                    .save_file()
                    .await;
                match handle {
                    Some(h) => {
                        let path = h.path().to_path_buf();
                        session::save_session(&path, &text, &previews).map(|()| path)
                    }
                    None => Err("Cancelled".to_string()),
                }
            },
            Msg::SessionSaved,
        )
    }
}

fn main() -> iced::Result {
    iced::application(init, update, view)
        .title("cadhr")
        .subscription(subscription)
        .run()
}

struct Model {
    editor: text_editor::Content,
    /// 複数プレビュー。`previews[0]` が「主プレビュー」、それ以降は追加。
    previews: Vec<Preview>,
    next_preview_id: u64,
    program: Option<CompiledProgram>,
    signature: MainSignature,
    error_message: String,
    error_span: Option<Span>,
    diagnostics: Vec<String>,
    current_file_path: Option<PathBuf>,
    auto_reload: bool,
    last_modified: Option<std::time::SystemTime>,
    unsaved: bool,
    dialogs: Box<dyn DialogHandler>,
    panes: pane_grid::State<PaneKind>,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
enum Msg {
    EditorAction(text_editor::Action),
    UpdatePreviews,
    SliderChanged(u64, String, f32),
    CompileDone(CompileJobResult),
    EvalDone(u64, EvalJobResult),
    Preview(u64, PreviewMsg),

    AddPreview,
    AddCollisionCheck,

    NewSession,
    OpenSession,
    SaveSession,
    SaveSessionAs,
    SessionOpened(Option<(PathBuf, String, session::SessionPreviews)>),
    SessionSaved(Result<PathBuf, String>),

    ToggleAutoReload,
    CheckFileChanged,

    PaneResized(pane_grid::ResizeEvent),
    ToggleEditor,
}

fn init() -> (Model, Task<Msg>) {
    let (mut panes, editor_pane) = pane_grid::State::new(PaneKind::Editor);
    let (_, split) = panes
        .split(pane_grid::Axis::Vertical, editor_pane, PaneKind::Preview)
        .expect("failed to create initial pane split");
    // 既存の FillPortion(3) : FillPortion(2) と同じ 0.6 比率
    panes.resize(split, 0.6);

    let mut model = Model {
        editor: text_editor::Content::with_text(DEFAULT_EDITOR_TEXT),
        previews: vec![Preview::new(0)],
        next_preview_id: 1,
        program: None,
        signature: MainSignature::default(),
        error_message: String::new(),
        error_span: None,
        diagnostics: Vec::new(),
        current_file_path: None,
        auto_reload: false,
        last_modified: None,
        unsaved: false,
        dialogs: Box::new(RfdDialogs),
        panes,
    };

    if let Some(path) = session::restore_last_session_path() {
        if let Some((db, previews)) = session::load_session(&path) {
            model.editor = text_editor::Content::with_text(&db);
            model.current_file_path = Some(path);
            if !previews.previews.is_empty() {
                model.previews = previews
                    .previews
                    .iter()
                    .map(Preview::from_session)
                    .collect();
                model.next_preview_id = model.previews.iter().map(|p| p.id).max().unwrap_or(0) + 1;
            }
        }
    }
    let task = spawn_compile_job(&model);
    (model, task)
}

fn search_paths(model: &Model) -> Vec<PathBuf> {
    let mut paths = Vec::new();
    if let Some(dir) = &model.current_file_path {
        if let Some(parent) = dir.parent() {
            paths.push(parent.to_path_buf());
        }
        paths.push(dir.clone());
    }
    paths
}

fn spawn_compile_job(model: &Model) -> Task<Msg> {
    let params = CompileJobParams {
        source: model.editor.text(),
        search_paths: search_paths(model),
    };
    Task::perform(
        async move {
            std::thread::spawn(move || interpreter::run_compile_job(params))
                .join()
                .expect("compile worker panicked")
        },
        Msg::CompileDone,
    )
}

fn spawn_eval_for(model: &Model, preview_id: u64) -> Task<Msg> {
    let Some(prog) = model.program.clone() else {
        return Task::none();
    };
    let Some(p) = model.previews.iter().find(|p| p.id == preview_id) else {
        return Task::none();
    };
    let params = p.build_eval_params(&prog, search_paths(model));
    let is_collision = p.is_collision;
    Task::perform(
        async move {
            std::thread::spawn(move || {
                if is_collision {
                    interpreter::run_collision_job(params)
                } else {
                    interpreter::run_eval_job(params)
                }
            })
            .join()
            .expect("eval worker panicked")
        },
        move |r| Msg::EvalDone(preview_id, r),
    )
}

fn spawn_eval_all(model: &Model) -> Task<Msg> {
    let tasks: Vec<Task<Msg>> = model
        .previews
        .iter()
        .map(|p| spawn_eval_for(model, p.id))
        .collect();
    Task::batch(tasks)
}

fn collect_session_previews(model: &Model) -> Vec<session::SessionPreview> {
    model
        .previews
        .iter()
        .enumerate()
        .map(|(i, p)| p.to_session(i))
        .collect()
}

fn apply_signature_defaults(model: &mut Model) {
    for p in &mut model.previews {
        for param in &model.signature.params {
            if let Some(r) = &param.range {
                p.slider_values
                    .entry(param.name.clone())
                    .or_insert((r.lo + r.hi) / 2.0);
            }
        }
    }
}

fn update(model: &mut Model, message: Msg) -> Task<Msg> {
    match message {
        Msg::EditorAction(action) => {
            let is_edit = action.is_edit();
            model.editor.perform(action);
            if is_edit {
                model.unsaved = true;
                return spawn_compile_job(model);
            }
            Task::none()
        }
        Msg::UpdatePreviews => spawn_compile_job(model),
        Msg::SliderChanged(pid, name, v) => {
            if let Some(p) = model.previews.iter_mut().find(|p| p.id == pid) {
                p.slider_values.insert(name, v as f64);
                model.unsaved = true;
                return spawn_eval_for(model, pid);
            }
            Task::none()
        }
        Msg::CompileDone(result) => match result {
            CompileJobResult::Success {
                program,
                signature,
                diagnostics,
            } => {
                model.signature = signature;
                model.program = Some(program);
                model.error_message.clear();
                model.error_span = None;
                model.diagnostics = diagnostics;
                apply_signature_defaults(model);
                spawn_eval_all(model)
            }
            CompileJobResult::Error {
                message,
                span,
                diagnostics,
            } => {
                model.error_message = message;
                model.error_span = span;
                model.diagnostics = diagnostics;
                Task::none()
            }
        },
        Msg::EvalDone(pid, result) => {
            if let Some(p) = model.previews.iter_mut().find(|p| p.id == pid) {
                p.apply_eval_result(result);
            }
            Task::none()
        }
        Msg::Preview(pid, pm) => match pm {
            PreviewMsg::Close => {
                model.previews.retain(|p| p.id != pid);
                model.unsaved = true;
                Task::none()
            }
            PreviewMsg::MoveUp => {
                if let Some(i) = model.previews.iter().position(|p| p.id == pid) {
                    if i > 0 {
                        model.previews.swap(i - 1, i);
                        model.unsaved = true;
                    }
                }
                Task::none()
            }
            PreviewMsg::MoveDown => {
                if let Some(i) = model.previews.iter().position(|p| p.id == pid) {
                    if i + 1 < model.previews.len() {
                        model.previews.swap(i, i + 1);
                        model.unsaved = true;
                    }
                }
                Task::none()
            }
            PreviewMsg::Minimize => {
                if let Some(p) = model.previews.iter_mut().find(|p| p.id == pid) {
                    p.minimized = !p.minimized;
                    model.unsaved = true;
                }
                Task::none()
            }
            PreviewMsg::SceneIgnored => Task::none(),
            PreviewMsg::ToggleViewCenter => {
                if let Some(p) = model.previews.iter_mut().find(|p| p.id == pid) {
                    p.view_at_object_center = !p.view_at_object_center;
                    p.scene.view_at_object_center = p.view_at_object_center;
                    model.unsaved = true;
                }
                Task::none()
            }
            PreviewMsg::SelectControlPoint(i) => {
                if let Some(p) = model.previews.iter_mut().find(|p| p.id == pid) {
                    p.selected_cp = Some(i);
                }
                spawn_eval_for(model, pid)
            }
            PreviewMsg::ControlPointEdited(name, axis, value_str) => {
                let v = match value_str.parse::<f64>() {
                    Ok(v) => v,
                    Err(_) => return Task::none(),
                };
                if let Some(p) = model.previews.iter_mut().find(|p| p.id == pid) {
                    // 既存値がなければ control_points から default を引いて初期化
                    let entry = p.control_overrides.entry(name.clone()).or_insert_with(|| {
                        p.control_points
                            .iter()
                            .find(|(n, _)| n == &name)
                            .map(|(_, pos)| *pos)
                            .unwrap_or([0.0, 0.0, 0.0])
                    });
                    if axis < 3 {
                        entry[axis] = v;
                    }
                    model.unsaved = true;
                    return spawn_eval_for(model, pid);
                }
                Task::none()
            }
        },

        Msg::AddPreview => {
            let id = model.next_preview_id;
            model.next_preview_id += 1;
            let mut p = Preview::new(id);
            for param in &model.signature.params {
                if let Some(r) = &param.range {
                    p.slider_values
                        .insert(param.name.clone(), (r.lo + r.hi) / 2.0);
                }
            }
            model.previews.push(p);
            model.unsaved = true;
            spawn_eval_for(model, id)
        }
        Msg::AddCollisionCheck => {
            let id = model.next_preview_id;
            model.next_preview_id += 1;
            let mut p = Preview::new_collision(id);
            for param in &model.signature.params {
                if let Some(r) = &param.range {
                    p.slider_values
                        .insert(param.name.clone(), (r.lo + r.hi) / 2.0);
                }
            }
            model.previews.push(p);
            model.unsaved = true;
            spawn_eval_for(model, id)
        }

        Msg::NewSession => {
            model.editor = text_editor::Content::with_text(DEFAULT_EDITOR_TEXT);
            model.current_file_path = None;
            model.error_message.clear();
            model.error_span = None;
            model.last_modified = None;
            model.unsaved = false;
            model.previews = vec![Preview::new(0)];
            model.next_preview_id = 1;
            spawn_compile_job(model)
        }
        Msg::OpenSession => model.dialogs.open_session(),
        Msg::SessionOpened(result) => {
            if let Some((path, db_content, previews)) = result {
                model.editor = text_editor::Content::with_text(&db_content);
                model.current_file_path = Some(path.clone());
                model.error_message.clear();
                model.error_span = None;
                model.last_modified = None;
                model.unsaved = false;
                if previews.previews.is_empty() {
                    model.previews = vec![Preview::new(0)];
                    model.next_preview_id = 1;
                } else {
                    model.previews = previews
                        .previews
                        .iter()
                        .map(Preview::from_session)
                        .collect();
                    model.next_preview_id =
                        model.previews.iter().map(|p| p.id).max().unwrap_or(0) + 1;
                }
                session::save_last_session_path(&path);
                return spawn_compile_job(model);
            }
            Task::none()
        }
        Msg::SaveSession => {
            if let Some(ref path) = model.current_file_path {
                let path = path.clone();
                let text = model.editor.text();
                let previews = collect_session_previews(model);
                Task::perform(
                    async move { session::save_session(&path, &text, &previews).map(|()| path) },
                    Msg::SessionSaved,
                )
            } else {
                update(model, Msg::SaveSessionAs)
            }
        }
        Msg::SaveSessionAs => {
            let text = model.editor.text();
            let previews = collect_session_previews(model);
            model.dialogs.save_session_as(text, previews)
        }
        Msg::SessionSaved(result) => {
            match result {
                Ok(path) => {
                    session::save_last_session_path(&path);
                    model.current_file_path = Some(path);
                    model.unsaved = false;
                }
                Err(e) if e != "Cancelled" => {
                    model.error_message = format!("Save failed: {e}");
                }
                _ => {}
            }
            Task::none()
        }

        Msg::ToggleAutoReload => {
            model.auto_reload = !model.auto_reload;
            if model.auto_reload {
                model.last_modified = model.current_file_path.as_ref().and_then(|p| {
                    std::fs::metadata(p.join("db.cadhr"))
                        .and_then(|m| m.modified())
                        .ok()
                });
            }
            Task::none()
        }
        Msg::PaneResized(event) => {
            model.panes.resize(event.split, event.ratio);
            Task::none()
        }
        Msg::ToggleEditor => {
            if model.panes.maximized().is_some() {
                model.panes.restore();
            } else {
                // エディタを隠したいので Preview ペインを最大化する
                let preview_pane = model
                    .panes
                    .iter()
                    .find(|(_, k)| **k == PaneKind::Preview)
                    .map(|(p, _)| *p);
                if let Some(p) = preview_pane {
                    model.panes.maximize(p);
                }
            }
            Task::none()
        }
        Msg::CheckFileChanged => {
            if !model.auto_reload {
                return Task::none();
            }
            if let Some(path) = &model.current_file_path {
                let db_path = path.join("db.cadhr");
                if let Ok(meta) = std::fs::metadata(&db_path) {
                    if let Ok(modified) = meta.modified() {
                        if model.last_modified.is_none_or(|prev| modified > prev) {
                            model.last_modified = Some(modified);
                            if let Ok(content) = std::fs::read_to_string(&db_path) {
                                model.editor = text_editor::Content::with_text(&content);
                                return spawn_compile_job(model);
                            }
                        }
                    }
                }
            }
            Task::none()
        }
    }
}

fn subscription(model: &Model) -> Subscription<Msg> {
    if model.auto_reload {
        iced::time::every(std::time::Duration::from_secs(1)).map(|_| Msg::CheckFileChanged)
    } else {
        Subscription::none()
    }
}

fn view(model: &Model) -> Element<'_, Msg> {
    let title = model
        .current_file_path
        .as_ref()
        .and_then(|p| p.file_name())
        .and_then(|n| n.to_str())
        .unwrap_or("untitled");
    let dirty = if model.unsaved { " *" } else { "" };

    let toolbar = row![
        parts::dark_button("New").on_press(Msg::NewSession),
        parts::dark_button("Open").on_press(Msg::OpenSession),
        parts::dark_button("Save").on_press(Msg::SaveSession),
        parts::dark_button("Save As").on_press(Msg::SaveSessionAs),
        pick_list(&AddKind::ALL[..], None::<AddKind>, |kind| match kind {
            AddKind::Preview => Msg::AddPreview,
            AddKind::Collision => Msg::AddCollisionCheck,
        })
        .placeholder("+ Add Workspace"),
        parts::dark_button("Update").on_press(Msg::UpdatePreviews),
        parts::dark_button(if model.panes.maximized().is_some() {
            "Show Editor"
        } else {
            "Hide Editor"
        })
        .on_press(Msg::ToggleEditor),
        toggler(model.auto_reload)
            .label("Auto Reload")
            .on_toggle(|_| Msg::ToggleAutoReload),
        text(format!("  {title}{dirty}")),
    ]
    .spacing(4)
    .padding(4);

    let panes_view = pane_grid::PaneGrid::new(&model.panes, |_pane, kind, _is_maximized| {
        match kind {
            PaneKind::Editor => {
                let hl_settings = highlight::Settings {
                    error_span: model.error_span.map(|s| (s.start, s.end)),
                    has_error: !model.error_message.is_empty(),
                };
                let editor = text_editor(&model.editor)
                    .on_action(Msg::EditorAction)
                    .key_binding(parts::emacs_key_binding)
                    .highlight_with::<highlight::SpanHighlighter>(hl_settings, highlight::format)
                    .height(Fill);
                pane_grid::Content::new(editor)
            }
            PaneKind::Preview => {
                // 最初の preview のスライダーを「主スライダー」として main パネルに出す。
                // 他の preview は同じ値を共有 (per-preview override は将来課題)。
                let sliders_view: Element<'_, Msg> =
                    if model.signature.params.is_empty() || model.previews.is_empty() {
                        column![].into()
                    } else {
                        let main_pid = model.previews[0].id;
                        let mut col = column![text("Main sliders:").size(13)]
                            .spacing(4)
                            .padding(4);
                        for p in &model.signature.params {
                            let r = match &p.range {
                                Some(r) => r,
                                None => continue,
                            };
                            let cur = model.previews[0]
                                .slider_values
                                .get(&p.name)
                                .copied()
                                .unwrap_or((r.lo + r.hi) / 2.0)
                                as f32;
                            let lo = r.lo as f32;
                            let hi = r.hi as f32;
                            let name = p.name.clone();
                            let label = text(format!("{}: {:.2}", p.name, cur));
                            let widget = slider(lo..=hi, cur, move |v| {
                                Msg::SliderChanged(main_pid, name.clone(), v)
                            });
                            col = col.push(row![label, widget].spacing(8));
                        }
                        col.into()
                    };

                let previews_view: Element<'_, Msg> =
                    ui::preview::list_view(&model.previews).map(|(id, pm)| Msg::Preview(id, pm));

                pane_grid::Content::new(column![sliders_view, previews_view].spacing(4))
            }
        }
    })
    .spacing(4)
    .on_resize(8, Msg::PaneResized);

    let error_bar: Element<'_, Msg> = if model.error_message.is_empty() {
        if model.diagnostics.is_empty() {
            column![].into()
        } else {
            let items: Vec<Element<'_, Msg>> = model
                .diagnostics
                .iter()
                .map(|d| text(format!("warning: {d}")).into())
                .collect();
            scrollable(column(items).spacing(2))
                .height(Length::Fixed(80.0))
                .into()
        }
    } else {
        text(format!("error: {}", model.error_message))
            .color(iced::Color::from_rgb(1.0, 0.3, 0.3))
            .into()
    };

    container(
        column![toolbar, panes_view, error_bar]
            .spacing(4)
            .padding(4),
    )
    .into()
}
