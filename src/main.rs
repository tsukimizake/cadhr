mod debug;
mod export;
mod highlight;
mod interpreter;
mod preview;
mod session;
mod ui;

use std::path::PathBuf;

use cadhr_lang::{BindingSignature, CompiledProgram, Span};
use iced::widget::{
    column, container, pane_grid, pick_list, row, scrollable, text, text_editor, toggler,
};
use iced::{Element, Fill, Length, Subscription, Task};
use interpreter::{CollisionJobParams, CompileJobParams, CompileJobResult, EvalJobResult};
use ui::parts;
use ui::preview::Preview;
use ui::sketch::Sketch;
use ui::workspace::{Workspace, WorkspaceEvent, WorkspaceMsg};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PaneKind {
    Editor,
    Preview,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AddKind {
    Preview,
    Collision,
    Sketch,
}

impl AddKind {
    const ALL: [AddKind; 3] = [AddKind::Preview, AddKind::Collision, AddKind::Sketch];
}

impl std::fmt::Display for AddKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AddKind::Preview => write!(f, "Preview"),
            AddKind::Collision => write!(f, "Collision Check"),
            AddKind::Sketch => write!(f, "2D Sketch"),
        }
    }
}

const DEFAULT_EDITOR_TEXT: &str =
    "main length =\n    cube 10.0 10.0 length\n\nslider main.length = 6.0 .. 80.0\n";

trait DialogHandler {
    fn open_session(&self) -> Task<Msg>;
    fn save_session_as(&self, editor_text: String, previews: session::SessionPreviews)
    -> Task<Msg>;
    fn export_3mf(&self, suggested_name: String, data: Vec<u8>) -> Task<Msg>;
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

    fn save_session_as(&self, text: String, previews: session::SessionPreviews) -> Task<Msg> {
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

    fn export_3mf(&self, suggested_name: String, data: Vec<u8>) -> Task<Msg> {
        Task::perform(
            async move {
                let handle = rfd::AsyncFileDialog::new()
                    .set_title("Export 3MF")
                    .add_filter("3MF", &["3mf"])
                    .set_file_name(&suggested_name)
                    .save_file()
                    .await;
                match handle {
                    Some(h) => std::fs::write(h.path(), data)
                        .map_err(|e| format!("Failed to write 3MF: {e}")),
                    None => Ok(()),
                }
            },
            Msg::ExportFinished,
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
    workspaces: Vec<Workspace>,
    next_workspace_id: u64,
    program: Option<CompiledProgram>,
    /// `previewable_bindings()` のキャッシュ。compile 完了時に更新。
    candidates: Vec<BindingSignature>,
    /// combo_box の選択肢用キャッシュ (= `candidates.iter().map(|b| b.name).collect()`)。
    candidate_names: Vec<String>,
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
    CompileDone(CompileJobResult),
    EvalDone(u64, EvalJobResult),
    Workspace(u64, WorkspaceMsg),

    AddPreview,
    AddCollisionCheck,
    AddSketch,

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

    ExportFinished(Result<(), String>),
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
        workspaces: vec![Workspace::Preview(Preview::new(0))],
        next_workspace_id: 1,
        program: None,
        candidates: Vec::new(),
        candidate_names: Vec::new(),
        error_message: String::new(),
        error_span: None,
        diagnostics: Vec::new(),
        current_file_path: None,
        auto_reload: true,
        last_modified: None,
        unsaved: false,
        dialogs: Box::new(RfdDialogs),
        panes,
    };

    if let Some(path) = session::restore_last_session_path() {
        if let Some((db, previews)) = session::load_session(&path) {
            model.editor = text_editor::Content::with_text(&db);
            model.current_file_path = Some(path);
            let workspaces = workspaces_from_session(&previews);
            if !workspaces.is_empty() {
                model.next_workspace_id =
                    workspaces.iter().map(Workspace::id).max().unwrap_or(0) + 1;
                model.workspaces = workspaces;
            }
        }
    }
    let task = spawn_compile_job(&model);
    (model, task)
}

/// session の previews / sketches を `order` でマージして workspace リストに戻す。
fn workspaces_from_session(sp: &session::SessionPreviews) -> Vec<Workspace> {
    let mut workspaces: Vec<(usize, Workspace)> = sp
        .previews
        .iter()
        .map(|p| (p.order, Workspace::Preview(Preview::from_session(p))))
        .chain(
            sp.sketches
                .iter()
                .map(|s| (s.order, Workspace::Sketch(Sketch::from_session(s)))),
        )
        .collect();
    workspaces.sort_by_key(|(order, _)| *order);
    workspaces.into_iter().map(|(_, w)| w).collect()
}

fn preview_mut(model: &mut Model, id: u64) -> Option<&mut Preview> {
    model
        .workspaces
        .iter_mut()
        .find(|w| w.id() == id)?
        .as_preview_mut()
}

fn move_workspace(model: &mut Model, id: u64, up: bool) {
    let Some(i) = model.workspaces.iter().position(|w| w.id() == id) else {
        return;
    };
    let j = if up {
        i.checked_sub(1)
    } else {
        (i + 1 < model.workspaces.len()).then_some(i + 1)
    };
    if let Some(j) = j {
        model.workspaces.swap(i, j);
        model.unsaved = true;
    }
}

fn base_name(model: &Model) -> String {
    model
        .current_file_path
        .as_ref()
        .and_then(|p| p.file_name())
        .and_then(|n| n.to_str())
        .unwrap_or("untitled")
        .to_string()
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
    let Some(p) = model
        .workspaces
        .iter()
        .find(|w| w.id() == preview_id)
        .and_then(Workspace::as_preview)
    else {
        return Task::none();
    };
    if p.is_collision {
        let params = CollisionJobParams {
            program: prog,
            slider_values: p.slider_values.clone(),
            search_paths: search_paths(model),
        };
        Task::perform(
            async move {
                std::thread::spawn(move || interpreter::run_collision_job(params))
                    .join()
                    .expect("eval worker panicked")
            },
            move |r| Msg::EvalDone(preview_id, r),
        )
    } else {
        let params = p.build_eval_params(&prog, search_paths(model));
        Task::perform(
            async move {
                std::thread::spawn(move || interpreter::run_eval_job(params))
                    .join()
                    .expect("eval worker panicked")
            },
            move |r| Msg::EvalDone(preview_id, r),
        )
    }
}

fn spawn_eval_all(model: &Model) -> Task<Msg> {
    let tasks: Vec<Task<Msg>> = model
        .workspaces
        .iter()
        .filter_map(Workspace::as_preview)
        .map(|p| spawn_eval_for(model, p.id))
        .collect();
    Task::batch(tasks)
}

fn collect_session(model: &Model) -> session::SessionPreviews {
    let mut previews = Vec::new();
    let mut sketches = Vec::new();
    for (i, w) in model.workspaces.iter().enumerate() {
        match w {
            Workspace::Preview(p) => previews.push(p.to_session(i)),
            Workspace::Sketch(s) => sketches.push(s.to_session(i)),
        }
    }
    session::SessionPreviews { previews, sketches }
}

/// 各 preview について現在の target binding の signature を引き、未登録 param
/// に slider 中央値 (range 無しなら 0.0) を populate する。
fn apply_signature_defaults(model: &mut Model) {
    let Some(prog) = &model.program else { return };
    for p in model
        .workspaces
        .iter_mut()
        .filter_map(Workspace::as_preview_mut)
    {
        let target = if p.is_collision { "main" } else { &p.target };
        let Some(sig) = prog.binding_signature(target) else {
            continue;
        };
        for param in &sig.params {
            let default = param
                .range
                .as_ref()
                .map(|r| (r.lo + r.hi) / 2.0)
                .unwrap_or(0.0);
            p.slider_values.entry(param.name.clone()).or_insert(default);
        }
    }
}

/// candidate 一覧が変わったとき、各 preview の combo_box state を更新する。
fn refresh_preview_candidates(model: &mut Model) {
    for p in model
        .workspaces
        .iter_mut()
        .filter_map(Workspace::as_preview_mut)
    {
        p.refresh_candidates(&model.candidate_names);
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
        Msg::CompileDone(result) => match result {
            CompileJobResult::Success {
                program,
                diagnostics,
            } => {
                model.candidates = program.previewable_bindings();
                model.candidate_names = model.candidates.iter().map(|b| b.name.clone()).collect();
                model.program = Some(program);
                model.error_message.clear();
                model.error_span = None;
                model.diagnostics = diagnostics;
                refresh_preview_candidates(model);
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
            if let Some(p) = preview_mut(model, pid) {
                p.apply_eval_result(result);
            }
            Task::none()
        }
        Msg::Workspace(id, wm) => {
            let Some(w) = model.workspaces.iter_mut().find(|w| w.id() == id) else {
                return Task::none();
            };
            match w.update(wm) {
                WorkspaceEvent::None => Task::none(),
                WorkspaceEvent::Edited => {
                    model.unsaved = true;
                    Task::none()
                }
                WorkspaceEvent::EvalNeeded { edited } => {
                    if edited {
                        model.unsaved = true;
                    }
                    spawn_eval_for(model, id)
                }
                WorkspaceEvent::TargetChanged => {
                    model.unsaved = true;
                    apply_signature_defaults(model);
                    spawn_eval_for(model, id)
                }
                WorkspaceEvent::ExportRequested(None) => {
                    model.error_message = "Nothing to export".to_string();
                    Task::none()
                }
                WorkspaceEvent::ExportRequested(Some(data)) => {
                    let suggested = format!("{}_{id}.3mf", base_name(model));
                    model.dialogs.export_3mf(suggested, data)
                }
                WorkspaceEvent::CopyRequested(code) => iced::clipboard::write(code),
                WorkspaceEvent::Close => {
                    model.workspaces.retain(|w| w.id() != id);
                    model.unsaved = true;
                    Task::none()
                }
                WorkspaceEvent::MoveUp => {
                    move_workspace(model, id, true);
                    Task::none()
                }
                WorkspaceEvent::MoveDown => {
                    move_workspace(model, id, false);
                    Task::none()
                }
            }
        }

        Msg::AddPreview => {
            let id = model.next_workspace_id;
            model.next_workspace_id += 1;
            let mut p = Preview::new(id);
            p.refresh_candidates(&model.candidate_names);
            model.workspaces.push(Workspace::Preview(p));
            apply_signature_defaults(model);
            model.unsaved = true;
            spawn_eval_for(model, id)
        }
        Msg::AddCollisionCheck => {
            let id = model.next_workspace_id;
            model.next_workspace_id += 1;
            let mut p = Preview::new_collision(id);
            p.refresh_candidates(&model.candidate_names);
            model.workspaces.push(Workspace::Preview(p));
            apply_signature_defaults(model);
            model.unsaved = true;
            spawn_eval_for(model, id)
        }
        Msg::AddSketch => {
            let id = model.next_workspace_id;
            model.next_workspace_id += 1;
            model.workspaces.push(Workspace::Sketch(Sketch::new(id)));
            model.unsaved = true;
            Task::none()
        }

        Msg::NewSession => {
            model.editor = text_editor::Content::with_text(DEFAULT_EDITOR_TEXT);
            model.current_file_path = None;
            model.error_message.clear();
            model.error_span = None;
            model.last_modified = None;
            model.unsaved = false;
            model.workspaces = vec![Workspace::Preview(Preview::new(0))];
            model.next_workspace_id = 1;
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
                let workspaces = workspaces_from_session(&previews);
                if workspaces.is_empty() {
                    model.workspaces = vec![Workspace::Preview(Preview::new(0))];
                    model.next_workspace_id = 1;
                } else {
                    model.next_workspace_id =
                        workspaces.iter().map(Workspace::id).max().unwrap_or(0) + 1;
                    model.workspaces = workspaces;
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
                let previews = collect_session(model);
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
            let previews = collect_session(model);
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
        Msg::ExportFinished(result) => {
            if let Err(e) = result {
                model.error_message = e;
            }
            Task::none()
        }
        Msg::ToggleEditor => {
            if model.panes.maximized().is_some() {
                model.panes.restore();
            } else {
                // エディタを隠してPreviewペインを最大化する
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
            AddKind::Sketch => Msg::AddSketch,
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
                // 各 preview が自分の binding signature に合わせた引数 slider を持つ。
                let workspaces_view: Element<'_, Msg> =
                    ui::workspace::list_view(&model.workspaces, &model.candidates)
                        .map(|(id, wm)| Msg::Workspace(id, wm));

                pane_grid::Content::new(workspaces_view)
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
