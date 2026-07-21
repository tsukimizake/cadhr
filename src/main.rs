mod debug;
mod export;
mod highlight;
mod history;
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
use ui::sketch::{Sketch, SketchEdit};
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
    /// エディタ本文の undo / redo 履歴 (キー入力・Sketch ドラッグ・構造編集)。
    history: history::History,
    workspaces: Vec<Workspace>,
    next_workspace_id: u64,
    program: Option<CompiledProgram>,
    /// `previewable_bindings()` のキャッシュ。compile 完了時に更新。
    candidates: Vec<BindingSignature>,
    /// combo_box の選択肢用キャッシュ (= `candidates.iter().map(|b| b.name).collect()`)。
    candidate_names: Vec<String>,
    /// Sketch の紐付け候補 (body が sketch..end の binding 名)。compile 完了時に更新。
    sketch_binding_names: Vec<String>,
    error_message: String,
    error_span: Option<Span>,
    diagnostics: Vec<String>,
    current_file_path: Option<PathBuf>,
    auto_reload: bool,
    last_modified: Option<std::time::SystemTime>,
    unsaved: bool,
    /// compile job の coalescing。in_flight 中の編集は dirty にして完了後 1 回だけ再実行。
    compile_in_flight: bool,
    compile_dirty: bool,
    dialogs: Box<dyn DialogHandler>,
    panes: pane_grid::State<PaneKind>,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
enum Msg {
    EditorAction(text_editor::Action),
    Undo,
    Redo,
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
        history: history::History::default(),
        workspaces: vec![Workspace::Preview(Preview::new(0))],
        next_workspace_id: 1,
        program: None,
        candidates: Vec::new(),
        candidate_names: Vec::new(),
        sketch_binding_names: Vec::new(),
        error_message: String::new(),
        error_span: None,
        diagnostics: Vec::new(),
        current_file_path: None,
        auto_reload: true,
        last_modified: None,
        unsaved: false,
        compile_in_flight: false,
        compile_dirty: false,
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
    let task = request_compile(&mut model);
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

fn sketch_mut(model: &mut Model, id: u64) -> Option<&mut Sketch> {
    match model.workspaces.iter_mut().find(|w| w.id() == id)? {
        Workspace::Sketch(s) => Some(s),
        _ => None,
    }
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

/// compile 要求の唯一の入口。実行中なら dirty を立てて完了後に 1 回だけ再実行する
/// (Sketch のドラッグ中は編集が高頻度に来るため)。
fn request_compile(model: &mut Model) -> Task<Msg> {
    if model.compile_in_flight {
        model.compile_dirty = true;
        return Task::none();
    }
    model.compile_in_flight = true;
    model.compile_dirty = false;
    spawn_compile_job(model)
}

fn spawn_eval_for(model: &Model, workspace_id: u64) -> Task<Msg> {
    let Some(prog) = model.program.clone() else {
        return Task::none();
    };
    match model.workspaces.iter().find(|w| w.id() == workspace_id) {
        Some(Workspace::Preview(p)) if p.is_collision => {
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
                move |r| Msg::EvalDone(workspace_id, r),
            )
        }
        Some(Workspace::Preview(p)) => {
            let params = p.build_eval_params(&prog, search_paths(model));
            Task::perform(
                async move {
                    std::thread::spawn(move || interpreter::run_eval_job(params))
                        .join()
                        .expect("eval worker panicked")
                },
                move |r| Msg::EvalDone(workspace_id, r),
            )
        }
        // Sketch の model は CompileDone 時に同期計算する (eval job 不要)
        Some(Workspace::Sketch(_)) => Task::none(),
        None => Task::none(),
    }
}

fn spawn_eval_all(model: &Model) -> Task<Msg> {
    let tasks: Vec<Task<Msg>> = model
        .workspaces
        .iter()
        .map(|w| spawn_eval_for(model, w.id()))
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

/// candidate 一覧が変わったとき、各 workspace の combo_box state を更新する。
fn refresh_workspace_candidates(model: &mut Model) {
    for w in &mut model.workspaces {
        match w {
            Workspace::Preview(p) => p.refresh_candidates(&model.candidate_names),
            Workspace::Sketch(s) => s.refresh_candidates(&model.sketch_binding_names),
        }
    }
}

/// Sketch workspace の model を現在のエディタ本文から再計算する。
fn refresh_sketch_model(model: &mut Model, id: u64) {
    let src = model.editor.text();
    let Some(s2) = sketch_mut(model, id) else {
        return;
    };
    if s2.binding.is_empty() {
        s2.set_model(None);
        return;
    }
    match cadhr_lang::sketch::model_from_source(&src, &s2.binding) {
        Ok(m) => s2.set_model(Some(m)),
        Err(e) => {
            s2.set_model(None);
            s2.set_status(e);
        }
    }
}

/// undo 履歴に積むための現在のエディタ状態。
fn editor_snapshot(model: &Model) -> history::Snapshot {
    history::Snapshot {
        text: model.editor.text(),
        cursor: model.editor.cursor(),
    }
}

/// undo / redo で取り出したスナップショットをエディタへ反映する。
fn apply_snapshot(model: &mut Model, snap: history::Snapshot) -> Task<Msg> {
    model.editor = text_editor::Content::with_text(&snap.text);
    model.editor.move_to(snap.cursor);
    model.unsaved = true;
    for wid in sketch_ids(model) {
        refresh_sketch_model(model, wid);
    }
    request_compile(model)
}

fn sketch_ids(model: &Model) -> Vec<u64> {
    model
        .workspaces
        .iter()
        .filter(|w| matches!(w, Workspace::Sketch(_)))
        .map(Workspace::id)
        .collect()
}

/// Sketch のテキスト編集結果をエディタへ反映して recompile を要求する。
fn apply_sketch_text_edit(
    model: &mut Model,
    id: u64,
    result: Result<String, String>,
) -> Task<Msg> {
    match result {
        Ok(new_src) => {
            model
                .history
                .record(editor_snapshot(model), history::EditKind::Oneshot);
            model.editor = text_editor::Content::with_text(&new_src);
            if let Some(s2) = sketch_mut(model, id) {
                s2.set_status(String::new());
            }
            refresh_sketch_model(model, id);
            model.unsaved = true;
            request_compile(model)
        }
        Err(e) => {
            if let Some(s2) = sketch_mut(model, id) {
                s2.set_status(e);
            }
            Task::none()
        }
    }
}

/// Sketch workspace からのコード書き換え要求を適用する。
fn handle_sketch_edit(model: &mut Model, id: u64, edit: SketchEdit) -> Task<Msg> {
    use cadhr_lang::sketch as sk;
    let Some(binding) = sketch_mut(model, id).map(|s| s.binding.clone()) else {
        return Task::none();
    };
    let src = model.editor.text();
    match edit {
        SketchEdit::Refresh => {
            refresh_sketch_model(model, id);
            model.unsaved = true;
            Task::none()
        }
        SketchEdit::DragEnd => {
            model.history.break_group();
            Task::none()
        }
        _ if binding.is_empty() => {
            if let Some(s2) = sketch_mut(model, id) {
                s2.set_status("sketch binding を選択してください".to_string());
            }
            Task::none()
        }
        SketchEdit::Drag { target, value } => match sk::drag(&src, &binding, target, value) {
            Ok(out) => {
                model
                    .history
                    .record(editor_snapshot(model), history::EditKind::Drag);
                model.editor = text_editor::Content::with_text(&out.source);
                if let Some(s2) = sketch_mut(model, id) {
                    s2.set_model(Some(out.model));
                    s2.set_status(out.pinned.join(" / "));
                }
                model.unsaved = true;
                request_compile(model)
            }
            Err(rej) => {
                if let Some(s2) = sketch_mut(model, id) {
                    s2.set_status(rej.message().to_string());
                }
                Task::none()
            }
        },
        SketchEdit::FactorVars => {
            apply_sketch_text_edit(model, id, sk::factor_vars(&src, &binding))
        }
        SketchEdit::AddPoint { pos } => apply_sketch_text_edit(
            model,
            id,
            sk::add_point(&src, &binding, pos).map(|(s, _)| s),
        ),
        SketchEdit::AddCircle { center, radius } => apply_sketch_text_edit(
            model,
            id,
            sk::add_circle(&src, &binding, center, radius).map(|(s, _)| s),
        ),
        SketchEdit::AddSegment { geom, a, b } => match geom {
            // 追記中の polygon があれば末尾に頂点を足す (a はチェーン継続なので無視)
            Some(g) => apply_sketch_text_edit(
                model,
                id,
                sk::append_polygon_vertex(&src, &binding, &g, b),
            ),
            None => match sk::add_polygon(&src, &binding, &[a, b]) {
                Ok((new_src, name)) => {
                    let task = apply_sketch_text_edit(model, id, Ok(new_src));
                    if let Some(s2) = sketch_mut(model, id) {
                        s2.active_poly = Some(name);
                    }
                    task
                }
                Err(e) => apply_sketch_text_edit(model, id, Err(e)),
            },
        },
        SketchEdit::RemoveGeom { name } => {
            apply_sketch_text_edit(model, id, sk::remove_geom(&src, &binding, &name))
        }
    }
}

fn update(model: &mut Model, message: Msg) -> Task<Msg> {
    match message {
        Msg::EditorAction(action) => {
            let is_edit = action.is_edit();
            if is_edit {
                model
                    .history
                    .record(editor_snapshot(model), history::EditKind::Typing);
            } else {
                // カーソル移動やクリックでタイピングの undo グループを区切る
                model.history.break_group();
            }
            model.editor.perform(action);
            if is_edit {
                model.unsaved = true;
                return request_compile(model);
            }
            Task::none()
        }
        Msg::Undo => {
            let current = editor_snapshot(model);
            match model.history.undo(current) {
                Some(snap) => apply_snapshot(model, snap),
                None => Task::none(),
            }
        }
        Msg::Redo => {
            let current = editor_snapshot(model);
            match model.history.redo(current) {
                Some(snap) => apply_snapshot(model, snap),
                None => Task::none(),
            }
        }
        Msg::UpdatePreviews => request_compile(model),
        Msg::CompileDone(result) => {
            model.compile_in_flight = false;
            // compile 中に編集が入っていたら最新テキストでもう 1 回だけ回す
            let followup = if model.compile_dirty {
                request_compile(model)
            } else {
                Task::none()
            };
            let apply = match result {
                CompileJobResult::Success {
                    program,
                    diagnostics,
                } => {
                    model.candidates = program.previewable_bindings();
                    model.candidate_names =
                        model.candidates.iter().map(|b| b.name.clone()).collect();
                    model.sketch_binding_names = program.sketch_block_bindings();
                    model.program = Some(program);
                    model.error_message.clear();
                    model.error_span = None;
                    model.diagnostics = diagnostics;
                    refresh_workspace_candidates(model);
                    apply_signature_defaults(model);
                    for wid in sketch_ids(model) {
                        refresh_sketch_model(model, wid);
                    }
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
            };
            Task::batch([followup, apply])
        }
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
                    let target = preview_mut(model, id)
                        .map(|p| p.target.clone())
                        .expect("ExportRequested from non-preview workspace");
                    let suggested = format!("{}_{target}.3mf", base_name(model));
                    model.dialogs.export_3mf(suggested, data)
                }
                WorkspaceEvent::SketchEdit(edit) => handle_sketch_edit(model, id, edit),
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
                WorkspaceEvent::ClipboardWrite(snippet) => iced::clipboard::write::<Msg>(snippet),
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
            let mut s = Sketch::new(id);
            s.refresh_candidates(&model.sketch_binding_names);
            model.workspaces.push(Workspace::Sketch(s));
            model.unsaved = true;
            Task::none()
        }

        Msg::NewSession => {
            model.history.clear();
            model.editor = text_editor::Content::with_text(DEFAULT_EDITOR_TEXT);
            model.current_file_path = None;
            model.error_message.clear();
            model.error_span = None;
            model.last_modified = None;
            model.unsaved = false;
            model.workspaces = vec![Workspace::Preview(Preview::new(0))];
            model.next_workspace_id = 1;
            request_compile(model)
        }
        Msg::OpenSession => model.dialogs.open_session(),
        Msg::SessionOpened(result) => {
            if let Some((path, db_content, previews)) = result {
                model.history.clear();
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
                return request_compile(model);
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
                                model
                                    .history
                                    .record(editor_snapshot(model), history::EditKind::Oneshot);
                                model.editor = text_editor::Content::with_text(&content);
                                return request_compile(model);
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
    // エディタ非フォーカス時 (sketch キャンバス操作後など) の undo / redo。
    // フォーカス時は key_binding 側が処理するため、Ignored なイベントのみ届く
    // listen_with で二重発火しない。
    let undo_keys = iced::event::listen_with(|event, status, _window| match (event, status) {
        (
            iced::Event::Keyboard(iced::keyboard::Event::KeyPressed { key, modifiers, .. }),
            iced::event::Status::Ignored,
        ) if modifiers.command() || modifiers.control() => match key.as_ref() {
            iced::keyboard::Key::Character("z") => Some(if modifiers.shift() {
                Msg::Redo
            } else {
                Msg::Undo
            }),
            _ => None,
        },
        _ => None,
    });
    let reload = if model.auto_reload {
        iced::time::every(std::time::Duration::from_secs(1)).map(|_| Msg::CheckFileChanged)
    } else {
        Subscription::none()
    };
    Subscription::batch([undo_keys, reload])
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
        parts::dark_button("Undo").on_press(Msg::Undo),
        parts::dark_button("Redo").on_press(Msg::Redo),
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
                    .key_binding(|kp| parts::emacs_key_binding(kp, Msg::Undo, Msg::Redo))
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
