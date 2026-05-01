mod debug;
mod export;
mod highlight;
mod interpreter;
mod preview;
mod session;
mod ui;

use cadhr_lang::parse::SrcSpan;
use iced::widget::{column, row, scrollable, text, text_editor, toggler};
use iced::{Element, Fill, Subscription, Task};
use std::path::PathBuf;

// rfd ダイアログ呼び出しはiced管理下ではないのでSimulateできない。
// テストでは固定パスを返す実装に差し替える。
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
    preview_model: ui::preview::PreviewModel,
    current_file_path: Option<PathBuf>,
    error_message: String,
    error_span: Option<SrcSpan>,
    unsaved: bool,
    auto_reload: bool,
    last_modified: Option<std::time::SystemTime>,
    dialogs: Box<dyn DialogHandler>,
}

#[derive(Debug, Clone)]
enum Msg {
    EditorAction(text_editor::Action),

    // Preview (delegated)
    Preview(ui::preview::Msg),

    // File I/O
    NewSession,
    OpenSession,
    SaveSession,
    SaveSessionAs,
    SessionOpened(Option<(PathBuf, String, session::SessionPreviews)>),
    SessionSaved(Result<PathBuf, String>),

    // Auto reload
    ToggleAutoReload,
    CheckFileChanged,
}

fn init() -> (Model, Task<Msg>) {
    if let Some(path) = session::restore_last_session_path() {
        if let Some((db_content, previews)) = session::load_session(&path) {
            let mut model = Model {
                editor: text_editor::Content::with_text(&db_content),
                preview_model: ui::preview::PreviewModel::new(),
                current_file_path: Some(path),
                error_message: String::new(),
                error_span: None,
                unsaved: false,
                auto_reload: false,
                last_modified: None,
                dialogs: Box::new(RfdDialogs),
            };
            let mut tasks = vec![];
            for sp in &previews.previews {
                let id = model.preview_model.add_from_session(sp);
                tasks.push(ui::preview::generate(
                    &model.preview_model,
                    id,
                    make_ctx(&model),
                ));
            }
            return (model, Task::batch(tasks).map(Msg::Preview));
        }
    }

    (
        Model {
            editor: text_editor::Content::with_text("main :- cube(10, 20, 30)."),
            preview_model: ui::preview::PreviewModel::new(),
            current_file_path: None,
            error_message: String::new(),
            error_span: None,
            unsaved: false,
            auto_reload: false,
            last_modified: None,
            dialogs: Box::new(RfdDialogs),
        },
        Task::none(),
    )
}

fn make_ctx(model: &Model) -> ui::preview::Context {
    ui::preview::Context {
        editor_text: model.editor.text(),
        include_paths: model.current_file_path.iter().cloned().collect(),
        base_name: model
            .current_file_path
            .as_ref()
            .and_then(|p| p.file_stem())
            .and_then(|n| n.to_str())
            .unwrap_or("untitled")
            .to_string(),
    }
}

fn update(model: &mut Model, message: Msg) -> Task<Msg> {
    match message {
        Msg::EditorAction(action) => {
            let is_edit = action.is_edit();
            model.editor.perform(action);
            if is_edit {
                model.unsaved = true;
            }
            Task::none()
        }
        Msg::Preview(msg) => {
            let ctx = make_ctx(model);
            let (task, outcome) = ui::preview::update(&mut model.preview_model, msg, ctx);
            if outcome.mark_unsaved {
                model.unsaved = true;
            }
            if let Some((msg_text, span)) = outcome.error {
                model.error_message = msg_text;
                model.error_span = span;
            } else {
                model.error_message.clear();
                model.error_span = None;
            }
            if let Some(new_text) = outcome.source_edit {
                model.editor = text_editor::Content::with_text(&new_text);
                model.unsaved = true;
            }
            task.map(Msg::Preview)
        }

        // File I/O
        Msg::NewSession => {
            model.editor = text_editor::Content::with_text("main :- cube(10, 20, 30).");
            model.preview_model = ui::preview::PreviewModel::new();
            model.current_file_path = None;
            model.error_message.clear();
            model.error_span = None;
            model.last_modified = None;
            model.unsaved = false;
            Task::none()
        }
        Msg::OpenSession => model.dialogs.open_session(),
        Msg::SessionOpened(result) => {
            if let Some((path, db_content, previews)) = result {
                model.editor = text_editor::Content::with_text(&db_content);
                model.preview_model = ui::preview::PreviewModel::new();
                model.current_file_path = Some(path.clone());
                model.error_message.clear();
                model.error_span = None;
                model.last_modified = None;
                model.unsaved = false;
                session::save_last_session_path(&path);

                let mut tasks = vec![];
                for sp in previews.previews {
                    let id = model.preview_model.add_from_session(&sp);
                    tasks.push(ui::preview::generate(
                        &model.preview_model,
                        id,
                        make_ctx(&model),
                    ));
                }
                return Task::batch(tasks).map(Msg::Preview);
            }
            Task::none()
        }
        Msg::SaveSession => {
            if let Some(ref path) = model.current_file_path {
                let path = path.clone();
                let text = model.editor.text();
                let previews = ui::preview::collect_session_previews(&model.preview_model);
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
            let previews = ui::preview::collect_session_previews(&model.preview_model);
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
                    model.error_message = format!("Save failed: {}", e);
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
                                return update(
                                    model,
                                    Msg::Preview(ui::preview::Msg::UpdatePreviews),
                                );
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
    let dirty_marker = if model.unsaved { " *" } else { "" };

    let toolbar = row![
        ui::parts::dark_button("New").on_press(Msg::NewSession),
        ui::parts::dark_button("Open").on_press(Msg::OpenSession),
        ui::parts::dark_button("Save").on_press(Msg::SaveSession),
        ui::parts::dark_button("Save As").on_press(Msg::SaveSessionAs),
        text(" | "),
        ui::parts::dark_button("Add Preview").on_press(Msg::Preview(ui::preview::Msg::AddPreview)),
        ui::parts::dark_button("Collision Check")
            .on_press(Msg::Preview(ui::preview::Msg::AddCollisionCheck)),
        ui::parts::dark_button("Update All")
            .on_press(Msg::Preview(ui::preview::Msg::UpdatePreviews)),
        text(" | "),
        toggler(model.auto_reload)
            .label("Auto Reload")
            .on_toggle(|_| Msg::ToggleAutoReload),
        text(format!("  {}{}", title, dirty_marker)),
    ]
    .spacing(4)
    .padding(4);

    let hl_settings = highlight::Settings {
        error_span: model.error_span,
        has_error: !model.error_message.is_empty(),
    };
    let editor = text_editor(&model.editor)
        .on_action(Msg::EditorAction)
        .key_binding(ui::parts::emacs_key_binding)
        .highlight_with::<highlight::SpanHighlighter>(hl_settings, highlight::format)
        .height(Fill);

    let preview_list: Element<'_, Msg> = if model.preview_model.previews.is_empty() {
        text("Add Preview を押してください").into()
    } else {
        let total = model.preview_model.previews.len();
        let items: Vec<Element<'_, Msg>> = model
            .preview_model
            .previews
            .iter()
            .enumerate()
            .map(|(i, p)| ui::preview::view(p, i, total).map(Msg::Preview))
            .collect();
        scrollable(column(items).spacing(12)).height(Fill).into()
    };

    let error_bar: Element<'_, Msg> = if model.error_message.is_empty() {
        column![].into()
    } else {
        text(&model.error_message)
            .color(iced::Color::from_rgb(1.0, 0.3, 0.3))
            .into()
    };

    column![
        toolbar,
        row![editor, preview_list].spacing(4).height(Fill),
        error_bar,
    ]
    .spacing(4)
    .padding(4)
    .into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use iced_test::runtime::{Action, task};
    use iced_test::simulator;

    /// テスト用の DialogHandler。rfd を開かず、事前に与えたパスを直接返す。
    struct TestDialogs {
        open_path: std::sync::Mutex<Option<PathBuf>>,
    }

    impl TestDialogs {
        fn new(open_path: PathBuf) -> Self {
            Self {
                open_path: std::sync::Mutex::new(Some(open_path)),
            }
        }
    }

    impl DialogHandler for TestDialogs {
        fn open_session(&self) -> Task<Msg> {
            let path = self
                .open_path
                .lock()
                .unwrap()
                .take()
                .expect("TestDialogs::open_session called twice without re-arming");
            let (db, previews) = session::load_session(&path).expect("test session should load");
            Task::done(Msg::SessionOpened(Some((path, db, previews))))
        }
        fn save_session_as(
            &self,
            _text: String,
            _previews: Vec<session::SessionPreview>,
        ) -> Task<Msg> {
            panic!("TestDialogs::save_session_as not expected in this test")
        }
    }

    fn fresh_model() -> Model {
        Model {
            editor: text_editor::Content::with_text("main :- cube(10, 20, 30)."),
            preview_model: ui::preview::PreviewModel::new(),
            current_file_path: None,
            error_message: String::new(),
            error_span: None,
            unsaved: false,
            auto_reload: false,
            last_modified: None,
            dialogs: Box::new(RfdDialogs),
        }
    }

    /// `Task` を同期的に消費し、含まれる `Msg` だけを取り出す。
    /// フォント読み込みやウィンドウ操作などの副作用 Action は捨てる。
    fn drain_task(task: Task<Msg>) -> Vec<Msg> {
        use iced::futures::StreamExt;
        let Some(stream) = task::into_stream(task) else {
            return vec![];
        };
        iced::futures::executor::block_on(async move {
            stream
                .filter_map(|action| async move {
                    match action {
                        Action::Output(msg) => Some(msg),
                        _ => None,
                    }
                })
                .collect::<Vec<_>>()
                .await
        })
    }

    /// 任意の query 群を持つセッションを tempdir に作成する。
    fn make_test_session(dir: &std::path::Path, db: &str, queries: &[&str]) {
        let previews: Vec<session::SessionPreview> = queries
            .iter()
            .enumerate()
            .map(|(i, q)| session::SessionPreview {
                preview_id: i as u64,
                query: q.to_string(),
                order: i,
                control_point_overrides: Default::default(),
                query_param_overrides: Default::default(),
                view_at_object_center: false,
                minimized: false,
            })
            .collect();
        session::save_session(dir, db, &previews).unwrap();
    }

    fn simulate_open(model: &mut Model, dir: &std::path::Path) {
        model.dialogs = Box::new(TestDialogs::new(dir.to_path_buf()));
        let mut ui = simulator(view(model));
        ui.click("Open").expect("Open button should be clickable");
        for msg in ui.into_messages() {
            let task = update(model, msg);
            for follow_up in drain_task(task) {
                let _ = update(model, follow_up);
            }
        }
    }

    #[test]
    fn opening_second_session_replaces_previews_from_first() {
        let tmp_a = tempfile::tempdir().unwrap();
        let tmp_b = tempfile::tempdir().unwrap();
        make_test_session(tmp_a.path(), "main :- cube(1, 1, 1).", &["main", "extra"]);
        make_test_session(tmp_b.path(), "main :- sphere(5).", &["main"]);

        let mut model = fresh_model();
        simulate_open(&mut model, tmp_a.path());
        assert_eq!(model.preview_model.previews.len(), 2);

        simulate_open(&mut model, tmp_b.path());

        assert_eq!(model.current_file_path.as_deref(), Some(tmp_b.path()));
        assert_eq!(
            model.preview_model.previews.len(),
            1,
            "session A のプレビューが残っている"
        );
        assert_eq!(model.preview_model.previews[0].query, "main");
    }

    #[test]
    fn scene_view_center_uses_bbox_when_toggled() {
        use glam::Vec3;
        use preview::pipeline::Vertex;

        let v = |p: [f32; 3]| Vertex {
            position: p,
            normal: [0.0, 0.0, 1.0],
            color: [0.0; 4],
        };
        let mut scene = preview::Scene::new();
        scene.set_mesh(vec![v([10.0, 20.0, 30.0]), v([20.0, 40.0, 60.0])], vec![]);

        assert_eq!(scene.view_center(), Vec3::ZERO);
        let origin_dist = scene.base_camera_distance();

        scene.view_at_object_center = true;
        assert_eq!(scene.view_center(), Vec3::new(15.0, 30.0, 45.0));

        // 中心モードでは bbox 半径ベース、原点モードでは原点からの最大距離ベース。
        // 同じメッシュなら中心モードの方が短くなるはず。
        assert!(
            scene.base_camera_distance() < origin_dist,
            "center mode distance {} should be less than origin mode distance {}",
            scene.base_camera_distance(),
            origin_dist
        );
    }

    fn find_preview(model: &Model, id: u64) -> &ui::preview::Preview {
        model
            .preview_model
            .previews
            .iter()
            .find(|p| p.id == id)
            .unwrap()
    }

    fn click_and_drain(model: &mut Model, label: &str) {
        let mut ui = simulator(view(model));
        ui.click(label)
            .unwrap_or_else(|_| panic!("ボタン '{}' が見つからない", label));
        for msg in ui.into_messages() {
            let task = update(model, msg);
            for follow_up in drain_task(task) {
                let _ = update(model, follow_up);
            }
        }
    }

    #[test]
    fn view_center_toggle_button_flips_flag() {
        let mut model = fresh_model();
        let id = model
            .preview_model
            .add_from_session(&session::SessionPreview {
                preview_id: 0,
                query: "main.".to_string(),
                order: 0,
                control_point_overrides: Default::default(),
                query_param_overrides: Default::default(),
                view_at_object_center: false,
                minimized: false,
            });

        assert!(!find_preview(&model, id).view_at_object_center);
        assert!(!find_preview(&model, id).scene.view_at_object_center);

        // 1 回目のクリックで Origin → Center
        click_and_drain(&mut model, "View: Origin");
        assert!(
            find_preview(&model, id).view_at_object_center,
            "1 回目のトグルで true に"
        );
        assert!(
            find_preview(&model, id).scene.view_at_object_center,
            "scene 側 flag も同期しているべき"
        );

        // 2 回目のクリックで Center → Origin。ラベルが反転していることも検証。
        click_and_drain(&mut model, "View: Center");
        assert!(
            !find_preview(&model, id).view_at_object_center,
            "2 回目のトグルで false に"
        );
        assert!(!find_preview(&model, id).scene.view_at_object_center);
    }
}
