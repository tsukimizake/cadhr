//! Sketch workspace: sketch DSL binding (`sketch .. in .. end`) との双方向編集。
//!
//! dropdown で sketch binding を選ぶと [`SketchModel`] がキャンバスに出る。
//! ハンドル (点 / 頂点 / 円中心 / 円周) のドラッグはリアルタイムにコードへ
//! 書き戻される (逆評価は `cadhr_lang::sketch::drag`)。Line / Circle / Point
//! ツールでの描画は binding の挿入としてコードに反映される。
//!
//! エディタ本文は Model が所有するため、テキスト書き換えを要する操作は
//! [`SketchEdit`] イベントとして main.rs に委譲する。

use iced::widget::canvas::{self, Path, Stroke};
use iced::widget::{
    canvas as canvas_widget, column, combo_box, container, pick_list, row, slider, text,
};
use iced::{Color, Element, Fill, Length, Point, Rectangle, Renderer, Size, Theme, mouse};

use cadhr_lang::sketch::{DragTarget, DragValue, SketchGeom, SketchModel};

use crate::session::SessionSketch;
use crate::ui::parts;
use crate::ui::workspace::WorkspaceEvent;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    /// ハンドルのドラッグ (逆評価)。
    Select,
    Line,
    Circle,
    Point,
}

/// Tool メニュー (pick_list) の並び。
const TOOLS: [Tool; 4] = [Tool::Select, Tool::Line, Tool::Circle, Tool::Point];

impl std::fmt::Display for Tool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Tool::Select => "Select",
            Tool::Line => "Line",
            Tool::Circle => "Circle",
            Tool::Point => "Point",
        })
    }
}

/// エディタ本文の書き換えを要する操作。main.rs が `cadhr_lang::sketch` を呼んで
/// 適用し、結果の model をこの workspace に書き戻す。
#[derive(Debug, Clone)]
pub enum SketchEdit {
    /// binding 選択変更などで model の再計算だけが必要。
    Refresh,
    Drag {
        target: DragTarget,
        value: DragValue,
    },
    /// ドラッグ 1 ジェスチャの終了。テキストは変えず undo グループを区切る。
    DragEnd,
    AddPoint {
        pos: [f64; 2],
    },
    AddCircle {
        center: [f64; 2],
        radius: f64,
    },
    /// `geom` が Some なら既存 polygon へ頂点追記、None なら新 polygon を開始。
    AddSegment {
        geom: Option<String>,
        a: [f64; 2],
        b: [f64; 2],
    },
    RemoveGeom {
        name: String,
    },
    /// 2 回以上現れる同値の座標リテラルを軸ごとに var へまとめる。
    FactorVars,
}

pub struct Sketch {
    pub id: u64,
    pub minimized: bool,
    /// 1 grid 単位あたりのピクセル数。
    pub zoom: f32,
    /// キャンバス中央に表示する world 座標。ホイール / トラックパッドでパンする。
    pub center: [f32; 2],
    /// 紐付けている sketch binding の名前。空なら未選択。
    pub binding: String,
    pub binding_state: combo_box::State<String>,
    /// 最新の評価済み model。compile / drag のたびに main.rs が更新する。
    pub model: Option<SketchModel>,
    pub tool: Tool,
    /// Line ツールで追記中の polygon 名 (新しい線は末尾頂点として追加される)。
    pub active_poly: Option<String>,
    /// ドラッグ拒否や編集エラーの表示。
    pub status: String,
    cache: canvas::Cache,
}

impl Sketch {
    pub fn new(id: u64) -> Self {
        Self {
            id,
            minimized: false,
            zoom: 20.0,
            center: [0.0, 0.0],
            binding: String::new(),
            binding_state: combo_box::State::new(Vec::new()),
            model: None,
            tool: Tool::Select,
            active_poly: None,
            status: String::new(),
            cache: canvas::Cache::new(),
        }
    }

    pub fn from_session(ss: &SessionSketch) -> Self {
        let mut s = Self::new(ss.sketch_id);
        s.minimized = ss.minimized;
        s.zoom = ss.zoom;
        s.center = ss.center;
        s.binding = ss.binding.clone();
        s
    }

    pub fn to_session(&self, order: usize) -> SessionSketch {
        SessionSketch {
            sketch_id: self.id,
            order,
            minimized: self.minimized,
            zoom: self.zoom,
            center: self.center,
            binding: self.binding.clone(),
        }
    }

    /// キャンバス描画・座標変換に使う視点。
    fn view(&self) -> View {
        View {
            zoom: self.zoom,
            center: self.center,
        }
    }

    pub fn refresh_candidates(&mut self, names: &[String]) {
        self.binding_state = combo_box::State::new(names.to_vec());
    }

    /// main.rs がテキスト操作の結果を書き戻す。
    pub fn set_model(&mut self, model: Option<SketchModel>) {
        // active_poly が消えた形状を指していたら解除する
        if let Some(active) = &self.active_poly {
            let alive = model
                .as_ref()
                .is_some_and(|m| m.geoms.iter().any(|g| g.name() == active));
            if !alive {
                self.active_poly = None;
            }
        }
        self.model = model;
        self.cache.clear();
    }

    pub fn set_status(&mut self, status: String) {
        self.status = status;
    }

    pub fn update(&mut self, msg: SketchMsg) -> WorkspaceEvent {
        match msg {
            SketchMsg::Close => WorkspaceEvent::Close,
            SketchMsg::MoveUp => WorkspaceEvent::MoveUp,
            SketchMsg::MoveDown => WorkspaceEvent::MoveDown,
            SketchMsg::Minimize => {
                self.minimized = !self.minimized;
                WorkspaceEvent::Edited
            }
            SketchMsg::ZoomChanged(zoom) => {
                self.zoom = zoom;
                self.cache.clear();
                WorkspaceEvent::Edited
            }
            SketchMsg::Panned(delta) => {
                self.center[0] += delta[0];
                self.center[1] += delta[1];
                self.cache.clear();
                WorkspaceEvent::Edited
            }
            SketchMsg::ResetView => {
                self.center = [0.0, 0.0];
                self.cache.clear();
                WorkspaceEvent::Edited
            }
            // ツールはセッションに保存しない UI 状態
            SketchMsg::SetTool(tool) => {
                self.tool = tool;
                WorkspaceEvent::None
            }
            SketchMsg::BindingChanged(name) => {
                if self.binding == name {
                    return WorkspaceEvent::None;
                }
                self.binding = name;
                self.active_poly = None;
                self.status.clear();
                self.cache.clear();
                WorkspaceEvent::SketchEdit(SketchEdit::Refresh)
            }
            SketchMsg::SelectGeom(name) => {
                let is_poly = self.model.as_ref().is_some_and(|m| {
                    m.geoms
                        .iter()
                        .any(|g| g.name() == name && matches!(g, SketchGeom::Polygon { .. }))
                });
                self.active_poly = if is_poly { Some(name) } else { None };
                WorkspaceEvent::None
            }
            SketchMsg::RemoveGeom(name) => {
                WorkspaceEvent::SketchEdit(SketchEdit::RemoveGeom { name })
            }
            SketchMsg::FactorVars => WorkspaceEvent::SketchEdit(SketchEdit::FactorVars),
            SketchMsg::DragStep { target, value } => {
                WorkspaceEvent::SketchEdit(SketchEdit::Drag { target, value })
            }
            SketchMsg::DragEnd => WorkspaceEvent::SketchEdit(SketchEdit::DragEnd),
            SketchMsg::LineAdded(a, b) => WorkspaceEvent::SketchEdit(SketchEdit::AddSegment {
                geom: self.active_poly.clone(),
                a,
                b,
            }),
            SketchMsg::CircleAdded(center, radius) => {
                WorkspaceEvent::SketchEdit(SketchEdit::AddCircle { center, radius })
            }
            SketchMsg::PointAdded(pos) => {
                WorkspaceEvent::SketchEdit(SketchEdit::AddPoint { pos })
            }
        }
    }
}

#[derive(Debug, Clone)]
pub enum SketchMsg {
    Close,
    MoveUp,
    MoveDown,
    Minimize,
    ZoomChanged(f32),
    /// スクロールによるパン (world 座標での中心移動量)。
    Panned([f32; 2]),
    /// パンを原点に戻す。
    ResetView,
    SetTool(Tool),
    BindingChanged(String),
    SelectGeom(String),
    RemoveGeom(String),
    FactorVars,
    /// ハンドルドラッグの 1 ステップ (mousemove ごとに逐次書き戻し)。
    DragStep {
        target: DragTarget,
        value: DragValue,
    },
    DragEnd,
    LineAdded([f64; 2], [f64; 2]),
    CircleAdded([f64; 2], f64),
    PointAdded([f64; 2]),
}

const GRID: Color = Color::from_rgb(0.22, 0.22, 0.25);
const AXIS: Color = Color::from_rgb(0.38, 0.38, 0.44);
const LINE: Color = Color::from_rgb(0.55, 0.85, 0.55);
const HANDLE: Color = Color::from_rgb(0.95, 0.85, 0.4);
const POINT_COLOR: Color = Color::from_rgb(0.75, 0.95, 0.75);
const SEGMENT_COLOR: Color = Color::from_rgb(0.55, 0.75, 0.95);
const ACTIVE_LINE: Color = Color::from_rgb(0.85, 0.95, 0.55);
const DRAG_PREVIEW: Color = Color::from_rgba(0.55, 0.85, 0.55, 0.5);
const SNAP_INDICATOR: Color = Color::from_rgba(0.95, 0.85, 0.4, 0.8);
/// hover / ドラッグ中のハンドルと、それに連動して動くハンドルの強調色。
const LINKED_HANDLE: Color = Color::from_rgb(0.25, 0.95, 0.35);

/// キャンバス描画の視点。`center` の world 座標がキャンバス中央に来る。
#[derive(Clone, Copy)]
struct View {
    zoom: f32,
    center: [f32; 2],
}

/// world 座標 → スクリーン座標。y は上向き。
fn to_screen(view: View, size: Size, p: [f64; 2]) -> Point {
    Point::new(
        size.width / 2.0 + (p[0] as f32 - view.center[0]) * view.zoom,
        size.height / 2.0 - (p[1] as f32 - view.center[1]) * view.zoom,
    )
}

/// 最寄りの格子交点にスナップした world 座標。
fn snap(view: View, size: Size, local: Point) -> [f64; 2] {
    [
        ((local.x - size.width / 2.0) / view.zoom + view.center[0]).round() as f64,
        ((size.height / 2.0 - local.y) / view.zoom + view.center[1]).round() as f64,
    ]
}

fn grid_distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt().round()
}

/// ハンドルの world 座標。CircleRadius は点ではないので None (円周を別途強調する)。
fn target_pos(model: &SketchModel, t: DragTarget) -> Option<[f64; 2]> {
    match t {
        DragTarget::Point { geom } => match model.geoms.get(geom)? {
            SketchGeom::Point { pos, .. } => Some(*pos),
            _ => None,
        },
        DragTarget::SegmentEnd { geom, end } => match model.geoms.get(geom)? {
            SketchGeom::Segment { a, b, .. } => Some(if end == 0 { *a } else { *b }),
            _ => None,
        },
        DragTarget::PolyVertex { geom, vert } => match model.geoms.get(geom)? {
            SketchGeom::Polygon { verts, .. } => verts.get(vert).copied(),
            _ => None,
        },
        DragTarget::CircleCenter { geom } => match model.geoms.get(geom)? {
            SketchGeom::Circle { center, .. } => Some(*center),
            _ => None,
        },
        DragTarget::CircleRadius { .. } => None,
    }
}

/// ハンドルの判定半径 (px)。
const HIT_PX: f32 = 8.0;

struct SketchCanvas<'a> {
    sketch: &'a Sketch,
}

impl SketchCanvas<'_> {
    /// カーソル位置のハンドルを探す。頂点系を円周 (半径ドラッグ) より優先する。
    fn hit_test(&self, size: Size, cursor: Point) -> Option<DragTarget> {
        let model = self.sketch.model.as_ref()?;
        let view = self.sketch.view();
        let near = |p: [f64; 2]| -> bool { to_screen(view, size, p).distance(cursor) <= HIT_PX };
        for (gi, geom) in model.geoms.iter().enumerate() {
            match geom {
                SketchGeom::Point { pos, .. } => {
                    if near(*pos) {
                        return Some(DragTarget::Point { geom: gi });
                    }
                }
                SketchGeom::Segment { a, b, .. } => {
                    if near(*a) {
                        return Some(DragTarget::SegmentEnd { geom: gi, end: 0 });
                    }
                    if near(*b) {
                        return Some(DragTarget::SegmentEnd { geom: gi, end: 1 });
                    }
                }
                SketchGeom::Polygon { verts, .. } => {
                    for (vi, v) in verts.iter().enumerate() {
                        if near(*v) {
                            return Some(DragTarget::PolyVertex { geom: gi, vert: vi });
                        }
                    }
                }
                SketchGeom::Circle { center, .. } => {
                    if near(*center) {
                        return Some(DragTarget::CircleCenter { geom: gi });
                    }
                }
            }
        }
        for (gi, geom) in model.geoms.iter().enumerate() {
            if let SketchGeom::Circle { center, radius, .. } = geom {
                let d = to_screen(view, size, *center).distance(cursor);
                if (d - *radius as f32 * view.zoom).abs() <= HIT_PX {
                    return Some(DragTarget::CircleRadius { geom: gi });
                }
            }
        }
        None
    }

    /// `hot` とリテラル葉を共有していて連動して動くハンドル (hot 自身を含む)。
    fn linked_targets(&self, hot: DragTarget) -> Vec<DragTarget> {
        let mut out = vec![hot];
        let Some(model) = &self.sketch.model else {
            return out;
        };
        let Some(hot_leaves) = model
            .links
            .iter()
            .find(|l| l.target == hot)
            .map(|l| &l.leaves)
        else {
            return out;
        };
        for l in &model.links {
            if l.target != hot && l.leaves.iter().any(|leaf| hot_leaves.contains(leaf)) {
                out.push(l.target);
            }
        }
        out
    }

    /// ドラッグ対象に応じた DragValue を作る。
    fn drag_value(&self, target: DragTarget, world: [f64; 2]) -> Option<DragValue> {
        match target {
            DragTarget::CircleRadius { geom } => {
                let model = self.sketch.model.as_ref()?;
                let SketchGeom::Circle { center, .. } = model.geoms.get(geom)? else {
                    return None;
                };
                let r = grid_distance(*center, world);
                if r <= 0.0 {
                    return None;
                }
                Some(DragValue::Radius(r))
            }
            _ => Some(DragValue::Pos(world)),
        }
    }
}

#[derive(Default)]
enum DragState {
    #[default]
    Idle,
    /// Select ツールでハンドルを掴んでいる。`last` は最後に送った snap 値。
    Handle {
        target: DragTarget,
        last: Option<DragValue>,
    },
    /// Line / Circle ツールのドラッグ描画。
    Draw {
        start: [f64; 2],
        hover: [f64; 2],
    },
    /// カーソル追従のみ (スナップ表示)。
    Hover([f64; 2]),
}

impl canvas::Program<SketchMsg> for SketchCanvas<'_> {
    type State = DragState;

    fn update(
        &self,
        state: &mut DragState,
        event: &canvas::Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<canvas::Action<SketchMsg>> {
        let canvas::Event::Mouse(mouse_event) = event else {
            return None;
        };
        let view = self.sketch.view();
        match mouse_event {
            mouse::Event::ButtonPressed(mouse::Button::Left) => {
                let pos = cursor.position_in(bounds)?;
                match self.sketch.tool {
                    Tool::Select => {
                        let target = self.hit_test(bounds.size(), pos)?;
                        *state = DragState::Handle { target, last: None };
                        Some(canvas::Action::request_redraw().and_capture())
                    }
                    Tool::Point => {
                        let p = snap(view, bounds.size(), pos);
                        Some(canvas::Action::publish(SketchMsg::PointAdded(p)).and_capture())
                    }
                    Tool::Line | Tool::Circle => {
                        let p = snap(view, bounds.size(), pos);
                        *state = DragState::Draw { start: p, hover: p };
                        Some(canvas::Action::request_redraw().and_capture())
                    }
                }
            }
            mouse::Event::WheelScrolled { delta } => {
                // カーソルがキャンバス上にあるときだけパンし、イベントを消費して
                // 外側の workspace リストのスクロールに渡さない
                cursor.position_in(bounds)?;
                let (dx, dy) = match delta {
                    mouse::ScrollDelta::Lines { x, y } => (x * 40.0, y * 40.0),
                    mouse::ScrollDelta::Pixels { x, y } => (*x, *y),
                };
                // scrollable と同じ向き: スクロールでシート (描画内容) が動く
                let world = [-dx / view.zoom, dy / view.zoom];
                Some(canvas::Action::publish(SketchMsg::Panned(world)).and_capture())
            }
            mouse::Event::CursorMoved { .. } => {
                let pos = cursor.position_in(bounds);
                match state {
                    DragState::Handle { target, last } => {
                        let pos = pos?;
                        let world = snap(view, bounds.size(), pos);
                        let value = self.drag_value(*target, world)?;
                        if *last == Some(value) {
                            return None;
                        }
                        let target = *target;
                        *last = Some(value);
                        // 逐次書き戻し: mousemove ごとに逆評価してコードに反映する
                        Some(
                            canvas::Action::publish(SketchMsg::DragStep { target, value })
                                .and_capture(),
                        )
                    }
                    DragState::Draw { hover, .. } => {
                        let pos = pos?;
                        let p = snap(view, bounds.size(), pos);
                        if *hover == p {
                            return None;
                        }
                        *hover = p;
                        Some(canvas::Action::request_redraw())
                    }
                    _ => {
                        match pos {
                            Some(p) => *state = DragState::Hover(snap(view, bounds.size(), p)),
                            None => *state = DragState::Idle,
                        }
                        Some(canvas::Action::request_redraw())
                    }
                }
            }
            mouse::Event::ButtonReleased(mouse::Button::Left) => {
                let prev = std::mem::take(state);
                match prev {
                    DragState::Handle { .. } => {
                        Some(canvas::Action::publish(SketchMsg::DragEnd).and_capture())
                    }
                    DragState::Draw { start, .. } => {
                        // キャンバス外で離した場合はキャンセル
                        let pos = cursor.position_in(bounds)?;
                        let end = snap(view, bounds.size(), pos);
                        match self.sketch.tool {
                            Tool::Line if start != end => Some(
                                canvas::Action::publish(SketchMsg::LineAdded(start, end))
                                    .and_capture(),
                            ),
                            Tool::Circle => {
                                let r = grid_distance(start, end);
                                if r <= 0.0 {
                                    return Some(canvas::Action::request_redraw());
                                }
                                Some(
                                    canvas::Action::publish(SketchMsg::CircleAdded(start, r))
                                        .and_capture(),
                                )
                            }
                            _ => Some(canvas::Action::request_redraw()),
                        }
                    }
                    _ => None,
                }
            }
            _ => None,
        }
    }

    fn draw(
        &self,
        state: &DragState,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let view = self.sketch.view();
        let base = self.sketch.cache.draw(renderer, bounds.size(), |frame| {
            let size = frame.size();
            frame.fill_rectangle(Point::ORIGIN, size, parts::CANVAS_BACKGROUND);

            // 低ズーム時はグリッド線を間引く (スナップ自体は常に 1 grid 単位)
            let step = [1, 5, 10, 50]
                .into_iter()
                .find(|s| *s as f32 * view.zoom >= 4.0)
                .unwrap_or(100);
            let half_w = size.width / (2.0 * view.zoom);
            let half_h = size.height / (2.0 * view.zoom);
            let x_min = (view.center[0] - half_w).floor() as i32;
            let x_max = (view.center[0] + half_w).ceil() as i32;
            for i in (x_min..=x_max).filter(|i| i % step == 0) {
                let x = to_screen(view, size, [i as f64, 0.0]).x;
                let color = if i == 0 { AXIS } else { GRID };
                frame.stroke(
                    &Path::line(Point::new(x, 0.0), Point::new(x, size.height)),
                    Stroke::default().with_width(1.0).with_color(color),
                );
            }
            let y_min = (view.center[1] - half_h).floor() as i32;
            let y_max = (view.center[1] + half_h).ceil() as i32;
            for j in (y_min..=y_max).filter(|j| j % step == 0) {
                let y = to_screen(view, size, [0.0, j as f64]).y;
                let color = if j == 0 { AXIS } else { GRID };
                frame.stroke(
                    &Path::line(Point::new(0.0, y), Point::new(size.width, y)),
                    Stroke::default().with_width(1.0).with_color(color),
                );
            }

            let Some(model) = &self.sketch.model else {
                return;
            };
            for geom in &model.geoms {
                match geom {
                    SketchGeom::Point { pos, .. } => {
                        let c = to_screen(view, size, *pos);
                        let cross = Stroke::default().with_width(2.0).with_color(POINT_COLOR);
                        frame.stroke(
                            &Path::line(
                                Point::new(c.x - 5.0, c.y),
                                Point::new(c.x + 5.0, c.y),
                            ),
                            cross.clone(),
                        );
                        frame.stroke(
                            &Path::line(
                                Point::new(c.x, c.y - 5.0),
                                Point::new(c.x, c.y + 5.0),
                            ),
                            cross,
                        );
                    }
                    SketchGeom::Segment { a, b, .. } => {
                        let pa = to_screen(view, size, *a);
                        let pb = to_screen(view, size, *b);
                        frame.stroke(
                            &Path::line(pa, pb),
                            Stroke::default().with_width(2.0).with_color(SEGMENT_COLOR),
                        );
                        frame.fill(&Path::circle(pa, 3.0), HANDLE);
                        frame.fill(&Path::circle(pb, 3.0), HANDLE);
                    }
                    SketchGeom::Polygon { name, verts } => {
                        if verts.len() < 2 {
                            continue;
                        }
                        let color = if self.sketch.active_poly.as_deref() == Some(name) {
                            ACTIVE_LINE
                        } else {
                            LINE
                        };
                        let path = Path::new(|b| {
                            b.move_to(to_screen(view, size, verts[0]));
                            for v in &verts[1..] {
                                b.line_to(to_screen(view, size, *v));
                            }
                            b.close();
                        });
                        frame.stroke(&path, Stroke::default().with_width(2.0).with_color(color));
                        for v in verts {
                            frame.fill(&Path::circle(to_screen(view, size, *v), 3.5), HANDLE);
                        }
                    }
                    SketchGeom::Circle { center, radius, .. } => {
                        let c = to_screen(view, size, *center);
                        frame.stroke(
                            &Path::circle(c, *radius as f32 * view.zoom),
                            Stroke::default().with_width(2.0).with_color(LINE),
                        );
                        frame.fill(&Path::circle(c, 3.5), HANDLE);
                    }
                }
            }
        });

        let mut overlay = canvas::Frame::new(renderer, bounds.size());
        let osize = overlay.size();

        // hover / ドラッグ中のハンドルと、それに連動して動くハンドルを緑で強調する
        let hot = match state {
            DragState::Handle { target, .. } => Some(*target),
            _ if self.sketch.tool == Tool::Select => cursor
                .position_in(bounds)
                .and_then(|p| self.hit_test(bounds.size(), p)),
            _ => None,
        };
        if let (Some(hot), Some(model)) = (hot, &self.sketch.model) {
            for t in self.linked_targets(hot) {
                if let DragTarget::CircleRadius { geom } = t {
                    if let Some(SketchGeom::Circle { center, radius, .. }) = model.geoms.get(geom)
                    {
                        overlay.stroke(
                            &Path::circle(
                                to_screen(view, osize, *center),
                                *radius as f32 * view.zoom,
                            ),
                            Stroke::default().with_width(3.0).with_color(LINKED_HANDLE),
                        );
                    }
                } else if let Some(pos) = target_pos(model, t) {
                    overlay.fill(&Path::circle(to_screen(view, osize, pos), 5.0), LINKED_HANDLE);
                }
            }
        }

        match state {
            DragState::Draw { start, hover } => {
                let ps = to_screen(view, osize, *start);
                let ph = to_screen(view, osize, *hover);
                match self.sketch.tool {
                    Tool::Circle => {
                        let r = grid_distance(*start, *hover);
                        if r > 0.0 {
                            overlay.stroke(
                                &Path::circle(ps, r as f32 * view.zoom),
                                Stroke::default().with_width(2.0).with_color(DRAG_PREVIEW),
                            );
                        }
                        overlay.stroke(
                            &Path::line(ps, ph),
                            Stroke::default().with_width(1.0).with_color(DRAG_PREVIEW),
                        );
                    }
                    _ => {
                        overlay.stroke(
                            &Path::line(ps, ph),
                            Stroke::default().with_width(2.0).with_color(DRAG_PREVIEW),
                        );
                    }
                }
                overlay.fill(&Path::circle(ph, 4.0), SNAP_INDICATOR);
            }
            DragState::Hover(p) => {
                if self.sketch.tool != Tool::Select {
                    overlay.fill(&Path::circle(to_screen(view, osize, *p), 4.0), SNAP_INDICATOR);
                }
            }
            _ => {}
        }
        vec![base, overlay.into_geometry()]
    }

    fn mouse_interaction(
        &self,
        state: &DragState,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        let Some(pos) = cursor.position_in(bounds) else {
            return mouse::Interaction::default();
        };
        match (self.sketch.tool, state) {
            (Tool::Select, DragState::Handle { .. }) => mouse::Interaction::Grabbing,
            (Tool::Select, _) => {
                if self.hit_test(bounds.size(), pos).is_some() {
                    mouse::Interaction::Grab
                } else {
                    mouse::Interaction::default()
                }
            }
            _ => mouse::Interaction::Crosshair,
        }
    }
}

pub fn view<'a>(s: &'a Sketch, index: usize, total: usize) -> Element<'a, SketchMsg> {
    let label = format!("sketch #{} ({}/{})", s.id, index + 1, total);
    let binding_cb = combo_box(
        &s.binding_state,
        "sketch binding",
        Some(&s.binding),
        SketchMsg::BindingChanged,
    )
    .on_input(SketchMsg::BindingChanged)
    .width(Length::Fixed(140.0))
    .size(13.0);
    let tool_menu = pick_list(TOOLS, Some(s.tool), SketchMsg::SetTool).text_size(13);
    let header = row![
        parts::dark_button("↑").on_press(SketchMsg::MoveUp),
        parts::dark_button("↓").on_press(SketchMsg::MoveDown),
        parts::dark_button(if s.minimized { "▶" } else { "▼" }).on_press(SketchMsg::Minimize),
        text(label),
        binding_cb,
        tool_menu,
        parts::dark_button("Factor xy").on_press(SketchMsg::FactorVars),
        parts::dark_button("×").on_press(SketchMsg::Close),
    ]
    .spacing(2);

    if s.minimized {
        return container(header).padding(4).into();
    }

    let canvas_el: Element<'a, SketchMsg> = canvas_widget(SketchCanvas { sketch: s })
        .width(Fill)
        .height(Length::Fixed(320.0))
        .into();

    let mut geom_list = column![text("Geoms:").size(13)].spacing(2);
    if let Some(model) = &s.model {
        for geom in &model.geoms {
            let name = geom.name().to_string();
            let label = match geom {
                SketchGeom::Point { pos, .. } => {
                    format!("{name} ({}, {})", pos[0], pos[1])
                }
                SketchGeom::Segment { .. } => format!("{name} (line)"),
                SketchGeom::Polygon { verts, .. } => {
                    format!("{name} ({} verts)", verts.len())
                }
                SketchGeom::Circle { radius, center, .. } => {
                    format!("{name} (r={radius} @ ({}, {}))", center[0], center[1])
                }
            };
            let selectable = matches!(geom, SketchGeom::Polygon { .. });
            let marker = if s.active_poly.as_deref() == Some(&name) {
                "●"
            } else {
                "○"
            };
            let mut r = row![].spacing(4);
            if selectable {
                r = r.push(
                    parts::dark_button(marker).on_press(SketchMsg::SelectGeom(name.clone())),
                );
            }
            r = r.push(text(label));
            r = r.push(parts::dark_button("×").on_press(SketchMsg::RemoveGeom(name)));
            geom_list = geom_list.push(r);
        }
    }

    let zoom_row = row![
        text(format!("zoom: {:.0}px/grid", s.zoom)).size(13),
        slider(1.0..=25.0, s.zoom, SketchMsg::ZoomChanged),
        text(format!("center: ({:.0}, {:.0})", s.center[0], s.center[1])).size(13),
        parts::dark_button("origin").on_press(SketchMsg::ResetView),
    ]
    .spacing(8);

    let mut col = column![header, canvas_el, geom_list, zoom_row].spacing(4);
    if s.binding.is_empty() {
        col = col.push(
            text("select a sketch binding to edit")
                .size(13)
                .color(Color::from_rgb(0.6, 0.6, 0.6)),
        );
    }
    if !s.status.is_empty() {
        col = col.push(
            text(&s.status)
                .size(13)
                .color(Color::from_rgb(1.0, 0.7, 0.4)),
        );
    }
    container(col).padding(4).into()
}
