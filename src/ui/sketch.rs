//! 2D sketch workspace。
//!
//! マス目 (1 grid 単位 = 1.0) 付きキャンバスに、ツール (Line / Circle / Point) を
//! 切り替えてドラッグで図形を描く。入力は最寄りの格子交点にスナップされ、
//! 描いた図形から `polygon` / `circle` / `p2` のコードを生成してコピーできる。

use iced::widget::canvas::{self, Path, Stroke};
use iced::widget::{canvas as canvas_widget, column, container, row, slider, text};
use iced::{Color, Element, Fill, Length, Point, Rectangle, Renderer, Size, Theme, mouse};
use serde::{Deserialize, Serialize};

use crate::session::SessionSketch;
use crate::ui::parts;
use crate::ui::workspace::WorkspaceEvent;

/// 格子交点にスナップ済みの点 (grid 単位、y は上向き)。
pub type GridPoint = [i32; 2];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    Line,
    Circle,
    Point,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SketchShape {
    /// 描画順の線分リスト。codegen 時にチェーン連結して polygon にする。
    Polygon {
        lines: Vec<[GridPoint; 2]>,
        /// true なら加算した幾何から diff2d で引く。
        #[serde(default)]
        subtract: bool,
    },
    Circle {
        center: GridPoint,
        radius: i32,
        #[serde(default)]
        subtract: bool,
    },
    /// 幾何には参加せず、`p2` の let 束縛だけを生成する参照点。
    Point(GridPoint),
}

impl SketchShape {
    fn empty_polygon() -> Self {
        SketchShape::Polygon {
            lines: Vec::new(),
            subtract: false,
        }
    }

    fn is_subtract(&self) -> bool {
        match self {
            SketchShape::Polygon { subtract, .. } | SketchShape::Circle { subtract, .. } => {
                *subtract
            }
            SketchShape::Point(_) => false,
        }
    }
}

pub struct Sketch {
    pub id: u64,
    pub minimized: bool,
    /// 1 grid 単位あたりのピクセル数。
    pub zoom: f32,
    /// 描いた図形のリスト。常に 1 つ以上あり、`active` は有効な index。
    pub shapes: Vec<SketchShape>,
    /// 選択中の shape の index。線は選択中の Polygon に追記される。
    pub active: usize,
    /// 現在の入力ツール (セッションに保存しない UI 状態)。
    pub tool: Tool,
    /// グリッド + 確定済み図形のキャッシュ。描画内容が変わる操作で clear する。
    cache: canvas::Cache,
}

impl Sketch {
    pub fn new(id: u64) -> Self {
        Self {
            id,
            minimized: false,
            zoom: 20.0,
            shapes: vec![SketchShape::empty_polygon()],
            active: 0,
            tool: Tool::Line,
            cache: canvas::Cache::new(),
        }
    }

    pub fn from_session(ss: &SessionSketch) -> Self {
        let mut s = Self::new(ss.sketch_id);
        s.minimized = ss.minimized;
        s.zoom = ss.zoom;
        if !ss.shapes.is_empty() {
            s.shapes = ss.shapes.clone();
        }
        s
    }

    pub fn to_session(&self, order: usize) -> SessionSketch {
        SessionSketch {
            sketch_id: self.id,
            order,
            minimized: self.minimized,
            zoom: self.zoom,
            shapes: self.shapes.clone(),
        }
    }

    /// アクティブが Polygon ならそこへ追記、そうでなければ新規 Polygon を起こす。
    pub fn add_line(&mut self, a: GridPoint, b: GridPoint) {
        match &mut self.shapes[self.active] {
            SketchShape::Polygon { lines, .. } => lines.push([a, b]),
            _ => {
                self.shapes.push(SketchShape::Polygon {
                    lines: vec![[a, b]],
                    subtract: false,
                });
                self.active = self.shapes.len() - 1;
            }
        }
        self.cache.clear();
    }

    fn remove_shape(&mut self, i: usize) {
        if i >= self.shapes.len() {
            return;
        }
        self.shapes.remove(i);
        if self.shapes.is_empty() {
            self.shapes.push(SketchShape::empty_polygon());
        }
        if self.active > i {
            self.active -= 1;
        } else {
            self.active = self.active.min(self.shapes.len() - 1);
        }
        self.cache.clear();
    }

    /// sketch 内で完結する状態遷移を適用し、Model レベルの後処理を返す。
    pub fn update(&mut self, msg: SketchMsg) -> WorkspaceEvent {
        match msg {
            SketchMsg::Close => WorkspaceEvent::Close,
            SketchMsg::MoveUp => WorkspaceEvent::MoveUp,
            SketchMsg::MoveDown => WorkspaceEvent::MoveDown,
            SketchMsg::Minimize => {
                self.minimized = !self.minimized;
                WorkspaceEvent::Edited
            }
            // ツールはセッションに保存しない UI 状態なので unsaved 化しない
            SketchMsg::SetTool(tool) => {
                self.tool = tool;
                WorkspaceEvent::None
            }
            SketchMsg::LineAdded(a, b) => {
                self.add_line(a, b);
                WorkspaceEvent::Edited
            }
            // 円と点は active を変えずリスト末尾に追加する
            SketchMsg::CircleAdded(center, radius) => {
                self.shapes.push(SketchShape::Circle {
                    center,
                    radius,
                    subtract: false,
                });
                self.cache.clear();
                WorkspaceEvent::Edited
            }
            SketchMsg::PointAdded(p) => {
                self.shapes.push(SketchShape::Point(p));
                self.cache.clear();
                WorkspaceEvent::Edited
            }
            SketchMsg::ZoomChanged(zoom) => {
                self.zoom = zoom;
                self.cache.clear();
                WorkspaceEvent::Edited
            }
            SketchMsg::Undo => {
                match &mut self.shapes[self.active] {
                    SketchShape::Polygon { lines, .. } => {
                        lines.pop();
                    }
                    _ => self.remove_shape(self.active),
                }
                self.cache.clear();
                WorkspaceEvent::Edited
            }
            SketchMsg::Clear => {
                if let SketchShape::Polygon { lines, .. } = &mut self.shapes[self.active] {
                    lines.clear();
                    self.cache.clear();
                }
                WorkspaceEvent::Edited
            }
            SketchMsg::AddPolygon => {
                self.shapes.push(SketchShape::empty_polygon());
                self.active = self.shapes.len() - 1;
                self.cache.clear();
                WorkspaceEvent::Edited
            }
            SketchMsg::ToggleSubtract(i) => {
                if let Some(
                    SketchShape::Polygon { subtract, .. } | SketchShape::Circle { subtract, .. },
                ) = self.shapes.get_mut(i)
                {
                    *subtract = !*subtract;
                    self.cache.clear();
                }
                WorkspaceEvent::Edited
            }
            // 選択はセッションに保存しない UI 状態なので unsaved 化しない
            SketchMsg::SelectShape(i) => {
                if i < self.shapes.len() {
                    self.active = i;
                    self.cache.clear();
                }
                WorkspaceEvent::None
            }
            SketchMsg::RemoveShape(i) => {
                self.remove_shape(i);
                WorkspaceEvent::Edited
            }
            SketchMsg::CopyShape(i) => WorkspaceEvent::CopyRequested(self.shape_code_at(i)),
            SketchMsg::CopyCode => WorkspaceEvent::CopyRequested(self.code()),
        }
    }

    /// 指定 shape 単体のコード (座標の let くくり出しはその shape 内で閉じる。
    /// subtract フラグは単体コピーでは無視する)。
    pub fn shape_code_at(&self, idx: usize) -> String {
        match self.shapes.get(idx) {
            None => String::new(),
            Some(SketchShape::Polygon { lines, .. }) => single_polygon_code(&chain_points(lines)),
            Some(SketchShape::Circle { center, radius, .. }) => {
                circle_expr(*center, *radius, &[], &[])
            }
            Some(SketchShape::Point(p)) => {
                format!("p2 {} {}", fmt_coord(p[0]), fmt_coord(p[1]))
            }
        }
    }

    /// 全 shape の結合コード。加算の幾何 (polygon / circle) は union2d で畳み、
    /// subtract の幾何はその結果から diff2d で引く。点は `p2` の let 束縛として並べる。
    /// 座標の let くくり出しは全 shape 横断。
    /// 幾何が無く点だけの場合は、貼り付け先の let に入れる想定で束縛行だけを出す。
    pub fn code(&self) -> String {
        enum Geom {
            Poly(Vec<GridPoint>),
            Circ { center: GridPoint, radius: i32 },
        }
        let mut geoms: Vec<(String, Geom, bool)> = Vec::new();
        let mut points: Vec<(String, GridPoint)> = Vec::new();
        let (mut poly_n, mut circ_n, mut pt_n) = (0, 0, 0);
        for shape in &self.shapes {
            match shape {
                SketchShape::Polygon { lines, subtract } => {
                    // 空 polygon も番号を消費し、panel 行のラベルと名前を一致させる
                    poly_n += 1;
                    let chain = chain_points(lines);
                    if chain.is_empty() {
                        continue;
                    }
                    geoms.push((format!("poly{poly_n}"), Geom::Poly(chain), *subtract));
                }
                SketchShape::Circle {
                    center,
                    radius,
                    subtract,
                } => {
                    circ_n += 1;
                    geoms.push((
                        format!("circ{circ_n}"),
                        Geom::Circ {
                            center: *center,
                            radius: *radius,
                        },
                        *subtract,
                    ));
                }
                SketchShape::Point(p) => {
                    pt_n += 1;
                    points.push((format!("pt{pt_n}"), *p));
                }
            }
        }
        if geoms.is_empty() && points.is_empty() {
            return String::new();
        }

        let mut pool: Vec<GridPoint> = Vec::new();
        for (_, g, _) in &geoms {
            match g {
                Geom::Poly(chain) => pool.extend(chain.iter().copied()),
                Geom::Circ { center, .. } => pool.push(*center),
            }
        }
        pool.extend(points.iter().map(|(_, p)| *p));
        let x_vars = shared_coord_vars(pool.iter().map(|p| p[0]), "x");
        let y_vars = shared_coord_vars(pool.iter().map(|p| p[1]), "y");

        let geom_exprs: Vec<(String, String, bool)> = geoms
            .iter()
            .map(|(name, g, subtract)| {
                let expr = match g {
                    Geom::Poly(chain) => polygon_expr(chain, &x_vars, &y_vars),
                    Geom::Circ { center, radius } => {
                        circle_expr(*center, *radius, &x_vars, &y_vars)
                    }
                };
                (name.clone(), expr, *subtract)
            })
            .collect();
        let point_bindings: Vec<String> = points
            .iter()
            .map(|(name, p)| {
                format!(
                    "{name} = p2 {} {}",
                    coord_term(p[0], &x_vars),
                    coord_term(p[1], &y_vars)
                )
            })
            .collect();

        if geom_exprs.is_empty() {
            let mut out = String::new();
            for (v, name) in x_vars.iter().chain(y_vars.iter()) {
                out.push_str(&format!("{name} = {}\n", fmt_coord(*v)));
            }
            for b in &point_bindings {
                out.push_str(b);
                out.push('\n');
            }
            return out.trim_end().to_string();
        }

        let adds: Vec<&String> = geom_exprs
            .iter()
            .filter(|(_, _, sub)| !sub)
            .map(|(name, _, _)| name)
            .collect();
        let subs: Vec<&String> = geom_exprs
            .iter()
            .filter(|(_, _, sub)| *sub)
            .map(|(name, _, _)| name)
            .collect();
        // 全部 subtract なら引く相手がいないので加算として扱う
        let (adds, subs) = if adds.is_empty() {
            (subs, Vec::new())
        } else {
            (adds, subs)
        };

        let body = if geom_exprs.len() == 1 {
            geom_exprs[0].1.clone()
        } else if subs.is_empty() {
            union_fold(&adds)
        } else {
            format!(
                "diff2d {} {}",
                paren_if_compound(union_fold(&adds)),
                paren_if_compound(union_fold(&subs))
            )
        };
        let needs_geom_bindings = geom_exprs.len() > 1;
        if x_vars.is_empty() && y_vars.is_empty() && !needs_geom_bindings && points.is_empty() {
            return body;
        }

        let mut out = String::from("let\n");
        for (v, name) in x_vars.iter().chain(y_vars.iter()) {
            out.push_str(&format!("    {name} = {}\n", fmt_coord(*v)));
        }
        if needs_geom_bindings {
            for (name, expr, _) in &geom_exprs {
                out.push_str(&format!("    {name} = {expr}\n"));
            }
        }
        for b in &point_bindings {
            out.push_str("    ");
            out.push_str(b);
            out.push('\n');
        }
        out.push_str("in\n");
        out.push_str(&body);
        out
    }
}

/// 線分を描画順に連結した頂点列。連続する線分が繋がっていればチェーンとして
/// 畳み、末尾が先頭に戻る閉ループなら末尾を落とす (polygon は暗黙に閉じる)。
fn chain_points(lines: &[[GridPoint; 2]]) -> Vec<GridPoint> {
    let mut pts: Vec<GridPoint> = Vec::new();
    for [a, b] in lines {
        if pts.last() != Some(a) {
            pts.push(*a);
        }
        pts.push(*b);
    }
    if pts.len() > 2 && pts.first() == pts.last() {
        pts.pop();
    }
    pts
}

fn union_fold(names: &[&String]) -> String {
    if names.len() == 1 {
        return names[0].clone();
    }
    let mut expr = format!("union2d {} {}", names[0], names[1]);
    for name in &names[2..] {
        expr = format!("union2d ({expr}) {name}");
    }
    expr
}

fn paren_if_compound(expr: String) -> String {
    if expr.contains(' ') {
        format!("({expr})")
    } else {
        expr
    }
}

fn coord_term(v: i32, vars: &[(i32, String)]) -> String {
    vars.iter()
        .find(|(w, _)| *w == v)
        .map(|(_, name)| name.clone())
        .unwrap_or_else(|| fmt_coord(v))
}

fn polygon_expr(pts: &[GridPoint], x_vars: &[(i32, String)], y_vars: &[(i32, String)]) -> String {
    let body = pts
        .iter()
        .map(|[x, y]| format!("p2 {} {}", coord_term(*x, x_vars), coord_term(*y, y_vars)))
        .collect::<Vec<_>>()
        .join(", ");
    format!("polygon [{body}]")
}

fn circle_expr(
    center: GridPoint,
    radius: i32,
    x_vars: &[(i32, String)],
    y_vars: &[(i32, String)],
) -> String {
    let r = fmt_coord(radius);
    if center == [0, 0] {
        format!("circle {r}")
    } else {
        format!(
            "circle {r} |> translate2d (p2 0.0 0.0) (p2 {} {})",
            coord_term(center[0], x_vars),
            coord_term(center[1], y_vars)
        )
    }
}

/// 同一軸内で 2 回以上使われる座標値は let 変数にくくり出す。
/// 変数名は軸ごとの出現順で x1, x2, ... / y1, y2, ...。
fn single_polygon_code(pts: &[GridPoint]) -> String {
    if pts.is_empty() {
        return String::new();
    }
    let x_vars = shared_coord_vars(pts.iter().map(|p| p[0]), "x");
    let y_vars = shared_coord_vars(pts.iter().map(|p| p[1]), "y");
    let poly = polygon_expr(pts, &x_vars, &y_vars);
    if x_vars.is_empty() && y_vars.is_empty() {
        return poly;
    }
    let mut out = String::from("let\n");
    for (v, name) in x_vars.iter().chain(y_vars.iter()) {
        out.push_str(&format!("    {name} = {}\n", fmt_coord(*v)));
    }
    out.push_str("in\n");
    out.push_str(&poly);
    out
}

/// 2 回以上現れる値を出現順に (値, 変数名) で列挙する。
fn shared_coord_vars(values: impl Iterator<Item = i32>, prefix: &str) -> Vec<(i32, String)> {
    let vals: Vec<i32> = values.collect();
    let mut vars: Vec<(i32, String)> = Vec::new();
    for &v in &vals {
        if vars.iter().any(|(w, _)| *w == v) {
            continue;
        }
        if vals.iter().filter(|&&w| w == v).count() >= 2 {
            vars.push((v, format!("{prefix}{}", vars.len() + 1)));
        }
    }
    vars
}

/// 負数は `p2 -3.0 ...` が減算に解釈されるため括弧で包む。
fn fmt_coord(v: i32) -> String {
    if v < 0 {
        format!("({v}.0)")
    } else {
        format!("{v}.0")
    }
}

fn grid_distance(a: GridPoint, b: GridPoint) -> i32 {
    let dx = (b[0] - a[0]) as f32;
    let dy = (b[1] - a[1]) as f32;
    (dx * dx + dy * dy).sqrt().round() as i32
}

#[derive(Debug, Clone)]
pub enum SketchMsg {
    Close,
    MoveUp,
    MoveDown,
    Minimize,
    SetTool(Tool),
    LineAdded(GridPoint, GridPoint),
    /// 中心と grid 単位の半径。
    CircleAdded(GridPoint, i32),
    PointAdded(GridPoint),
    ZoomChanged(f32),
    /// アクティブが Polygon なら最後の線を、そうでなければその shape を取り消す。
    Undo,
    /// アクティブ Polygon の線を全消去する (Polygon 以外では no-op)。
    Clear,
    AddPolygon,
    SelectShape(usize),
    RemoveShape(usize),
    /// 指定 shape の加算/減算 (diff2d) を切り替える。
    ToggleSubtract(usize),
    /// 指定 shape 単体のコードをコピー。
    CopyShape(usize),
    /// 全 shape の結合コードをコピー。
    CopyCode,
}

const GRID: Color = Color::from_rgb(0.22, 0.22, 0.25);
const AXIS: Color = Color::from_rgb(0.38, 0.38, 0.44);
const LINE: Color = Color::from_rgb(0.55, 0.85, 0.55);
const VERTEX: Color = Color::from_rgb(0.75, 0.95, 0.75);
const INACTIVE_LINE: Color = Color::from_rgb(0.35, 0.5, 0.35);
const INACTIVE_VERTEX: Color = Color::from_rgb(0.45, 0.58, 0.45);
const SUBTRACT_LINE: Color = Color::from_rgb(0.9, 0.5, 0.5);
const SUBTRACT_VERTEX: Color = Color::from_rgb(1.0, 0.65, 0.65);
const SUBTRACT_INACTIVE_LINE: Color = Color::from_rgb(0.55, 0.35, 0.35);
const SUBTRACT_INACTIVE_VERTEX: Color = Color::from_rgb(0.65, 0.45, 0.45);
const DRAG_PREVIEW: Color = Color::from_rgba(0.55, 0.85, 0.55, 0.5);
const SNAP_INDICATOR: Color = Color::from_rgba(0.95, 0.85, 0.4, 0.8);

/// キャンバスの中心が world 原点。y は上向き (screen y と反転)。
fn to_screen(zoom: f32, size: Size, p: GridPoint) -> Point {
    Point::new(
        size.width / 2.0 + p[0] as f32 * zoom,
        size.height / 2.0 - p[1] as f32 * zoom,
    )
}

fn snap(zoom: f32, size: Size, local: Point) -> GridPoint {
    [
        ((local.x - size.width / 2.0) / zoom).round() as i32,
        ((size.height / 2.0 - local.y) / zoom).round() as i32,
    ]
}

struct SketchCanvas<'a> {
    sketch: &'a Sketch,
}

#[derive(Default)]
struct DragState {
    start: Option<GridPoint>,
    hover: Option<GridPoint>,
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
        let zoom = self.sketch.zoom;
        match mouse_event {
            mouse::Event::ButtonPressed(mouse::Button::Left) => {
                let pos = cursor.position_in(bounds)?;
                let p = snap(zoom, bounds.size(), pos);
                if self.sketch.tool == Tool::Point {
                    return Some(canvas::Action::publish(SketchMsg::PointAdded(p)).and_capture());
                }
                state.start = Some(p);
                state.hover = Some(p);
                Some(canvas::Action::request_redraw().and_capture())
            }
            mouse::Event::CursorMoved { .. } => {
                let hover = cursor
                    .position_in(bounds)
                    .map(|p| snap(zoom, bounds.size(), p));
                if state.hover == hover {
                    return None;
                }
                state.hover = hover;
                Some(canvas::Action::request_redraw())
            }
            mouse::Event::ButtonReleased(mouse::Button::Left) => {
                let start = state.start.take()?;
                // キャンバス外で離した場合はキャンセル
                let pos = cursor.position_in(bounds)?;
                let end = snap(zoom, bounds.size(), pos);
                match self.sketch.tool {
                    Tool::Line => {
                        if start == end {
                            return Some(canvas::Action::request_redraw());
                        }
                        Some(
                            canvas::Action::publish(SketchMsg::LineAdded(start, end))
                                .and_capture(),
                        )
                    }
                    Tool::Circle => {
                        let r = grid_distance(start, end);
                        if r == 0 {
                            return Some(canvas::Action::request_redraw());
                        }
                        Some(
                            canvas::Action::publish(SketchMsg::CircleAdded(start, r))
                                .and_capture(),
                        )
                    }
                    Tool::Point => Some(canvas::Action::request_redraw()),
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
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let zoom = self.sketch.zoom;
        let base = self.sketch.cache.draw(renderer, bounds.size(), |frame| {
            let size = frame.size();
            frame.fill_rectangle(Point::ORIGIN, size, parts::CANVAS_BACKGROUND);

            // 低ズーム時はグリッド線を間引く (スナップ自体は常に 1 grid 単位)
            let step = [1, 5, 10, 50]
                .into_iter()
                .find(|s| *s as f32 * zoom >= 4.0)
                .unwrap_or(100);
            let nx = (size.width / (2.0 * zoom)).ceil() as i32;
            let ny = (size.height / (2.0 * zoom)).ceil() as i32;
            for i in (-nx..=nx).filter(|i| i % step == 0) {
                let x = size.width / 2.0 + i as f32 * zoom;
                let color = if i == 0 { AXIS } else { GRID };
                frame.stroke(
                    &Path::line(Point::new(x, 0.0), Point::new(x, size.height)),
                    Stroke::default().with_width(1.0).with_color(color),
                );
            }
            for j in (-ny..=ny).filter(|j| j % step == 0) {
                let y = size.height / 2.0 + j as f32 * zoom;
                let color = if j == 0 { AXIS } else { GRID };
                frame.stroke(
                    &Path::line(Point::new(0.0, y), Point::new(size.width, y)),
                    Stroke::default().with_width(1.0).with_color(color),
                );
            }

            for (si, shape) in self.sketch.shapes.iter().enumerate() {
                let (line_color, vertex_color) =
                    match (si == self.sketch.active, shape.is_subtract()) {
                        (true, false) => (LINE, VERTEX),
                        (false, false) => (INACTIVE_LINE, INACTIVE_VERTEX),
                        (true, true) => (SUBTRACT_LINE, SUBTRACT_VERTEX),
                        (false, true) => (SUBTRACT_INACTIVE_LINE, SUBTRACT_INACTIVE_VERTEX),
                    };
                match shape {
                    SketchShape::Polygon { lines, .. } => {
                        for [a, b] in lines {
                            let pa = to_screen(zoom, size, *a);
                            let pb = to_screen(zoom, size, *b);
                            frame.stroke(
                                &Path::line(pa, pb),
                                Stroke::default().with_width(2.0).with_color(line_color),
                            );
                            frame.fill(&Path::circle(pa, 3.0), vertex_color);
                            frame.fill(&Path::circle(pb, 3.0), vertex_color);
                        }
                    }
                    SketchShape::Circle { center, radius, .. } => {
                        let c = to_screen(zoom, size, *center);
                        frame.stroke(
                            &Path::circle(c, *radius as f32 * zoom),
                            Stroke::default().with_width(2.0).with_color(line_color),
                        );
                        frame.fill(&Path::circle(c, 3.0), vertex_color);
                    }
                    SketchShape::Point(p) => {
                        let c = to_screen(zoom, size, *p);
                        let cross = Stroke::default().with_width(2.0).with_color(vertex_color);
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
                }
            }
        });

        let mut overlay = canvas::Frame::new(renderer, bounds.size());
        if let Some(hover) = state.hover {
            let ph = to_screen(zoom, overlay.size(), hover);
            if let Some(start) = state.start {
                let ps = to_screen(zoom, overlay.size(), start);
                match self.sketch.tool {
                    Tool::Circle => {
                        let r = grid_distance(start, hover);
                        if r > 0 {
                            overlay.stroke(
                                &Path::circle(ps, r as f32 * zoom),
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
            }
            overlay.fill(&Path::circle(ph, 4.0), SNAP_INDICATOR);
        }
        vec![base, overlay.into_geometry()]
    }

    fn mouse_interaction(
        &self,
        _state: &DragState,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            mouse::Interaction::Crosshair
        } else {
            mouse::Interaction::default()
        }
    }
}

pub fn view<'a>(s: &'a Sketch, index: usize, total: usize) -> Element<'a, SketchMsg> {
    let label = format!("sketch #{} ({}/{})", s.id, index + 1, total);
    let header = row![
        parts::dark_button("↑").on_press(SketchMsg::MoveUp),
        parts::dark_button("↓").on_press(SketchMsg::MoveDown),
        parts::dark_button(if s.minimized { "▶" } else { "▼" }).on_press(SketchMsg::Minimize),
        text(label),
        parts::dark_button(if s.tool == Tool::Line { "[Line]" } else { "Line" })
            .on_press(SketchMsg::SetTool(Tool::Line)),
        parts::dark_button(if s.tool == Tool::Circle { "[Circle]" } else { "Circle" })
            .on_press(SketchMsg::SetTool(Tool::Circle)),
        parts::dark_button(if s.tool == Tool::Point { "[Point]" } else { "Point" })
            .on_press(SketchMsg::SetTool(Tool::Point)),
        parts::dark_button("Undo").on_press(SketchMsg::Undo),
        parts::dark_button("Clear").on_press(SketchMsg::Clear),
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

    let mut shape_list = column![
        row![
            text("Shapes:").size(13),
            parts::dark_button("+ Polygon").on_press(SketchMsg::AddPolygon),
        ]
        .spacing(8),
    ]
    .spacing(2);
    let (mut poly_n, mut circ_n, mut pt_n) = (0, 0, 0);
    for (i, shape) in s.shapes.iter().enumerate() {
        // ラベルは codegen の束縛名と揃える
        let (label, has_content) = match shape {
            SketchShape::Polygon { lines, .. } => {
                poly_n += 1;
                (
                    format!("poly{poly_n} ({} lines)", lines.len()),
                    !lines.is_empty(),
                )
            }
            SketchShape::Circle { center, radius, .. } => {
                circ_n += 1;
                (
                    format!("circ{circ_n} (r={radius} @ ({}, {}))", center[0], center[1]),
                    true,
                )
            }
            SketchShape::Point(p) => {
                pt_n += 1;
                (format!("pt{pt_n} ({}, {})", p[0], p[1]), true)
            }
        };
        let marker = if i == s.active { "●" } else { "○" };
        let mut r = row![
            parts::dark_button(marker).on_press(SketchMsg::SelectShape(i)),
            text(label),
        ]
        .spacing(4);
        if !matches!(shape, SketchShape::Point(_)) {
            r = r.push(
                parts::dark_button(if shape.is_subtract() { "−" } else { "+" })
                    .on_press(SketchMsg::ToggleSubtract(i)),
            );
        }
        if has_content {
            r = r.push(parts::dark_button("Copy").on_press(SketchMsg::CopyShape(i)));
        }
        r = r.push(parts::dark_button("×").on_press(SketchMsg::RemoveShape(i)));
        shape_list = shape_list.push(r);
    }

    let zoom_row = row![
        text(format!("zoom: {:.0}px/grid", s.zoom)).size(13),
        slider(1.0..=25.0, s.zoom, SketchMsg::ZoomChanged),
    ]
    .spacing(8);

    let code = s.code();
    let code_row: Element<'a, SketchMsg> = if code.is_empty() {
        text("drag on the canvas to draw")
            .size(13)
            .color(Color::from_rgb(0.6, 0.6, 0.6))
            .into()
    } else {
        row![
            parts::dark_button("Copy").on_press(SketchMsg::CopyCode),
            text(code).size(13).font(iced::Font::MONOSPACE),
        ]
        .spacing(8)
        .into()
    };

    container(column![header, canvas_el, shape_list, zoom_row, code_row].spacing(4))
        .padding(4)
        .into()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sketch_with(lines: &[[GridPoint; 2]]) -> Sketch {
        let mut s = Sketch::new(0);
        for [a, b] in lines {
            s.add_line(*a, *b);
        }
        s
    }

    fn poly_lines(s: &Sketch, i: usize) -> &Vec<[GridPoint; 2]> {
        match &s.shapes[i] {
            SketchShape::Polygon { lines, .. } => lines,
            other => panic!("expected polygon at {i}, got {other:?}"),
        }
    }

    #[test]
    fn connected_lines_chain_into_polygon() {
        let s = sketch_with(&[[[0, 0], [10, 0]], [[10, 0], [10, 5]]]);
        assert_eq!(chain_points(poly_lines(&s, 0)), vec![[0, 0], [10, 0], [10, 5]]);
        assert_eq!(
            s.code(),
            "let\n    x1 = 10.0\n    y1 = 0.0\nin\npolygon [p2 0.0 y1, p2 x1 y1, p2 x1 5.0]"
        );
    }

    #[test]
    fn unique_coords_stay_literal() {
        let s = sketch_with(&[[[0, 1], [10, 2]]]);
        assert_eq!(s.code(), "polygon [p2 0.0 1.0, p2 10.0 2.0]");
    }

    #[test]
    fn shared_coords_are_let_bound_per_axis() {
        // 矩形: 全座標値が 2 回ずつ使われる
        let s = sketch_with(&[
            [[-1, 0], [2, 0]],
            [[2, 0], [2, 3]],
            [[2, 3], [-1, 3]],
            [[-1, 3], [-1, 0]],
        ]);
        assert_eq!(
            s.code(),
            "let\n    x1 = (-1.0)\n    x2 = 2.0\n    y1 = 0.0\n    y2 = 3.0\nin\npolygon [p2 x1 y1, p2 x2 y1, p2 x2 y2, p2 x1 y2]"
        );
    }

    #[test]
    fn closed_loop_drops_duplicated_last_point() {
        let s = sketch_with(&[
            [[0, 0], [10, 0]],
            [[10, 0], [10, 5]],
            [[10, 5], [0, 0]],
        ]);
        assert_eq!(chain_points(poly_lines(&s, 0)), vec![[0, 0], [10, 0], [10, 5]]);
    }

    #[test]
    fn negative_coords_are_parenthesized() {
        let s = sketch_with(&[[[-1, 0], [0, -2]]]);
        assert_eq!(s.code(), "polygon [p2 (-1.0) 0.0, p2 0.0 (-2.0)]");
    }

    #[test]
    fn disconnected_segments_keep_both_endpoints() {
        let s = sketch_with(&[[[0, 0], [1, 0]], [[2, 2], [3, 2]]]);
        assert_eq!(
            chain_points(poly_lines(&s, 0)),
            vec![[0, 0], [1, 0], [2, 2], [3, 2]]
        );
    }

    #[test]
    fn multiple_polygons_are_let_bound_and_unioned() {
        let mut s = sketch_with(&[[[0, 0], [4, 0]], [[4, 0], [0, 4]]]);
        s.update(SketchMsg::AddPolygon);
        s.add_line([10, 0], [14, 0]);
        s.add_line([14, 0], [10, 4]);
        assert_eq!(
            s.code(),
            "let\n    x1 = 0.0\n    x2 = 10.0\n    y1 = 0.0\n    y2 = 4.0\n    poly1 = polygon [p2 x1 y1, p2 4.0 y1, p2 x1 y2]\n    poly2 = polygon [p2 x2 y1, p2 14.0 y1, p2 x2 y2]\nin\nunion2d poly1 poly2"
        );
    }

    #[test]
    fn three_polygons_fold_with_union2d() {
        let mut s = sketch_with(&[[[0, 0], [1, 1]]]);
        s.update(SketchMsg::AddPolygon);
        s.add_line([2, 2], [3, 3]);
        s.update(SketchMsg::AddPolygon);
        s.add_line([4, 4], [5, 5]);
        assert!(
            s.code()
                .ends_with("in\nunion2d (union2d poly1 poly2) poly3")
        );
    }

    #[test]
    fn empty_polygons_are_skipped_in_code() {
        let mut s = sketch_with(&[[[0, 0], [1, 2]]]);
        s.update(SketchMsg::AddPolygon);
        // アクティブな空 polygon があっても single 形式のまま
        assert_eq!(s.code(), "polygon [p2 0.0 0.0, p2 1.0 2.0]");
    }

    #[test]
    fn copy_shape_is_scoped_to_that_shape() {
        let mut s = sketch_with(&[[[0, 0], [4, 0]], [[4, 0], [4, 4]]]);
        s.update(SketchMsg::AddPolygon);
        s.add_line([9, 9], [8, 8]);
        s.update(SketchMsg::CircleAdded([2, 3], 5));
        assert_eq!(
            s.shape_code_at(0),
            "let\n    x1 = 4.0\n    y1 = 0.0\nin\npolygon [p2 0.0 y1, p2 x1 y1, p2 x1 4.0]"
        );
        assert_eq!(s.shape_code_at(1), "polygon [p2 9.0 9.0, p2 8.0 8.0]");
        assert_eq!(
            s.shape_code_at(2),
            "circle 5.0 |> translate2d (p2 0.0 0.0) (p2 2.0 3.0)"
        );
    }

    #[test]
    fn remove_shape_keeps_active_valid() {
        let mut s = sketch_with(&[[[0, 0], [1, 0]]]);
        s.update(SketchMsg::AddPolygon);
        s.add_line([2, 0], [3, 0]);
        assert_eq!(s.active, 1);

        // 前の shape を消すと active は左に詰まる
        s.update(SketchMsg::RemoveShape(0));
        assert_eq!(s.active, 0);
        assert_eq!(s.shapes.len(), 1);
        assert_eq!(poly_lines(&s, 0), &vec![[[2, 0], [3, 0]]]);

        // 最後の 1 つを消すと空 polygon が補充される
        s.update(SketchMsg::RemoveShape(0));
        assert_eq!(s.active, 0);
        assert_eq!(s.shapes.len(), 1);
        assert!(poly_lines(&s, 0).is_empty());
    }

    #[test]
    fn circle_at_origin_is_bare_circle() {
        let mut s = Sketch::new(0);
        s.update(SketchMsg::CircleAdded([0, 0], 3));
        assert_eq!(s.code(), "circle 3.0");
    }

    #[test]
    fn circle_and_polygon_union_with_shared_coords() {
        let mut s = sketch_with(&[[[0, 0], [4, 0]], [[4, 0], [0, 4]]]);
        s.update(SketchMsg::CircleAdded([4, 4], 2));
        assert_eq!(
            s.code(),
            "let\n    x1 = 0.0\n    x2 = 4.0\n    y1 = 0.0\n    y2 = 4.0\n    poly1 = polygon [p2 x1 y1, p2 x2 y1, p2 x1 y2]\n    circ1 = circle 2.0 |> translate2d (p2 0.0 0.0) (p2 x2 y2)\nin\nunion2d poly1 circ1"
        );
    }

    #[test]
    fn points_only_emit_bare_bindings() {
        let mut s = Sketch::new(0);
        s.update(SketchMsg::PointAdded([3, 4]));
        s.update(SketchMsg::PointAdded([3, 8]));
        assert_eq!(s.code(), "x1 = 3.0\npt1 = p2 x1 4.0\npt2 = p2 x1 8.0");
    }

    #[test]
    fn point_with_geometry_is_let_bound() {
        let mut s = sketch_with(&[[[0, 0], [5, 0]]]);
        s.update(SketchMsg::PointAdded([5, 5]));
        assert_eq!(
            s.code(),
            "let\n    x1 = 5.0\n    y1 = 0.0\n    pt1 = p2 x1 5.0\nin\npolygon [p2 0.0 y1, p2 x1 y1]"
        );
    }

    #[test]
    fn line_drawn_while_circle_selected_starts_new_polygon() {
        let mut s = Sketch::new(0);
        s.update(SketchMsg::CircleAdded([0, 0], 2));
        s.update(SketchMsg::SelectShape(1));
        s.add_line([1, 1], [2, 1]);
        assert_eq!(s.shapes.len(), 3);
        assert_eq!(s.active, 2);
        assert_eq!(poly_lines(&s, 2), &vec![[[1, 1], [2, 1]]]);
    }

    #[test]
    fn undo_on_non_polygon_removes_the_shape() {
        let mut s = Sketch::new(0);
        s.update(SketchMsg::CircleAdded([0, 0], 2));
        s.update(SketchMsg::SelectShape(1));
        s.update(SketchMsg::Undo);
        assert_eq!(s.shapes.len(), 1);
        assert!(matches!(s.shapes[0], SketchShape::Polygon { .. }));
    }

    #[test]
    fn subtract_shape_generates_diff2d() {
        let mut s = sketch_with(&[[[0, 0], [4, 0]], [[4, 0], [0, 4]]]);
        s.update(SketchMsg::CircleAdded([1, 1], 1));
        s.update(SketchMsg::ToggleSubtract(1));
        assert_eq!(
            s.code(),
            "let\n    x1 = 0.0\n    y1 = 0.0\n    poly1 = polygon [p2 x1 y1, p2 4.0 y1, p2 x1 4.0]\n    circ1 = circle 1.0 |> translate2d (p2 0.0 0.0) (p2 1.0 1.0)\nin\ndiff2d poly1 circ1"
        );
    }

    #[test]
    fn multiple_adds_and_subs_fold_before_diff2d() {
        let mut s = sketch_with(&[[[0, 0], [1, 2]]]);
        s.update(SketchMsg::AddPolygon);
        s.add_line([10, 0], [11, 2]);
        s.update(SketchMsg::CircleAdded([20, 20], 1));
        s.update(SketchMsg::CircleAdded([30, 30], 1));
        s.update(SketchMsg::ToggleSubtract(2));
        s.update(SketchMsg::ToggleSubtract(3));
        assert!(
            s.code()
                .ends_with("in\ndiff2d (union2d poly1 poly2) (union2d circ1 circ2)")
        );
    }

    #[test]
    fn lone_subtract_is_treated_as_add() {
        let mut s = Sketch::new(0);
        s.update(SketchMsg::CircleAdded([0, 0], 2));
        s.update(SketchMsg::ToggleSubtract(1));
        assert_eq!(s.code(), "circle 2.0");
    }

    #[test]
    fn grid_distance_rounds_to_nearest_int() {
        assert_eq!(grid_distance([0, 0], [3, 4]), 5);
        assert_eq!(grid_distance([0, 0], [1, 1]), 1); // sqrt(2) ≈ 1.41 → 1
        assert_eq!(grid_distance([2, 2], [2, 2]), 0);
    }

    #[test]
    fn generated_code_compiles_as_cadhr_lang() {
        // let 括り出し (負数 RHS 含む)・複数 polygon・circle・point・diff2d を通る形で、
        // 貼り付け時と同様に binding body としてインデントして埋め込む。
        let mut s = sketch_with(&[
            [[-1, 0], [2, 0]],
            [[2, 0], [2, 3]],
            [[2, 3], [-1, 3]],
            [[-1, 3], [-1, 0]],
        ]);
        s.update(SketchMsg::AddPolygon);
        s.add_line([5, 0], [8, 0]);
        s.add_line([8, 0], [5, 3]);
        s.update(SketchMsg::CircleAdded([-1, 3], 2));
        s.update(SketchMsg::PointAdded([2, 3]));
        s.update(SketchMsg::CircleAdded([0, 1], 1));
        s.update(SketchMsg::ToggleSubtract(4));
        let indented = s
            .code()
            .lines()
            .map(|l| format!("            {l}"))
            .collect::<Vec<_>>()
            .join("\n");
        let src = format!(
            "main =\n    let\n        shape =\n{indented}\n    in\n    {{ models = [extrude_xy 1.0 shape], bom = [], controls = [] }}\n"
        );
        cadhr_lang::compile(&src).unwrap_or_else(|e| panic!("generated code failed: {e:?}"));
    }

    #[test]
    fn snap_rounds_to_nearest_grid_intersection() {
        let size = Size::new(200.0, 100.0);
        // 中心 (100, 50) が原点。zoom 20 で (129, 32) は world (1.45, 0.9) → (1, 1)
        assert_eq!(snap(20.0, size, Point::new(129.0, 32.0)), [1, 1]);
        assert_eq!(to_screen(20.0, size, [1, 1]), Point::new(120.0, 30.0));
    }
}
