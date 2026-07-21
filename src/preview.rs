pub mod pipeline;

use glam::{Mat4, Vec3};
use iced::advanced::graphics::Viewport;
use iced::mouse;
use iced::wgpu;
use iced::widget::shader;
use iced::{Event, Point, Rectangle};
use pipeline::{MeshData, Pipeline, Uniforms, Vertex};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

static NEXT_SCENE_ID: AtomicU64 = AtomicU64::new(1);
static NEXT_MESH_VERSION: AtomicU64 = AtomicU64::new(1);
/// Scene が drop された ID をここに積み、次回 Primitive::prepare で Pipeline から撤去する
static PENDING_REMOVALS: Mutex<Vec<u64>> = Mutex::new(Vec::new());

const DEFAULT_COLOR: [f32; 4] = [0.7, 0.2, 0.2, 0.5];
const DEFAULT_LIGHT_DIR: [f32; 4] = [-0.5, -1.0, -0.3, 0.0];
/// Sharp edge を描く線の色 (RGBA)
const DEFAULT_EDGE_COLOR: [f32; 4] = [0.7, 0.5, 0.5, 1.0];
const MIN_ZOOM: f32 = 1.0;
const MAX_ZOOM: f32 = 100.0;
const ROTATE_SENSITIVITY: f64 = 0.01;
const ZOOM_SENSITIVITY: f32 = 0.5;
const MAX_PITCH: f64 = std::f64::consts::FRAC_PI_2 - 0.001;
/// 隣接2三角形の法線がこの角度以上開いているエッジを sharp edge として線描画する
const EDGE_ANGLE_THRESHOLD_DEG: f32 = 25.0;

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub enum SceneMessage {
    /// マウスドラッグによる回転 (ピクセル差分)。update 側で Scene.camera に適用する。
    Orbit { dx: f64, dy: f64 },
    /// ホイールによるズーム差分。
    Zoom { delta: f32 },
    /// UV coordinates + widget aspect ratio (カメラは Scene.camera から読む)
    Clicked { u: f32, v: f32, aspect: f32 },
}

pub struct Scene {
    pub id: u64,
    pub color: [f32; 4],
    pub bbox_min: Vec3,
    pub bbox_max: Vec3,
    /// false: 注視点は原点 / true: bbox 中心
    pub view_at_object_center: bool,
    pub mesh: Arc<MeshData>,
    /// 0 = まだメッシュが設定されていない
    pub mesh_version: u64,
    /// カメラは widget state ではなく Model 側 (ここ) に持つ。
    /// widget state だとワークスペースの並び替えで表示位置に取り残される。
    pub camera: Camera,
}

#[derive(Debug, Clone, Copy)]
pub struct Camera {
    pub rotate_x: f64,
    pub rotate_y: f64,
    pub zoom: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            rotate_x: 0.0,
            rotate_y: 0.0,
            zoom: 10.0,
        }
    }
}

/// shader widget 内部 state。ドラッグ追跡のみでカメラ値は持たない。
#[derive(Default)]
pub struct DragState {
    dragging: bool,
    last_cursor: Option<Point>,
}

impl Drop for Scene {
    fn drop(&mut self) {
        if let Ok(mut q) = PENDING_REMOVALS.lock() {
            q.push(self.id);
        }
    }
}

impl Scene {
    pub fn new() -> Self {
        Self {
            id: NEXT_SCENE_ID.fetch_add(1, Ordering::Relaxed),
            color: DEFAULT_COLOR,
            bbox_min: Vec3::ZERO,
            bbox_max: Vec3::ZERO,
            view_at_object_center: false,
            mesh: Arc::new(MeshData {
                vertices: vec![],
                indices: vec![],
                edge_indices: vec![],
            }),
            mesh_version: 0,
            camera: Camera::default(),
        }
    }

    pub fn orbit(&mut self, dx: f64, dy: f64) {
        self.camera.rotate_y += dx * ROTATE_SENSITIVITY;
        self.camera.rotate_x =
            (self.camera.rotate_x + dy * ROTATE_SENSITIVITY).clamp(-MAX_PITCH, MAX_PITCH);
    }

    pub fn zoom_by(&mut self, delta: f32) {
        self.camera.zoom = (self.camera.zoom + delta * ZOOM_SENSITIVITY).clamp(MIN_ZOOM, MAX_ZOOM);
    }

    pub fn set_mesh(&mut self, vertices: Vec<Vertex>, indices: Vec<u32>) {
        self.update_bbox(&vertices);
        let edge_indices = extract_sharp_edges(&vertices, &indices, EDGE_ANGLE_THRESHOLD_DEG);
        self.mesh = Arc::new(MeshData {
            vertices,
            indices,
            edge_indices,
        });
        self.mesh_version = NEXT_MESH_VERSION.fetch_add(1, Ordering::Relaxed);
    }

    /// control points を球体としてメインメッシュに追加してから書き戻す。
    /// 旧 GUI の `set_mesh_with_control_points` の再復活。CP の色は `selected_cp`
    /// で 1 つだけ強調表示する。
    pub fn set_mesh_with_control_points(
        &mut self,
        mut vertices: Vec<Vertex>,
        mut indices: Vec<u32>,
        control_points: &[(String, [f64; 3])],
        selected_cp: Option<usize>,
    ) {
        self.update_bbox(&vertices);
        let edge_indices = extract_sharp_edges(&vertices, &indices, EDGE_ANGLE_THRESHOLD_DEG);
        let cp_radius = (self.aabb_max_abs() * 0.03).max(0.5);
        for (i, (_, pos)) in control_points.iter().enumerate() {
            let color = if Some(i) == selected_cp {
                [0.0, 1.0, 0.5, 1.0]
            } else {
                [1.0, 0.9, 0.0, 1.0]
            };
            let center = [pos[0] as f32, pos[1] as f32, pos[2] as f32];
            append_sphere(&mut vertices, &mut indices, center, cp_radius, color, 8);
        }
        self.mesh = Arc::new(MeshData {
            vertices,
            indices,
            edge_indices,
        });
        self.mesh_version = NEXT_MESH_VERSION.fetch_add(1, Ordering::Relaxed);
    }

    fn aabb_max_abs(&self) -> f32 {
        let v = self.bbox_min.abs().max(self.bbox_max.abs());
        v.x.max(v.y).max(v.z)
    }

    fn update_bbox(&mut self, vertices: &[Vertex]) {
        if vertices.is_empty() {
            self.bbox_min = Vec3::ZERO;
            self.bbox_max = Vec3::ZERO;
            return;
        }
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);
        for v in vertices {
            let p = Vec3::from_array(v.position);
            min = min.min(p);
            max = max.max(p);
        }
        self.bbox_min = min;
        self.bbox_max = max;
    }

    /// 現在の表示モードに応じた注視点 (camera target)
    pub fn view_center(&self) -> Vec3 {
        if self.view_at_object_center {
            (self.bbox_min + self.bbox_max) * 0.5
        } else {
            Vec3::ZERO
        }
    }

    /// 現在の表示モードに応じた基準カメラ距離。
    /// 原点モード: 原点からの最大距離 / 中心モード: bbox 半径の最大成分
    pub fn base_camera_distance(&self) -> f32 {
        let extent = if self.view_at_object_center {
            (self.bbox_max - self.bbox_min) * 0.5
        } else {
            self.bbox_min.abs().max(self.bbox_max.abs())
        };
        let max = extent.x.max(extent.y).max(extent.z);
        (max * 2.4 * 3.0).max(5.0)
    }

    fn build_uniforms(&self, bounds: Rectangle) -> Uniforms {
        let aspect = (bounds.width / bounds.height.max(1.0)).max(0.01);

        let cam = &self.camera;
        let rx = cam.rotate_x as f32;
        let ry = cam.rotate_y as f32;
        let dist = self.base_camera_distance() * (20.0 / cam.zoom);

        // near/far を dist に比例させる: そうしないと大きなモデルで far クリップに全部消える
        let near = (dist * 0.01).max(0.1);
        let far = (dist * 10.0).max(1000.0);
        let proj = Mat4::perspective_rh(45.0_f32.to_radians(), aspect, near, far);

        let center = self.view_center();
        let x = dist * ry.sin() * rx.cos();
        let y = dist * ry.cos() * rx.cos();
        let z = dist * rx.sin();
        let eye = center + Vec3::new(x, y, z);
        let view = Mat4::look_at_rh(eye, center, Vec3::Z);

        Uniforms {
            view_proj: (proj * view).to_cols_array_2d(),
            color: self.color,
            light_dir: DEFAULT_LIGHT_DIR,
            edge_color: DEFAULT_EDGE_COLOR,
        }
    }

    /// 軸ジゾモ用の uniform。メインカメラの「回転だけ」を再利用し、
    /// 平行投影で常に同じ画面サイズで描画する (zoom/モデル位置の影響を受けない)。
    /// 描画先は固定サイズの正方形サブビューポートのため aspect は 1.0 固定。
    fn build_gizmo_uniforms(&self) -> Uniforms {
        let rx = self.camera.rotate_x as f32;
        let ry = self.camera.rotate_y as f32;
        // 軸ベクトル長 1.0 がジゾモ枠の約 2/3 を占めるように [-1.5, 1.5] に張る
        let dist = 5.0_f32;
        let x = dist * ry.sin() * rx.cos();
        let y = dist * ry.cos() * rx.cos();
        let z = dist * rx.sin();
        let eye = Vec3::new(x, y, z);
        let view = Mat4::look_at_rh(eye, Vec3::ZERO, Vec3::Z);
        let proj = Mat4::orthographic_rh(-1.5, 1.5, -1.5, 1.5, 0.1, 10.0);

        Uniforms {
            view_proj: (proj * view).to_cols_array_2d(),
            color: [0.0; 4],
            light_dir: [0.0; 4],
            edge_color: [0.0; 4],
        }
    }
}

/// Generate ray from UV coordinates (0..1) through the camera.
/// Returns (origin, direction) in world space.
#[allow(dead_code)]
pub fn generate_ray_from_uv(
    u: f32,
    v: f32,
    cam: &Camera,
    base_camera_distance: f32,
    aspect: f32,
    view_center: Vec3,
) -> ([f64; 3], [f64; 3]) {
    let rx = cam.rotate_x as f32;
    let ry = cam.rotate_y as f32;
    let dist = base_camera_distance * (20.0 / cam.zoom);

    let x = dist * ry.sin() * rx.cos();
    let y = dist * ry.cos() * rx.cos();
    let z = dist * rx.sin();
    let eye = view_center + Vec3::new(x, y, z);

    let fov_y = 45.0_f32.to_radians();
    let half_h = (fov_y / 2.0).tan();
    let half_w = half_h * aspect;

    // NDC: [-1,1]
    let ndc_x = u * 2.0 - 1.0;
    let ndc_y = 1.0 - v * 2.0; // flip Y

    let forward = (view_center - eye).normalize();
    let right = forward.cross(Vec3::Z).normalize();
    let up = right.cross(forward).normalize();

    let dir = (forward + right * (ndc_x * half_w) + up * (ndc_y * half_h)).normalize();

    (
        [eye.x as f64, eye.y as f64, eye.z as f64],
        [dir.x as f64, dir.y as f64, dir.z as f64],
    )
}

/// Ray-triangle intersection (Möller-Trumbore)。
/// 前方 (t > 0) の hit 距離を返す。
#[allow(dead_code)]
pub fn ray_triangle_intersect(
    origin: &[f64; 3],
    dir: &[f64; 3],
    v0: &[f64; 3],
    v1: &[f64; 3],
    v2: &[f64; 3],
) -> Option<f64> {
    let eps = 1e-9;
    let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
    let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
    let p = [
        dir[1] * e2[2] - dir[2] * e2[1],
        dir[2] * e2[0] - dir[0] * e2[2],
        dir[0] * e2[1] - dir[1] * e2[0],
    ];
    let det = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];
    if det.abs() < eps {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = [origin[0] - v0[0], origin[1] - v0[1], origin[2] - v0[2]];
    let u = (s[0] * p[0] + s[1] * p[1] + s[2] * p[2]) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = [
        s[1] * e1[2] - s[2] * e1[1],
        s[2] * e1[0] - s[0] * e1[2],
        s[0] * e1[1] - s[1] * e1[0],
    ];
    let v = (dir[0] * q[0] + dir[1] * q[1] + dir[2] * q[2]) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * inv_det;
    if t > eps { Some(t) } else { None }
}

/// mesh 全体に対して ray-triangle intersection を回し、最も近い hit point (world 座標) を返す。
/// `vertices` は Vertex 配列、`indices` は 3n 個の u32 (三角形リスト)。
#[allow(dead_code)]
pub fn ray_mesh_intersect(
    origin: &[f64; 3],
    dir: &[f64; 3],
    vertices: &[pipeline::Vertex],
    indices: &[u32],
) -> Option<[f64; 3]> {
    let mut best_t: Option<f64> = None;
    for tri in indices.chunks_exact(3) {
        let v0p = &vertices[tri[0] as usize].position;
        let v1p = &vertices[tri[1] as usize].position;
        let v2p = &vertices[tri[2] as usize].position;
        let v0 = [v0p[0] as f64, v0p[1] as f64, v0p[2] as f64];
        let v1 = [v1p[0] as f64, v1p[1] as f64, v1p[2] as f64];
        let v2 = [v2p[0] as f64, v2p[1] as f64, v2p[2] as f64];
        if let Some(t) = ray_triangle_intersect(origin, dir, &v0, &v1, &v2) {
            if best_t.is_none_or(|b| t < b) {
                best_t = Some(t);
            }
        }
    }
    best_t.map(|t| [origin[0] + t * dir[0], origin[1] + t * dir[1], origin[2] + t * dir[2]])
}

/// Ray-sphere intersection, returns distance t or None.
#[allow(dead_code)]
pub fn ray_sphere_intersect(
    origin: &[f64; 3],
    dir: &[f64; 3],
    center: &[f64; 3],
    radius: f64,
) -> Option<f64> {
    let oc = [
        origin[0] - center[0],
        origin[1] - center[1],
        origin[2] - center[2],
    ];
    let a = dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2];
    let b = 2.0 * (oc[0] * dir[0] + oc[1] * dir[1] + oc[2] * dir[2]);
    let c = oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2] - radius * radius;
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }
    let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
    let t2 = (-b + discriminant.sqrt()) / (2.0 * a);
    let t = if t1 > 0.0 { t1 } else { t2 };
    if t > 0.0 { Some(t) } else { None }
}

fn append_sphere(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    center: [f32; 3],
    radius: f32,
    color: [f32; 4],
    segments: usize,
) {
    let base_idx = vertices.len() as u32;
    let stacks = segments;
    let slices = segments * 2;
    for i in 0..=stacks {
        let phi = std::f32::consts::PI * i as f32 / stacks as f32;
        for j in 0..=slices {
            let theta = 2.0 * std::f32::consts::PI * j as f32 / slices as f32;
            let nx = phi.sin() * theta.cos();
            let ny = phi.sin() * theta.sin();
            let nz = phi.cos();
            vertices.push(Vertex {
                position: [
                    center[0] + radius * nx,
                    center[1] + radius * ny,
                    center[2] + radius * nz,
                ],
                normal: [nx, ny, nz],
                color,
            });
        }
    }
    for i in 0..stacks {
        for j in 0..slices {
            let row1 = base_idx + (i * (slices + 1)) as u32;
            let row2 = base_idx + ((i + 1) * (slices + 1)) as u32;
            let a = row1 + j as u32;
            let b = row2 + j as u32;
            let c = row2 + (j + 1) as u32;
            let d = row1 + (j + 1) as u32;
            indices.extend_from_slice(&[a, b, c, a, c, d]);
        }
    }
}

/// 同位置にある別 vertex (sharp edge で分離された頂点) をマージしてエッジ隣接情報を取り、
/// 隣接2三角形の法線角度差が閾値超のエッジだけ LineList 用 index として返す。
/// 境界エッジ (open mesh) も sharp 扱い。
fn extract_sharp_edges(vertices: &[Vertex], indices: &[u32], threshold_deg: f32) -> Vec<u32> {
    use std::collections::HashMap;

    if indices.len() < 3 {
        return Vec::new();
    }

    let mut pos_map: HashMap<(u32, u32, u32), u32> = HashMap::new();
    let mut canonical: Vec<u32> = Vec::with_capacity(vertices.len());
    let mut canonical_to_original: HashMap<u32, u32> = HashMap::new();
    for (orig_idx, v) in vertices.iter().enumerate() {
        let key = (
            v.position[0].to_bits(),
            v.position[1].to_bits(),
            v.position[2].to_bits(),
        );
        let next_id = pos_map.len() as u32;
        let canon = *pos_map.entry(key).or_insert(next_id);
        canonical.push(canon);
        canonical_to_original
            .entry(canon)
            .or_insert(orig_idx as u32);
    }

    let tri_count = indices.len() / 3;
    let mut tri_normals: Vec<Vec3> = Vec::with_capacity(tri_count);
    for tri in indices.chunks_exact(3) {
        let p0 = Vec3::from_array(vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(vertices[tri[2] as usize].position);
        tri_normals.push((p1 - p0).cross(p2 - p0).normalize_or_zero());
    }

    let mut edge_tris: std::collections::HashMap<(u32, u32), Vec<usize>> =
        std::collections::HashMap::new();
    for (ti, tri) in indices.chunks_exact(3).enumerate() {
        let a = canonical[tri[0] as usize];
        let b = canonical[tri[1] as usize];
        let c = canonical[tri[2] as usize];
        for (e0, e1) in [(a, b), (b, c), (c, a)] {
            let key = if e0 < e1 { (e0, e1) } else { (e1, e0) };
            edge_tris.entry(key).or_default().push(ti);
        }
    }

    let cos_thresh = threshold_deg.to_radians().cos();
    let mut line_indices: Vec<u32> = Vec::new();
    for (edge, tris) in edge_tris {
        let is_sharp = match tris.len() {
            1 => true,
            2 => tri_normals[tris[0]].dot(tri_normals[tris[1]]) < cos_thresh,
            _ => true,
        };
        if !is_sharp {
            continue;
        }
        let (Some(&a), Some(&b)) = (
            canonical_to_original.get(&edge.0),
            canonical_to_original.get(&edge.1),
        ) else {
            continue;
        };
        line_indices.push(a);
        line_indices.push(b);
    }

    line_indices
}

impl shader::Program<SceneMessage> for Scene {
    type State = DragState;
    type Primitive = Primitive;

    fn update(
        &self,
        state: &mut Self::State,
        event: &Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<shader::Action<SceneMessage>> {
        let in_bounds = cursor.is_over(bounds);

        match event {
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) if in_bounds => {
                state.dragging = true;
                state.last_cursor = cursor.position();
                Some(shader::Action::capture())
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                let was_dragging = state.dragging;
                state.dragging = false;

                if was_dragging && in_bounds {
                    if let Some(pos) = cursor.position() {
                        let u = (pos.x - bounds.x) / bounds.width;
                        let v = (pos.y - bounds.y) / bounds.height;
                        let aspect = bounds.width / bounds.height.max(1.0);
                        state.last_cursor = None;
                        return Some(
                            shader::Action::publish(SceneMessage::Clicked { u, v, aspect })
                                .and_capture(),
                        );
                    }
                }
                state.last_cursor = None;
                Some(shader::Action::capture())
            }
            Event::Mouse(mouse::Event::CursorMoved { position }) if state.dragging => {
                let action = if let Some(last) = state.last_cursor {
                    let dx = (position.x - last.x) as f64;
                    let dy = (position.y - last.y) as f64;
                    shader::Action::publish(SceneMessage::Orbit { dx, dy }).and_capture()
                } else {
                    shader::Action::capture()
                };
                state.last_cursor = Some(*position);
                Some(action)
            }
            Event::Mouse(mouse::Event::WheelScrolled { delta }) if in_bounds => {
                let scroll = match delta {
                    mouse::ScrollDelta::Lines { y, .. } => *y,
                    mouse::ScrollDelta::Pixels { y, .. } => y / 50.0,
                };
                Some(shader::Action::publish(SceneMessage::Zoom { delta: scroll }).and_capture())
            }
            _ => None,
        }
    }

    fn draw(
        &self,
        _state: &Self::State,
        _cursor: mouse::Cursor,
        bounds: Rectangle,
    ) -> Self::Primitive {
        Primitive {
            id: self.id,
            uniforms: self.build_uniforms(bounds),
            gizmo_uniforms: self.build_gizmo_uniforms(),
            mesh: self.mesh.clone(),
            mesh_version: self.mesh_version,
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if state.dragging {
            mouse::Interaction::Grabbing
        } else if cursor.is_over(bounds) {
            mouse::Interaction::Grab
        } else {
            mouse::Interaction::default()
        }
    }
}

#[derive(Debug)]
pub struct Primitive {
    id: u64,
    uniforms: Uniforms,
    gizmo_uniforms: Uniforms,
    mesh: Arc<MeshData>,
    mesh_version: u64,
}

impl shader::Primitive for Primitive {
    type Pipeline = Pipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        let drained: Vec<u64> = PENDING_REMOVALS
            .lock()
            .map(|mut q| std::mem::take(&mut *q))
            .unwrap_or_default();
        for id in drained {
            pipeline.remove_instance(id);
        }
        pipeline.update_instance(
            device,
            queue,
            viewport,
            *bounds * viewport.scale_factor() as f32,
            self.id,
            &self.uniforms,
            &self.gizmo_uniforms,
            &self.mesh,
            self.mesh_version,
        );
    }

    fn render(
        &self,
        pipeline: &Self::Pipeline,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        clip_bounds: &Rectangle<u32>,
    ) {
        pipeline.render_instance(encoder, target, clip_bounds, self.id);
    }
}
