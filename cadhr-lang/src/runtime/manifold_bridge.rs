//! `Model3D` (宣言ツリー) → `manifold-csg` の `Manifold` に評価する橋渡し。
//!
//! evaluator (`runtime::eval`) は副作用なく `Model3D` を組み立てるだけ。実際の
//! CSG 計算 / メッシュ生成はここで manifold-csg に投げる。GUI 側はここから
//! `Vertex` / `index` を受け取って iced shader に流す。
//!
//! manifold-csg 自体は cargo feature `manifold` の有無で gating する。feature OFF の
//! ビルドではこのモジュール全体が disable される。

#![cfg(feature = "manifold")]

use crate::runtime::value::{Model2D, Model3D, Plane3D};
use manifold_csg::{CrossSection, Manifold};
use std::path::{Path as StdPath, PathBuf};

const DEFAULT_SEGMENTS: i32 = 64;

#[derive(Debug, Clone)]
pub enum BridgeError {
    Stl(String),
    InvalidShape(String),
}

impl std::fmt::Display for BridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BridgeError::Stl(s) => write!(f, "STL 読み込みエラー: {s}"),
            BridgeError::InvalidShape(s) => write!(f, "形状エラー: {s}"),
        }
    }
}

impl std::error::Error for BridgeError {}

/// `Model3D` を `Manifold` に評価する。引数 `include_paths` は STL ファイル探索に使う。
pub fn evaluate_with_paths(
    model: &Model3D,
    include_paths: &[PathBuf],
) -> Result<Manifold, BridgeError> {
    match model {
        Model3D::Empty => Ok(Manifold::empty()),
        Model3D::Cube { x, y, z } => Ok(Manifold::cube(*x, *y, *z, false)),
        Model3D::Sphere(r) => Ok(Manifold::sphere(*r, DEFAULT_SEGMENTS)),
        Model3D::Cylinder { r, h } => Ok(Manifold::cylinder(*h, *r, *r, DEFAULT_SEGMENTS, false)),
        Model3D::Tetrahedron => Ok(Manifold::tetrahedron()),

        Model3D::Union(a, b) => {
            Ok(evaluate_with_paths(a, include_paths)?
                .union(&evaluate_with_paths(b, include_paths)?))
        }
        Model3D::Diff(a, b) => Ok(evaluate_with_paths(a, include_paths)?
            .difference(&evaluate_with_paths(b, include_paths)?)),
        Model3D::Intersect(a, b) => Ok(evaluate_with_paths(a, include_paths)?
            .intersection(&evaluate_with_paths(b, include_paths)?)),
        Model3D::Hull(a, b) => Ok(evaluate_with_paths(a, include_paths)?
            .union(&evaluate_with_paths(b, include_paths)?)
            .hull()),

        Model3D::Translate { shape, src, dst } => {
            let dx = dst.0 - src.0;
            let dy = dst.1 - src.1;
            let dz = dst.2 - src.2;
            Ok(evaluate_with_paths(shape, include_paths)?.translate(dx, dy, dz))
        }
        Model3D::Scale { shape, factor } => {
            Ok(evaluate_with_paths(shape, include_paths)?.scale(factor.0, factor.1, factor.2))
        }
        Model3D::Rotate { shape, angles } => {
            Ok(evaluate_with_paths(shape, include_paths)?.rotate(angles.0, angles.1, angles.2))
        }
        Model3D::LinearExtrude {
            profile,
            plane,
            height,
        } => extrude_polygon(profile, *plane, *height, 0, 0.0, 1.0, 1.0),
        Model3D::ComplexExtrude {
            profile,
            plane,
            height,
            twist,
            scale_x,
            scale_y,
        } => {
            let n_div = height.abs().max(1.0) as u32;
            extrude_polygon(profile, *plane, *height, n_div, *twist, *scale_x, *scale_y)
        }
        Model3D::Revolve {
            profile,
            plane,
            degrees,
        } => revolve_polygon(profile, *plane, *degrees),
        Model3D::Stl { path } => load_stl(path, include_paths),
        Model3D::SweepExtrude {
            profile,
            plane,
            path,
        } => sweep_polygon(profile, *plane, path),
    }
}

/// `include_paths = &[]` で呼ぶラッパ。STL を使わない場合はこれで十分。
pub fn evaluate(model: &Model3D) -> Result<Manifold, BridgeError> {
    evaluate_with_paths(model, &[])
}

/// Shape3D の AABB 中心を計算。`center3d` builtin の本体。
pub fn bbox_center_3d(
    model: &Model3D,
    include_paths: &[PathBuf],
) -> Result<(f64, f64, f64), BridgeError> {
    let m = evaluate_with_paths(model, include_paths)?;
    let bb = m
        .bounding_box()
        .ok_or_else(|| BridgeError::InvalidShape("bbox_center_3d: 空の Shape3D".to_string()))?;
    let [cx, cy, cz] = bb.center();
    Ok((cx, cy, cz))
}

/// Shape2D の AABB 中心を計算。`center2d` builtin の本体。
pub fn bbox_center_2d(model: &Model2D) -> Result<(f64, f64), BridgeError> {
    let cs = to_cross_section(model)
        .filter(|cs| !cs.is_empty())
        .ok_or_else(|| BridgeError::InvalidShape("bbox_center_2d: 空の Shape2D".to_string()))?;
    let bounds = cs.bounds();
    let [min_x, min_y] = bounds.min();
    let [max_x, max_y] = bounds.max();
    Ok(((min_x + max_x) / 2.0, (min_y + max_y) / 2.0))
}

/// `Model2D` を評価して輪郭 polygon 群を返す。穴は別 contour (逆巻き) として
/// 含まれる。空形状は空 Vec。GUI の 2D sketch が参照表示に使う。
pub fn shape2d_contours(profile: &Model2D) -> Vec<Vec<[f64; 2]>> {
    to_cross_section(profile)
        .map(|cs| cs.to_polygons())
        .unwrap_or_default()
}

/// `Model2D` を `CrossSection` (Clipper2 ベースの 2D 領域) に評価する。
/// CSG ノードは manifold-csg のネイティブ 2D boolean をそのまま使う。
fn to_cross_section(profile: &Model2D) -> Option<CrossSection> {
    match profile {
        Model2D::Polygon(points) if !points.is_empty() => {
            // from_simple_polygon は FillRule::Positive なので CCW を保証する必要がある。
            let mut pts: Vec<[f64; 2]> = points.iter().map(|&(x, y)| [x, y]).collect();
            ensure_ccw(&mut pts);
            Some(CrossSection::from_simple_polygon(&pts))
        }
        Model2D::Empty2D | Model2D::Polygon(_) => None,
        Model2D::Union2D(a, b) => match (to_cross_section(a), to_cross_section(b)) {
            (Some(ca), Some(cb)) => Some(ca.union(&cb)),
            (Some(c), None) | (None, Some(c)) => Some(c),
            (None, None) => None,
        },
        Model2D::Diff2D(a, b) => match (to_cross_section(a), to_cross_section(b)) {
            (Some(ca), Some(cb)) => Some(ca.difference(&cb)),
            (Some(c), None) => Some(c),
            _ => None,
        },
        Model2D::Intersect2D(a, b) => match (to_cross_section(a), to_cross_section(b)) {
            (Some(ca), Some(cb)) => Some(ca.intersection(&cb)),
            _ => None,
        },
        Model2D::Translate2D { shape, src, dst } => {
            let cs = to_cross_section(shape)?;
            Some(cs.translate(dst.0 - src.0, dst.1 - src.1))
        }
    }
}

/// extrude/revolve 前の平面合わせ。XZ 押し出しは Y 反転 + ensure_ccw が必要
/// (apply_plane_rotation の rotate(-90,0,0) との辻褄合わせ)。
fn prep_cross_section(cs: CrossSection, plane: Plane3D) -> CrossSection {
    if plane != Plane3D::XZ {
        return cs;
    }
    let flipped: Vec<Vec<[f64; 2]>> = cs
        .to_polygons()
        .into_iter()
        .map(|ring| {
            let mut pts: Vec<[f64; 2]> = ring.into_iter().map(|[x, y]| [x, -y]).collect();
            ensure_ccw(&mut pts);
            pts
        })
        .collect();
    CrossSection::from_polygons(&flipped)
}

fn apply_plane_rotation(m: Manifold, plane: Plane3D) -> Manifold {
    match plane {
        Plane3D::XY => m,
        Plane3D::YZ => m.rotate(90.0, 0.0, 90.0),
        Plane3D::XZ => m.rotate(-90.0, 0.0, 0.0),
    }
}

fn extrude_polygon(
    profile: &Model2D,
    plane: Plane3D,
    height: f64,
    slices: u32,
    twist: f64,
    sx: f64,
    sy: f64,
) -> Result<Manifold, BridgeError> {
    let cs = match to_cross_section(profile) {
        Some(cs) if !cs.is_empty() => prep_cross_section(cs, plane),
        _ => return Ok(Manifold::empty()),
    };
    let m = Manifold::extrude_with_options(&cs, height, slices as i32, twist, sx, sy);
    Ok(apply_plane_rotation(m, plane))
}

fn revolve_polygon(
    profile: &Model2D,
    plane: Plane3D,
    degrees: f64,
) -> Result<Manifold, BridgeError> {
    let cs = match to_cross_section(profile) {
        Some(cs) if !cs.is_empty() => prep_cross_section(cs, plane),
        _ => return Ok(Manifold::empty()),
    };
    let m = Manifold::revolve(&cs, DEFAULT_SEGMENTS, degrees);
    Ok(apply_plane_rotation(m, plane))
}

fn load_stl(path: &str, include_paths: &[PathBuf]) -> Result<Manifold, BridgeError> {
    let raw = StdPath::new(path);
    let resolved = if raw.is_absolute() {
        PathBuf::from(path)
    } else {
        include_paths
            .iter()
            .map(|dir| dir.join(raw))
            .find(|p| p.exists())
            .unwrap_or_else(|| PathBuf::from(path))
    };
    let mut file = std::fs::OpenOptions::new()
        .read(true)
        .open(&resolved)
        .map_err(|e| BridgeError::Stl(format!("{}: {}", resolved.display(), e)))?;
    let stl = stl_io::read_stl(&mut file)
        .map_err(|e| BridgeError::Stl(format!("{}: {}", resolved.display(), e)))?;
    let verts: Vec<f32> = stl
        .vertices
        .iter()
        .flat_map(|v| [v[0], v[1], v[2]])
        .collect();
    let indices: Vec<u32> = stl
        .faces
        .iter()
        .flat_map(|f| f.vertices.iter().map(|&i| i as u32))
        .collect();
    Manifold::from_mesh_f32(&verts, 3, &indices)
        .map_err(|e| BridgeError::Stl(format!("{}: manifold 化失敗: {e}", resolved.display())))
}

fn sweep_polygon(
    profile: &Model2D,
    plane: Plane3D,
    path: &[(f64, f64, f64)],
) -> Result<Manifold, BridgeError> {
    let cs = to_cross_section(profile)
        .filter(|cs| !cs.is_empty())
        .ok_or_else(|| BridgeError::InvalidShape("sweep_extrude: profile が空".to_string()))?;
    let contours = cs.to_polygons();
    let first = contours
        .first()
        .ok_or_else(|| BridgeError::InvalidShape("sweep_extrude: profile が空".to_string()))?;
    let profile_pairs: Vec<(f64, f64)> = first.iter().map(|&[x, y]| (x, y)).collect();
    let (verts, indices) = sweep_mesh(&profile_pairs, path)?;
    let m = Manifold::from_mesh_f32(&verts, 3, &indices)
        .map_err(|e| BridgeError::InvalidShape(format!("sweep_extrude: manifold 化失敗: {e}")))?;
    Ok(apply_plane_rotation(m, plane))
}

/// 3D path に沿った sweep extrude。`profile` は XY 平面上の閉路ポリゴンの (x, y) ペア列で、
/// 各 path 点で構築する rotation minimizing frame の (N, B) 平面に展開する。
///
/// フレームの定義:
///   - T (tangent): path の進行方向
///   - N: profile.x が向く軸 (初期は reference up = world Z から T 直交成分を取る。
///        T が Z にほぼ平行なときは world Y にフォールバック)
///   - B = N × T: profile.y が向く軸 (これを使うと start cap (center, j, j_next) /
///                end cap (center, j_next, j) の winding が outward 向きになる)
///   - 2 点目以降の N は前点 N を `T_prev → T_cur` の最小回転で並進輸送して求める
fn sweep_mesh(
    profile: &[(f64, f64)],
    path: &[(f64, f64, f64)],
) -> Result<(Vec<f32>, Vec<u32>), BridgeError> {
    let n_profile = profile.len();
    if n_profile < 3 {
        return Err(BridgeError::InvalidShape(
            "sweep_extrude: profile 頂点 < 3".to_string(),
        ));
    }
    // 連続する重複点を除去 (退化したセグメントで tangent 計算が壊れるのを防ぐ)
    let mut clean: Vec<[f64; 3]> = Vec::with_capacity(path.len());
    for p in path {
        let v = [p.0, p.1, p.2];
        if let Some(last) = clean.last() {
            let dx = v[0] - last[0];
            let dy = v[1] - last[1];
            let dz = v[2] - last[2];
            if dx * dx + dy * dy + dz * dz < 1e-24 {
                continue;
            }
        }
        clean.push(v);
    }
    let n_path = clean.len();
    if n_path < 2 {
        return Err(BridgeError::InvalidShape(
            "sweep_extrude: 退化を除いた path 頂点が 2 個未満".to_string(),
        ));
    }
    // 各 path 点での tangent。
    //   - 端点: 隣接 segment の単位方向
    //   - 内点: incoming/outgoing **単位** tangent の和を正規化 (miter 角の二等分線)
    //
    // 内点で centered diff (P_{i+1}-P_{i-1}) を使うと、segment の長さに依存して
    // 真の bisector からズレ、cross-section が片側 segment の法線平面に整合せず
    // side wall がねじれた潰れた楕円状に見えてしまうため、bisector を採用する。
    let mut tangents: Vec<[f64; 3]> = Vec::with_capacity(n_path);
    for i in 0..n_path {
        let t = if i == 0 {
            normalize3([
                clean[1][0] - clean[0][0],
                clean[1][1] - clean[0][1],
                clean[1][2] - clean[0][2],
            ])
        } else if i == n_path - 1 {
            normalize3([
                clean[i][0] - clean[i - 1][0],
                clean[i][1] - clean[i - 1][1],
                clean[i][2] - clean[i - 1][2],
            ])
        } else {
            let t_in = normalize3([
                clean[i][0] - clean[i - 1][0],
                clean[i][1] - clean[i - 1][1],
                clean[i][2] - clean[i - 1][2],
            ]);
            let t_out = normalize3([
                clean[i + 1][0] - clean[i][0],
                clean[i + 1][1] - clean[i][1],
                clean[i + 1][2] - clean[i][2],
            ]);
            let sum = [t_in[0] + t_out[0], t_in[1] + t_out[1], t_in[2] + t_out[2]];
            if norm3(sum) < 1e-9 {
                // 180° fold-back: bisector が決まらない。仕様未定なので panic させる。
                // TODO: 180° fold-back の解釈 (二点間で path を切る? error にする?) を決める。
                panic!(
                    "sweep_extrude: path 点 {i} で 180° 折り返しが発生し miter bisector が計算できない",
                );
            }
            normalize3(sum)
        };
        tangents.push(t);
    }
    // 初期フレーム + parallel transport
    let (n0, b0) = initial_frame(tangents[0]);
    let mut frames: Vec<([f64; 3], [f64; 3])> = Vec::with_capacity(n_path);
    frames.push((n0, b0));
    for i in 1..n_path {
        let (n_prev, _) = frames[i - 1];
        let n_cur = parallel_transport(tangents[i - 1], tangents[i], n_prev);
        // 数値誤差で N が T 直交から少しずれるので再直交化
        let dot_nt = dot3(n_cur, tangents[i]);
        let n_cur = normalize3([
            n_cur[0] - dot_nt * tangents[i][0],
            n_cur[1] - dot_nt * tangents[i][1],
            n_cur[2] - dot_nt * tangents[i][2],
        ]);
        let b_cur = cross3(n_cur, tangents[i]);
        frames.push((n_cur, b_cur));
    }
    // 頂点生成
    let mut vertices: Vec<f32> = Vec::with_capacity((n_path * n_profile + 2) * 3);
    for i in 0..n_path {
        let p = clean[i];
        let (n, b) = frames[i];
        for &(lx, ly) in profile {
            vertices.push((p[0] + lx * n[0] + ly * b[0]) as f32);
            vertices.push((p[1] + lx * n[1] + ly * b[1]) as f32);
            vertices.push((p[2] + lx * n[2] + ly * b[2]) as f32);
        }
    }
    let mut indices: Vec<u32> = Vec::with_capacity((n_path - 1) * n_profile * 6);
    for i in 0..(n_path - 1) {
        for j in 0..n_profile {
            let j_next = (j + 1) % n_profile;
            let c0 = (i * n_profile + j) as u32;
            let c1 = (i * n_profile + j_next) as u32;
            let n0 = ((i + 1) * n_profile + j) as u32;
            let n1 = ((i + 1) * n_profile + j_next) as u32;
            indices.extend_from_slice(&[c0, n0, c1, c1, n0, n1]);
        }
    }
    // start cap: fan triangulation with a center vertex.
    let start_center_idx = (vertices.len() / 3) as u32;
    let (cx, cy, cz) = ring_center(&vertices, 0, n_profile);
    vertices.extend_from_slice(&[cx, cy, cz]);
    for j in 0..n_profile as u32 {
        let j_next = (j + 1) % n_profile as u32;
        indices.extend_from_slice(&[start_center_idx, j, j_next]);
    }
    // end cap
    let end_center_idx = (vertices.len() / 3) as u32;
    let base = ((n_path - 1) * n_profile) as u32;
    let (cx, cy, cz) = ring_center(&vertices, ((n_path - 1) * n_profile) * 3, n_profile);
    vertices.extend_from_slice(&[cx, cy, cz]);
    for j in 0..n_profile as u32 {
        let j_next = (j + 1) % n_profile as u32;
        indices.extend_from_slice(&[end_center_idx, base + j_next, base + j]);
    }
    Ok((vertices, indices))
}

/// 初期フレーム (N, B)。reference up = world Z を tangent に直交化したものを N とする。
/// tangent が Z にほぼ平行なときだけ world Y にフォールバックする。
/// B = N × T で右手系の cap winding を outward 向きに固定する。
fn initial_frame(t: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let ref_up = if t[2].abs() > 0.95 {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let d = dot3(ref_up, t);
    let n = normalize3([
        ref_up[0] - d * t[0],
        ref_up[1] - d * t[1],
        ref_up[2] - d * t[2],
    ]);
    let b = cross3(n, t);
    (n, b)
}

/// Rodrigues の公式で `t_prev → t_cur` を回す最小回転を `n_prev` に適用する。
/// tangent が変わらない (sin_theta ≈ 0) 場合は n をそのまま返す。
fn parallel_transport(
    t_prev: [f64; 3],
    t_cur: [f64; 3],
    n_prev: [f64; 3],
) -> [f64; 3] {
    let axis_raw = cross3(t_prev, t_cur);
    let sin_theta = norm3(axis_raw);
    let cos_theta = dot3(t_prev, t_cur);
    if sin_theta < 1e-9 {
        return n_prev;
    }
    let axis = [
        axis_raw[0] / sin_theta,
        axis_raw[1] / sin_theta,
        axis_raw[2] / sin_theta,
    ];
    let cav = cross3(axis, n_prev);
    let dav = dot3(axis, n_prev);
    let one_minus_cos = 1.0 - cos_theta;
    [
        n_prev[0] * cos_theta + cav[0] * sin_theta + axis[0] * dav * one_minus_cos,
        n_prev[1] * cos_theta + cav[1] * sin_theta + axis[1] * dav * one_minus_cos,
        n_prev[2] * cos_theta + cav[2] * sin_theta + axis[2] * dav * one_minus_cos,
    ]
}

fn normalize3(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn ring_center(vertices: &[f32], base_offset_floats: usize, n_profile: usize) -> (f32, f32, f32) {
    let mut sx = 0.0f64;
    let mut sy = 0.0f64;
    let mut sz = 0.0f64;
    for k in 0..n_profile {
        let idx = base_offset_floats + k * 3;
        sx += vertices[idx] as f64;
        sy += vertices[idx + 1] as f64;
        sz += vertices[idx + 2] as f64;
    }
    let n = n_profile as f64;
    ((sx / n) as f32, (sy / n) as f32, (sz / n) as f32)
}

fn ensure_ccw(points: &mut Vec<[f64; 2]>) {
    if points.len() < 3 {
        return;
    }
    let mut signed_area = 0.0;
    for i in 0..points.len() {
        let [x1, y1] = points[i];
        let [x2, y2] = points[(i + 1) % points.len()];
        signed_area += x1 * y2 - x2 * y1;
    }
    if signed_area < 0.0 {
        points.reverse();
    }
}

/// 法線計算 + Mesh への落とし込み。`(positions[f32x3], normals[f32x3], indices[u32])` 形式で返す。
pub fn to_mesh_arrays(model: &Model3D) -> Result<MeshArrays, BridgeError> {
    to_mesh_arrays_with_paths(model, &[])
}

pub fn to_mesh_arrays_with_paths(
    model: &Model3D,
    include_paths: &[PathBuf],
) -> Result<MeshArrays, BridgeError> {
    let manifold = evaluate_with_paths(model, include_paths)?;
    let with_normals = manifold.calculate_normals(0, 30.0);
    let (verts, num_props, indices) = with_normals.to_mesh_f32();
    Ok(MeshArrays::from_mesh_data(&verts, num_props, &indices))
}

/// GUI に渡しやすい平坦化したメッシュ表現。
pub struct MeshArrays {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

impl MeshArrays {
    /// `vert_props` は頂点ごとに `num_props` 個の f32 (先頭 3 つが位置、`num_props >= 6` なら
    /// 4..6 が法線) が並んだ平坦バッファ。
    fn from_mesh_data(vert_props: &[f32], num_props: usize, indices: &[u32]) -> Self {
        let n_vertices = if num_props == 0 {
            0
        } else {
            vert_props.len() / num_props
        };
        let mut positions = Vec::with_capacity(n_vertices);
        let mut normals = Vec::with_capacity(n_vertices);
        for i in 0..n_vertices {
            let base = i * num_props;
            positions.push([vert_props[base], vert_props[base + 1], vert_props[base + 2]]);
            if num_props >= 6 {
                normals.push([
                    vert_props[base + 3],
                    vert_props[base + 4],
                    vert_props[base + 5],
                ]);
            } else {
                normals.push([0.0, 0.0, 1.0]);
            }
        }
        Self {
            positions,
            normals,
            indices: indices.to_vec(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.positions.is_empty() || self.indices.is_empty()
    }
}

/// 評価済み `Manifold` の AABB を返す。`center3d` builtin の実装で使う。
pub fn bbox_of(model: &Model3D) -> Option<((f64, f64, f64), (f64, f64, f64))> {
    let manifold = evaluate(model).ok()?;
    let bb = manifold.bounding_box()?;
    let [min_x, min_y, min_z] = bb.min();
    let [max_x, max_y, max_z] = bb.max();
    Some(((min_x, min_y, min_z), (max_x, max_y, max_z)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cube_to_mesh() {
        let m = Model3D::Cube {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let arrays = to_mesh_arrays(&m).unwrap();
        assert!(!arrays.is_empty());
        assert_eq!(arrays.positions.len(), arrays.normals.len());
        assert!(arrays.indices.len() % 3 == 0);
    }

    #[test]
    fn union_to_mesh() {
        let m = Model3D::Union(
            Box::new(Model3D::Cube {
                x: 2.0,
                y: 2.0,
                z: 2.0,
            }),
            Box::new(Model3D::Sphere(1.5)),
        );
        let arrays = to_mesh_arrays(&m).unwrap();
        assert!(!arrays.is_empty());
    }

    #[test]
    fn revolve_to_mesh() {
        let profile = Model2D::Polygon(vec![(1.0, 0.0), (3.0, 0.0), (3.0, 1.0), (1.0, 1.0)]);
        let m = Model3D::Revolve {
            profile,
            plane: Plane3D::XY,
            degrees: 360.0,
        };
        let arrays = to_mesh_arrays(&m).unwrap();
        assert!(!arrays.is_empty());
    }

    #[test]
    fn union_2d_extruded() {
        let a = Model2D::Polygon(vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]);
        let b = Model2D::Polygon(vec![(2.0, 2.0), (6.0, 2.0), (6.0, 6.0), (2.0, 6.0)]);
        let union = Model2D::Union2D(Box::new(a), Box::new(b));
        let m = Model3D::LinearExtrude {
            profile: union,
            plane: Plane3D::XY,
            height: 1.0,
        };
        let arrays = to_mesh_arrays(&m).unwrap();
        assert!(!arrays.is_empty());
    }
}
