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
    // path 3D → XZ 平面投影 (sweep.rs 互換)。XY/YZ も将来対応可能だが、まず XZ パスのみ。
    let path_xz: Vec<(f64, f64)> = path.iter().map(|p| (p.0, p.2)).collect();
    let (verts, indices) = sweep_mesh(&profile_pairs, &path_xz)?;
    let m = Manifold::from_mesh_f32(&verts, 3, &indices)
        .map_err(|e| BridgeError::InvalidShape(format!("sweep_extrude: manifold 化失敗: {e}")))?;
    Ok(apply_plane_rotation(m, plane))
}

/// 2D path に沿った sweep extrude。`profile` は閉路ポリゴンの (x, y) ペア列。
/// `path` 2D を XZ 平面上の 3D 曲線 (px, 0, pz) として解釈する。
fn sweep_mesh(
    profile: &[(f64, f64)],
    path: &[(f64, f64)],
) -> Result<(Vec<f32>, Vec<u32>), BridgeError> {
    let n_profile = profile.len();
    let n_path = path.len();
    if n_profile < 3 {
        return Err(BridgeError::InvalidShape(
            "sweep_extrude: profile 頂点 < 3".to_string(),
        ));
    }
    if n_path < 2 {
        return Err(BridgeError::InvalidShape(
            "sweep_extrude: path 頂点 < 2".to_string(),
        ));
    }
    let mut vertices: Vec<f32> = Vec::with_capacity(n_path * n_profile * 3 + 6);
    for i in 0..n_path {
        let (tx, ty) = if i == 0 {
            (path[1].0 - path[0].0, path[1].1 - path[0].1)
        } else if i == n_path - 1 {
            (path[i].0 - path[i - 1].0, path[i].1 - path[i - 1].1)
        } else {
            (path[i + 1].0 - path[i - 1].0, path[i + 1].1 - path[i - 1].1)
        };
        let len = (tx * tx + ty * ty).sqrt();
        if len < 1e-12 {
            continue;
        }
        let (tx, ty) = (tx / len, ty / len);
        let (nx, ny) = (-ty, tx);
        let px = path[i].0;
        let pz = path[i].1;
        for &(lx, ly) in profile {
            vertices.push((px + lx * nx) as f32);
            vertices.push(ly as f32);
            vertices.push((pz + lx * ny) as f32);
        }
    }
    let n_rings = vertices.len() / 3 / n_profile;
    if n_rings < 2 {
        return Err(BridgeError::InvalidShape(
            "sweep_extrude: path 退化".to_string(),
        ));
    }
    let mut indices: Vec<u32> = Vec::with_capacity((n_rings - 1) * n_profile * 6);
    for i in 0..(n_rings - 1) {
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
    let base = ((n_rings - 1) * n_profile) as u32;
    let (cx, cy, cz) = ring_center(&vertices, ((n_rings - 1) * n_profile) * 3, n_profile);
    vertices.extend_from_slice(&[cx, cy, cz]);
    for j in 0..n_profile as u32 {
        let j_next = (j + 1) % n_profile as u32;
        indices.extend_from_slice(&[end_center_idx, base + j_next, base + j]);
    }
    Ok((vertices, indices))
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
