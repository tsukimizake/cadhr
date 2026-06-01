//! `Model3D` (宣言ツリー) → `manifold-rs` の `Manifold` に評価する橋渡し。
//!
//! evaluator (`runtime::eval`) は副作用なく `Model3D` を組み立てるだけ。実際の
//! CSG 計算 / メッシュ生成はここで manifold-rs に投げる。GUI 側はここから
//! `Vertex` / `index` を受け取って iced shader に流す。
//!
//! manifold-rs 自体は cargo feature `manifold` の有無で gating する。feature OFF の
//! ビルドではこのモジュール全体が disable される。

#![cfg(feature = "manifold")]

use crate::runtime::value::{Model2D, Model3D, Plane3D};
use manifold_rs::{Manifold, Mesh};
use std::path::{Path as StdPath, PathBuf};

const DEFAULT_SEGMENTS: u32 = 64;
/// 2D boolean 用に 2D を 3D へ薄く extrude するときの高さ。`THIN > 0` なら manifold-rs が
/// 動く最小 unit。
const THIN_EXTRUDE_HEIGHT: f64 = 0.001;

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
        Model3D::Cube { x, y, z } => Ok(Manifold::cube(*x, *y, *z)),
        Model3D::Sphere(r) => Ok(Manifold::sphere(*r, DEFAULT_SEGMENTS)),
        Model3D::Cylinder { r, h } => Ok(Manifold::cylinder(*r, *r, *h, DEFAULT_SEGMENTS)),
        Model3D::Tetrahedron => Ok(Manifold::tetrahedron()),

        Model3D::Union(a, b) => Ok(evaluate_with_paths(a, include_paths)?
            .union(&evaluate_with_paths(b, include_paths)?)),
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
        Model3D::Scale { shape, factor } => Ok(evaluate_with_paths(shape, include_paths)?
            .scale(factor.0, factor.1, factor.2)),
        Model3D::Rotate { shape, angles } => Ok(evaluate_with_paths(shape, include_paths)?
            .rotate(angles.0, angles.1, angles.2)),
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
        Model3D::Center3D { shape, target } => {
            let m = evaluate_with_paths(shape, include_paths)?;
            let mesh = m.to_mesh();
            let verts = mesh.vertices();
            let stride = mesh.num_props() as usize;
            if stride == 0 || verts.is_empty() {
                return Ok(m);
            }
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for chunk in verts.chunks(stride) {
                for i in 0..3 {
                    let v = chunk[i] as f64;
                    if v < lo[i] {
                        lo[i] = v;
                    }
                    if v > hi[i] {
                        hi[i] = v;
                    }
                }
            }
            let cx = (lo[0] + hi[0]) / 2.0;
            let cy = (lo[1] + hi[1]) / 2.0;
            let cz = (lo[2] + hi[2]) / 2.0;
            Ok(m.translate(target.0 - cx, target.1 - cy, target.2 - cz))
        }
    }
}

/// `include_paths = &[]` で呼ぶラッパ。STL を使わない場合はこれで十分。
pub fn evaluate(model: &Model3D) -> Result<Manifold, BridgeError> {
    evaluate_with_paths(model, &[])
}

/// `Model2D` を平坦な polygon ring 列に変換する。CSG ノードは extrude → 3D boolean →
/// project で平坦化する (manifold-rs 0.6 が CrossSection を持たない workaround)。
fn to_polygon_rings(profile: &Model2D) -> Option<Vec<Vec<f64>>> {
    match profile {
        Model2D::Polygon(points) if !points.is_empty() => {
            let mut pairs: Vec<(f64, f64)> = points.clone();
            ensure_ccw(&mut pairs);
            let mut ring = Vec::with_capacity(pairs.len() * 2);
            for (x, y) in pairs {
                ring.push(x);
                ring.push(y);
            }
            Some(vec![ring])
        }
        Model2D::Empty2D => None,
        Model2D::Union2D(a, b) => match (to_polygon_rings(a), to_polygon_rings(b)) {
            (Some(ra), Some(rb)) => Some(boolean_2d(&ra, &rb, |x, y| x.union(y))),
            (Some(r), None) | (None, Some(r)) => Some(r),
            (None, None) => None,
        },
        Model2D::Diff2D(a, b) => match (to_polygon_rings(a), to_polygon_rings(b)) {
            (Some(ra), Some(rb)) => Some(boolean_2d(&ra, &rb, |x, y| x.difference(y))),
            (Some(r), None) => Some(r),
            _ => None,
        },
        Model2D::Intersect2D(a, b) => match (to_polygon_rings(a), to_polygon_rings(b)) {
            (Some(ra), Some(rb)) => Some(boolean_2d(&ra, &rb, |x, y| x.intersection(y))),
            _ => None,
        },
        Model2D::Center2D { shape, target } => {
            let rings = to_polygon_rings(shape)?;
            let (cx, cy) = bbox_2d(&rings)?;
            let dx = target.0 - cx;
            let dy = target.1 - cy;
            let shifted: Vec<Vec<f64>> = rings
                .into_iter()
                .map(|ring| {
                    ring.chunks_exact(2)
                        .flat_map(|c| vec![c[0] + dx, c[1] + dy])
                        .collect()
                })
                .collect();
            Some(shifted)
        }
        Model2D::Polygon(_) => None,
    }
}

fn boolean_2d(
    rings_a: &[Vec<f64>],
    rings_b: &[Vec<f64>],
    op: impl FnOnce(&Manifold, &Manifold) -> Manifold,
) -> Vec<Vec<f64>> {
    let refs_a: Vec<&[f64]> = rings_a.iter().map(|r| r.as_slice()).collect();
    let refs_b: Vec<&[f64]> = rings_b.iter().map(|r| r.as_slice()).collect();
    let ma = Manifold::extrude(&refs_a, THIN_EXTRUDE_HEIGHT, 0, 0.0, 1.0, 1.0);
    let mb = Manifold::extrude(&refs_b, THIN_EXTRUDE_HEIGHT, 0, 0.0, 1.0, 1.0);
    let result = op(&ma, &mb);
    let polygons = result.project();
    let mut rings = Vec::with_capacity(polygons.size());
    for i in 0..polygons.size() {
        rings.push(polygons.get_as_slice(i).to_vec());
    }
    rings
}

fn bbox_2d(rings: &[Vec<f64>]) -> Option<(f64, f64)> {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for ring in rings {
        for c in ring.chunks_exact(2) {
            min_x = min_x.min(c[0]);
            max_x = max_x.max(c[0]);
            min_y = min_y.min(c[1]);
            max_y = max_y.max(c[1]);
        }
    }
    if min_x.is_infinite() {
        None
    } else {
        Some(((min_x + max_x) / 2.0, (min_y + max_y) / 2.0))
    }
}

fn prep_rings(rings: Vec<Vec<f64>>, plane: Plane3D) -> Vec<Vec<f64>> {
    if plane != Plane3D::XZ {
        return rings;
    }
    // XZ 押し出しは Y 反転 + ensure_ccw が必要 (rotate(-90,0,0) との辻褄合わせ)
    rings
        .into_iter()
        .map(|ring| {
            let mut pairs: Vec<(f64, f64)> =
                ring.chunks_exact(2).map(|c| (c[0], -c[1])).collect();
            ensure_ccw(&mut pairs);
            pairs.into_iter().flat_map(|(x, y)| vec![x, y]).collect()
        })
        .collect()
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
    n_div: u32,
    twist: f64,
    sx: f64,
    sy: f64,
) -> Result<Manifold, BridgeError> {
    let rings = match to_polygon_rings(profile) {
        Some(r) if !r.is_empty() => prep_rings(r, plane),
        _ => return Ok(Manifold::empty()),
    };
    let refs: Vec<&[f64]> = rings.iter().map(|r| r.as_slice()).collect();
    let m = Manifold::extrude(&refs, height, n_div, twist, sx, sy);
    Ok(apply_plane_rotation(m, plane))
}

fn revolve_polygon(
    profile: &Model2D,
    plane: Plane3D,
    degrees: f64,
) -> Result<Manifold, BridgeError> {
    let rings = match to_polygon_rings(profile) {
        Some(r) if !r.is_empty() => prep_rings(r, plane),
        _ => return Ok(Manifold::empty()),
    };
    let refs: Vec<&[f64]> = rings.iter().map(|r| r.as_slice()).collect();
    let m = Manifold::revolve(&refs, DEFAULT_SEGMENTS, degrees);
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
    let mesh = Mesh::new(&verts, &indices);
    Ok(Manifold::from_mesh(mesh))
}

fn sweep_polygon(
    profile: &Model2D,
    plane: Plane3D,
    path: &[(f64, f64, f64)],
) -> Result<Manifold, BridgeError> {
    let rings = to_polygon_rings(profile).ok_or_else(|| {
        BridgeError::InvalidShape("sweep_extrude: profile が空".to_string())
    })?;
    if rings.is_empty() {
        return Ok(Manifold::empty());
    }
    let profile_pairs = flat_to_pairs(&rings[0]);
    // path 3D → XZ 平面投影 (sweep.rs 互換)。XY/YZ も将来対応可能だが、まず XZ パスのみ。
    let path_xz: Vec<(f64, f64)> = path.iter().map(|p| (p.0, p.2)).collect();
    let (verts, indices) = sweep_mesh(&profile_pairs, &path_xz)?;
    let mesh = Mesh::new(&verts, &indices);
    let m = Manifold::from_mesh(mesh);
    Ok(apply_plane_rotation(m, plane))
}

fn flat_to_pairs(flat: &[f64]) -> Vec<(f64, f64)> {
    flat.chunks_exact(2).map(|c| (c[0], c[1])).collect()
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

fn ensure_ccw(points: &mut Vec<(f64, f64)>) {
    if points.len() < 3 {
        return;
    }
    let mut signed_area = 0.0;
    for i in 0..points.len() {
        let (x1, y1) = points[i];
        let (x2, y2) = points[(i + 1) % points.len()];
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
    let mesh = with_normals.to_mesh();
    Ok(MeshArrays::from_mesh(&mesh))
}

/// GUI に渡しやすい平坦化したメッシュ表現。
pub struct MeshArrays {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

impl MeshArrays {
    fn from_mesh(mesh: &Mesh) -> Self {
        let verts_flat = mesh.vertices();
        let num_props = mesh.num_props() as usize;
        let n_vertices = verts_flat.len() / num_props;
        let mut positions = Vec::with_capacity(n_vertices);
        let mut normals = Vec::with_capacity(n_vertices);
        for i in 0..n_vertices {
            let base = i * num_props;
            positions.push([
                verts_flat[base] as f32,
                verts_flat[base + 1] as f32,
                verts_flat[base + 2] as f32,
            ]);
            if num_props >= 6 {
                normals.push([
                    verts_flat[base + 3] as f32,
                    verts_flat[base + 4] as f32,
                    verts_flat[base + 5] as f32,
                ]);
            } else {
                normals.push([0.0, 0.0, 1.0]);
            }
        }
        Self {
            positions,
            normals,
            indices: mesh.indices().to_vec(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.positions.is_empty() || self.indices.is_empty()
    }
}

/// 評価済み `Manifold` の AABB を返す。`center3d` builtin の実装で使う。
pub fn bbox_of(model: &Model3D) -> Option<((f64, f64, f64), (f64, f64, f64))> {
    let manifold = evaluate(model).ok()?;
    let mesh = manifold.to_mesh();
    let verts = mesh.vertices();
    let stride = mesh.num_props() as usize;
    if stride == 0 || verts.is_empty() {
        return None;
    }
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for chunk in verts.chunks(stride) {
        for i in 0..3 {
            let v = chunk[i] as f64;
            if v < lo[i] {
                lo[i] = v;
            }
            if v > hi[i] {
                hi[i] = v;
            }
        }
    }
    Some(((lo[0], lo[1], lo[2]), (hi[0], hi[1], hi[2])))
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
