use crate::manifold_bridge::ConversionError;

/// Path の2D座標を XZ 平面上の3D曲線として解釈し、
/// profile ポリゴンをその曲線に沿って sweep してメッシュを生成する。
/// profile の local X → path の法線方向、local Y → global Y。
pub fn sweep_extrude_mesh(
    profile: &[(f64, f64)],
    path: &[(f64, f64)],
) -> Result<(Vec<f32>, Vec<u32>), ConversionError> {
    let n_profile = profile.len();
    let n_path = path.len();

    if n_profile < 3 {
        return Err(ConversionError::TypeMismatch {
            functor: "sweep_extrude".to_string(),
            arg_index: 0,
            expected: "polygon with at least 3 points",
        });
    }
    if n_path < 2 {
        return Err(ConversionError::TypeMismatch {
            functor: "sweep_extrude".to_string(),
            arg_index: 1,
            expected: "path with at least 2 points",
        });
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
        // path 法線 (tangent に垂直)
        let (nx, ny) = (-ty, tx);

        // path 2D (px, py) → 3D (px, 0, py)
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
        return Err(ConversionError::TypeMismatch {
            functor: "sweep_extrude".to_string(),
            arg_index: 1,
            expected: "path with non-degenerate segments",
        });
    }

    let mut indices: Vec<u32> = Vec::with_capacity((n_rings - 1) * n_profile * 6 + n_profile * 6);

    // side quads
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

    // start cap
    let start_center_idx = (vertices.len() / 3) as u32;
    let (cx, cy, cz) = ring_center(&vertices, 0, n_profile);
    vertices.extend_from_slice(&[cx, cy, cz]);
    for j in 0..n_profile {
        let j_next = (j + 1) % n_profile;
        indices.extend_from_slice(&[start_center_idx, j as u32, j_next as u32]);
    }

    // end cap
    let end_center_idx = (vertices.len() / 3) as u32;
    let base = (n_rings - 1) * n_profile;
    let (cx, cy, cz) = ring_center(&vertices, base, n_profile);
    vertices.extend_from_slice(&[cx, cy, cz]);
    for j in 0..n_profile {
        let j_next = (j + 1) % n_profile;
        indices.extend_from_slice(&[end_center_idx, (base + j_next) as u32, (base + j) as u32]);
    }

    Ok((vertices, indices))
}

fn ring_center(vertices: &[f32], base: usize, n: usize) -> (f32, f32, f32) {
    let (mut sx, mut sy, mut sz) = (0.0f64, 0.0f64, 0.0f64);
    for k in 0..n {
        let idx = (base + k) * 3;
        sx += vertices[idx] as f64;
        sy += vertices[idx + 1] as f64;
        sz += vertices[idx + 2] as f64;
    }
    let n = n as f64;
    ((sx / n) as f32, (sy / n) as f32, (sz / n) as f32)
}
