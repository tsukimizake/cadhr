use cadhr_lang::export::{Mesh, meshes_to_threemf};

use crate::preview::pipeline::Vertex;

pub fn vertices_to_threemf(vertices: &[Vertex], indices: &[u32]) -> Option<Vec<u8>> {
    meshes_to_threemf([Mesh {
        positions: vertices.iter().map(|v| v.position.map(f64::from)).collect(),
        indices: indices.iter().copied().map(u64::from).collect(),
    }])
    .ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exports_preview_vertices() {
        let vertices =
            [[1.25, 2.5, 3.75], [2.25, 2.5, 3.75], [1.25, 3.5, 3.75]].map(|position| Vertex {
                position,
                normal: [0.0, 0.0, 1.0],
                color: [1.0, 0.0, 0.0, 1.0],
            });
        let data = vertices_to_threemf(&vertices, &[0, 1, 2]).unwrap();
        assert!(data.starts_with(b"PK"));
    }

    #[test]
    fn rejects_empty_or_invalid_preview_meshes() {
        let vertices = [Vertex {
            position: [0.0; 3],
            normal: [0.0; 3],
            color: [0.0; 4],
        }];
        assert!(vertices_to_threemf(&[], &[]).is_none());
        assert!(vertices_to_threemf(&vertices, &[]).is_none());
        assert!(vertices_to_threemf(&[], &[0, 0, 0]).is_none());
        assert!(vertices_to_threemf(&vertices, &[0, 0]).is_none());
        assert!(vertices_to_threemf(&vertices, &[0, 0, 1]).is_none());
    }
}
