use cadhr_lang::export::{Mesh, meshes_to_threemf};

use crate::preview::pipeline::Vertex;

pub fn vertices_to_threemf(vertices: &[Vertex], indices: &[u32]) -> Option<Vec<u8>> {
    meshes_to_threemf([Mesh {
        positions: vertices.iter().map(|v| v.position.map(f64::from)).collect(),
        indices: indices.iter().copied().map(u64::from).collect(),
    }])
    .ok()
}
