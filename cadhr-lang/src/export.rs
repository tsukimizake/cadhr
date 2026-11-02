use std::io::Cursor;
use std::path::PathBuf;

use crate::Model3D;
use crate::runtime::manifold_bridge::evaluate_with_paths;
use threemf::model;

#[derive(Default)]
pub struct Mesh {
    pub positions: Vec<[f64; 3]>,
    pub indices: Vec<u64>,
}

pub fn models_to_threemf(models: &[Model3D], search_paths: &[PathBuf]) -> Result<Vec<u8>, String> {
    let mut meshes = Vec::new();
    for (index, shape) in models.iter().enumerate() {
        let manifold = evaluate_with_paths(shape, search_paths)
            .map_err(|e| format!("mesh (model #{}): {e}", index + 1))?;
        if manifold.is_empty() {
            continue;
        }
        let (vertices, num_props, indices) = manifold.to_mesh_f64();
        meshes.push(Mesh {
            positions: vertices
                .chunks_exact(num_props)
                .map(|v| [v[0], v[1], v[2]])
                .collect(),
            indices,
        });
    }
    meshes_to_threemf(meshes)
}

pub fn meshes_to_threemf(meshes: impl IntoIterator<Item = Mesh>) -> Result<Vec<u8>, String> {
    let mut document = model::Model::default();
    for (index, mesh) in meshes.into_iter().enumerate() {
        if mesh.positions.is_empty() || mesh.indices.is_empty() {
            continue;
        }
        if !mesh.indices.len().is_multiple_of(3) {
            return Err(format!(
                "mesh #{}: index count must be a multiple of 3",
                index + 1
            ));
        }
        if mesh
            .indices
            .iter()
            .any(|&i| i >= mesh.positions.len() as u64)
        {
            return Err(format!("mesh #{}: vertex index out of bounds", index + 1));
        }
        if mesh.positions.iter().flatten().any(|v| !v.is_finite()) {
            return Err(format!(
                "mesh #{}: vertex coordinates must be finite",
                index + 1
            ));
        }
        let id = document.resources.object.len() + 1;
        document.resources.object.push(model::Object {
            id,
            mesh: Some(model::Mesh {
                vertices: model::Vertices {
                    vertex: mesh
                        .positions
                        .into_iter()
                        .map(|[x, y, z]| model::Vertex { x, y, z })
                        .collect(),
                },
                triangles: model::Triangles {
                    triangle: mesh
                        .indices
                        .chunks_exact(3)
                        .map(|tri| model::Triangle {
                            v1: tri[0] as usize,
                            v2: tri[1] as usize,
                            v3: tri[2] as usize,
                        })
                        .collect(),
                },
            }),
            ..Default::default()
        });
        document.build.item.push(model::Item {
            objectid: id,
            transform: None,
            partnumber: None,
        });
    }
    if document.build.item.is_empty() {
        return Err("nothing to export".to_string());
    }
    let mut buffer = Cursor::new(Vec::new());
    threemf::write(&mut buffer, document).map_err(|e| e.to_string())?;
    Ok(buffer.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tetrahedron(offset: f64) -> Mesh {
        Mesh {
            positions: vec![
                [offset, 0.0, 0.0],
                [offset + 1.0, 0.0, 0.0],
                [offset, 1.0, 0.0],
                [offset, 0.0, 1.0],
            ],
            indices: vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
        }
    }

    #[test]
    fn writes_separate_objects_with_local_indices_and_double_precision() {
        let meshes = [tetrahedron(0.0), tetrahedron(100000000.125)];
        let data = meshes_to_threemf([
            Mesh::default(),
            tetrahedron(0.0),
            Mesh::default(),
            tetrahedron(100000000.125),
        ])
        .unwrap();
        let mut documents = threemf::read(Cursor::new(data)).unwrap();
        assert_eq!(documents.len(), 1);
        let document = documents.pop().unwrap();
        assert!(matches!(document.unit, model::Unit::Millimeter));
        assert_eq!(document.resources.object.len(), 2);
        assert_eq!(document.build.item.len(), 2);
        for (index, expected) in meshes.iter().enumerate() {
            let object = &document.resources.object[index];
            let item = &document.build.item[index];
            assert_eq!(object.id, index + 1);
            assert_eq!(item.objectid, object.id);
            assert!(item.transform.is_none());
            let mesh = object.mesh.as_ref().unwrap();
            assert_eq!(
                mesh.vertices
                    .vertex
                    .iter()
                    .map(|v| [v.x, v.y, v.z])
                    .collect::<Vec<_>>(),
                expected.positions,
            );
            assert_eq!(
                mesh.triangles
                    .triangle
                    .iter()
                    .flat_map(|t| [t.v1 as u64, t.v2 as u64, t.v3 as u64])
                    .collect::<Vec<_>>(),
                expected.indices,
            );
        }
    }

    #[test]
    fn rejects_empty_exports() {
        assert_eq!(meshes_to_threemf([]).unwrap_err(), "nothing to export");
        assert_eq!(
            meshes_to_threemf([Mesh::default()]).unwrap_err(),
            "nothing to export"
        );
        assert_eq!(
            models_to_threemf(&[], &[]).unwrap_err(),
            "nothing to export"
        );
    }

    #[test]
    fn rejects_incomplete_triangles() {
        let mut mesh = tetrahedron(0.0);
        mesh.indices.pop();
        assert!(
            meshes_to_threemf([mesh])
                .unwrap_err()
                .contains("multiple of 3")
        );
    }

    #[test]
    fn rejects_out_of_bounds_indices() {
        for index in [4, u64::MAX] {
            let mut mesh = tetrahedron(0.0);
            mesh.indices[0] = index;
            assert!(
                meshes_to_threemf([mesh])
                    .unwrap_err()
                    .contains("out of bounds")
            );
        }
    }

    #[test]
    fn rejects_non_finite_coordinates() {
        for coordinate in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut mesh = tetrahedron(0.0);
            mesh.positions[0][0] = coordinate;
            assert!(meshes_to_threemf([mesh]).unwrap_err().contains("finite"));
        }
    }
}
