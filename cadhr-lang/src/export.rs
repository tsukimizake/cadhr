use std::io::Cursor;
use std::path::PathBuf;

use cadhr_lang::Model3D;
use cadhr_lang::runtime::manifold_bridge::evaluate_with_paths;
use threemf::model;

pub fn models_to_threemf(models: &[Model3D], search_paths: &[PathBuf]) -> Result<Vec<u8>, String> {
    let mut document = model::Model::default();
    for (index, shape) in models.iter().enumerate() {
        let manifold = evaluate_with_paths(shape, search_paths)
            .map_err(|e| format!("mesh (model #{}): {e}", index + 1))?;
        if manifold.is_empty() {
            continue;
        }
        let (vertices, num_props, indices) = manifold.to_mesh_f64();
        let id = document.resources.object.len() + 1;
        document.resources.object.push(model::Object {
            id,
            mesh: Some(model::Mesh {
                vertices: model::Vertices {
                    vertex: vertices
                        .chunks_exact(num_props)
                        .map(|v| model::Vertex {
                            x: v[0],
                            y: v[1],
                            z: v[2],
                        })
                        .collect(),
                },
                triangles: model::Triangles {
                    triangle: indices
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
