use crate::preview::pipeline::Vertex;
use threemf::model;

pub fn vertices_to_threemf(vertices: &[Vertex], indices: &[u32]) -> Option<Vec<u8>> {
    if vertices.is_empty() || indices.is_empty() {
        return None;
    }

    let mesh = model::Mesh {
        vertices: model::Vertices {
            vertex: vertices
                .iter()
                .map(|v| model::Vertex {
                    x: v.position[0] as f64,
                    y: v.position[1] as f64,
                    z: v.position[2] as f64,
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
    };

    let object = model::Object {
        id: 1,
        partnumber: None,
        name: None,
        pid: None,
        mesh: Some(mesh),
        components: None,
    };

    let mut m = model::Model::default();
    m.resources.object.push(object);
    m.build.item.push(model::Item {
        objectid: 1,
        transform: None,
        partnumber: None,
    });

    let mut buf = std::io::Cursor::new(Vec::new());
    threemf::write(&mut buf, m).ok()?;
    Some(buf.into_inner())
}
