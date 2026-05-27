use crate::manifold_bridge::{ConversionError, Model3D};
use manifold_rs::{Manifold, Mesh};
use std::path::PathBuf;

pub struct CollisionResult {
    pub combined_mesh: Mesh,
    pub collision_meshes: Vec<Mesh>,
    pub part_count: usize,
}

fn compute_aabb(mesh: &Mesh) -> ([f64; 3], [f64; 3]) {
    let verts = mesh.vertices();
    let stride = mesh.num_props() as usize;
    let mut aabb_min = [f64::INFINITY; 3];
    let mut aabb_max = [f64::NEG_INFINITY; 3];
    for chunk in verts.chunks(stride) {
        for i in 0..3 {
            let v = chunk[i] as f64;
            if v < aabb_min[i] {
                aabb_min[i] = v;
            }
            if v > aabb_max[i] {
                aabb_max[i] = v;
            }
        }
    }
    (aabb_min, aabb_max)
}

fn aabb_overlap(a: &([f64; 3], [f64; 3]), b: &([f64; 3], [f64; 3])) -> bool {
    for i in 0..3 {
        if a.1[i] < b.0[i] || b.1[i] < a.0[i] {
            return false;
        }
    }
    true
}

pub fn check_collisions(
    exprs: &[Model3D],
    include_paths: &[PathBuf],
) -> Result<CollisionResult, ConversionError> {
    if exprs.is_empty() {
        return Err(ConversionError::UnknownPrimitive(
            "no mesh terms found".to_string(),
        ));
    }

    let manifolds: Vec<Manifold> = exprs
        .iter()
        .map(|e| e.evaluate(include_paths))
        .collect::<Result<Vec<_>, _>>()?;

    let part_count = manifolds.len();

    let combined = manifolds
        .iter()
        .fold(Manifold::empty(), |acc, m| acc.union(m));

    let aabbs: Vec<_> = manifolds
        .iter()
        .map(|m| compute_aabb(&m.to_mesh()))
        .collect();

    let mut collision_meshes = Vec::new();
    for i in 0..manifolds.len() {
        for j in (i + 1)..manifolds.len() {
            if !aabb_overlap(&aabbs[i], &aabbs[j]) {
                continue;
            }
            let intersection = manifolds[i].intersection(&manifolds[j]);
            if intersection.is_empty() {
                continue;
            }
            let with_normals = intersection.calculate_normals(0, 30.0).to_mesh();
            collision_meshes.push(with_normals);
        }
    }

    let combined_mesh = combined.calculate_normals(0, 30.0).to_mesh();

    Ok(CollisionResult {
        combined_mesh,
        collision_meshes,
        part_count,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_overlapping_cubes_have_collision() {
        let cube1 = Model3D::Cube {
            x: 10.0,
            y: 10.0,
            z: 10.0,
        };
        let cube2 = Model3D::Translate {
            model: Box::new(Model3D::Cube {
                x: 10.0,
                y: 10.0,
                z: 10.0,
            }),
            x: 5.0,
            y: 0.0,
            z: 0.0,
        };

        let result = check_collisions(&[cube1, cube2], &[]).unwrap();
        assert_eq!(result.part_count, 2);
        assert!(
            !result.collision_meshes.is_empty(),
            "overlapping cubes should have collision"
        );
    }

    #[test]
    fn test_separated_cubes_no_collision() {
        let cube1 = Model3D::Cube {
            x: 10.0,
            y: 10.0,
            z: 10.0,
        };
        let cube2 = Model3D::Translate {
            model: Box::new(Model3D::Cube {
                x: 10.0,
                y: 10.0,
                z: 10.0,
            }),
            x: 100.0,
            y: 0.0,
            z: 0.0,
        };

        let result = check_collisions(&[cube1, cube2], &[]).unwrap();
        assert_eq!(result.part_count, 2);
        assert!(
            result.collision_meshes.is_empty(),
            "separated cubes should have no collision"
        );
    }
}
