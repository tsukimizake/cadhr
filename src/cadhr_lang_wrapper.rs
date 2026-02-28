use bevy::asset::RenderAssetUsages;
use bevy::render::render_resource::PrimitiveTopology;
use bevy::tasks::AsyncComputeTaskPool;
use bevy::{mesh::Indices, prelude::*};
use bevy_async_ecs::AsyncWorld;

use crate::events::{CadhrLangOutput, GeneratePreviewRequest, PreviewGenerated};
use cadhr_lang::manifold_bridge::{apply_var_overrides, extract_control_points, generate_mesh_and_tree_from_terms};
use cadhr_lang::parse::{database, query as parse_query};
use cadhr_lang::term_rewrite::execute;
use manifold_rs::Mesh as RsMesh;

pub struct CadhrLangPlugin;

impl Plugin for CadhrLangPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, start_request_worker);
    }
}

fn start_request_worker(world: &mut World) {
    let async_world = AsyncWorld::from_world(world);
    AsyncComputeTaskPool::get()
        .spawn(async move {
            let requests = async_world
                .start_waiting_for_messages::<GeneratePreviewRequest>()
                .await;
            loop {
                let req = requests.wait().await;
                spawn_mesh_job(async_world.clone(), req);
            }
        })
        .detach();
}

fn spawn_mesh_job(async_world: AsyncWorld, req: GeneratePreviewRequest) {
    AsyncComputeTaskPool::get()
        .spawn(async move {
            let preview_id = req.preview_id;
            let db_src = req.database;
            let query = req.query;

            let mut logs: Vec<String> = Vec::new();
            let result = (|| -> Result<(Mesh, Vec<cadhr_lang::manifold_bridge::EvaluatedNode>, Vec<cadhr_lang::manifold_bridge::ControlPoint>), String> {
                let (_, query_terms) =
                    parse_query(&query).map_err(|e| format!("Query parse error: {:?}", e))?;

                let mut db =
                    database(&db_src).map_err(|e| format!("Database parse error: {:?}", e))?;

                logs.push(format!("Query terms: {:?}", query_terms));
                logs.push(format!("Database clauses: {:#?}", db));

                let mut resolved =
                    execute(&mut db, query_terms).map_err(|e| format!("Rewrite error: {}", e))?;

                logs.push(format!("Resolved terms: {:?}", resolved));

                apply_var_overrides(&mut resolved, &req.control_point_overrides);

                let control_points = extract_control_points(&mut resolved);

                if resolved.is_empty() {
                    // control pointsのみでgeometryがない場合、空メッシュを返す
                    let empty_mesh = Mesh::new(
                        bevy::render::render_resource::PrimitiveTopology::TriangleList,
                        bevy::asset::RenderAssetUsages::RENDER_WORLD | bevy::asset::RenderAssetUsages::MAIN_WORLD,
                    );
                    return Ok((empty_mesh, vec![], control_points));
                }

                let (rs_mesh, evaluated_nodes) = generate_mesh_and_tree_from_terms(&resolved, &req.include_paths)
                    .map_err(|e| format!("Mesh error: {}", e))?;

                Ok((rs_mesh_to_bevy_mesh(&rs_mesh), evaluated_nodes, control_points))
            })();

            let log_message = logs.join("\n");
            if !log_message.is_empty() {
                async_world
                    .send_message(CadhrLangOutput {
                        message: log_message,
                        is_error: false,
                    })
                    .await;
            }

            match result {
                Ok((mesh, evaluated_nodes, control_points)) => {
                    async_world
                        .send_message(PreviewGenerated {
                            preview_id,
                            query,
                            mesh,
                            evaluated_nodes,
                            control_points,
                        })
                        .await;
                }
                Err(e) => {
                    bevy::log::error!("Failed to generate mesh: {}", e);
                    async_world
                        .send_message(CadhrLangOutput {
                            message: e,
                            is_error: true,
                        })
                        .await;
                }
            }
        })
        .detach();
}

// Convert a manifold-rs Mesh into a Bevy Mesh
fn rs_mesh_to_bevy_mesh(rs_mesh: &RsMesh) -> Mesh {
    // Vertices are a flat Vec<f32> with `num_props` stride (first 3 are XYZ).
    // 法線がプロパティに無い場合はpanicします（calculate_normalsを先に呼んでいることを想定）。
    let vertices = rs_mesh.vertices();
    bevy::log::info!("manifold-rs mesh has {} vertices", vertices.len());
    let stride = rs_mesh.num_props() as usize;
    bevy::log::info!("num_props (stride) = {}", stride);
    assert!(
        stride == 6,
        "manifold-rs mesh has no normals; call calculate_normals(0, ...) before to_mesh()"
    );

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(vertices.len() / stride);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(vertices.len() / stride);
    for chunk in vertices.chunks_exact(stride) {
        positions.push([chunk[0], chunk[1], chunk[2]]);
        normals.push([chunk[3], chunk[4], chunk[5]]);
    }

    // Indices are already triangles from manifold-rs
    let indices: Vec<u32> = rs_mesh.indices();

    let mut bevy_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
    );
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_indices(Indices::U32(indices));

    // Insert normals from manifold-rs (required by assertion above)
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);

    bevy_mesh
}
