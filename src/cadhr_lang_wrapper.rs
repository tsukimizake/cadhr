use bevy::asset::RenderAssetUsages;
use bevy::render::render_resource::PrimitiveTopology;
use bevy::tasks::AsyncComputeTaskPool;
use bevy::{mesh::Indices, prelude::*};
use bevy_async_ecs::AsyncWorld;

use crate::events::{CadhrLangOutput, GeneratePreviewRequest, PreviewGenerated};
use cadhr_lang::manifold_bridge::{extract_control_points, generate_mesh_and_tree_from_terms};
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

            // Parse and resolve: extract control points before mesh generation
            // so they're available even when mesh generation fails
            let resolve_result = (|| -> Result<(Vec<cadhr_lang::parse::Term>, Vec<cadhr_lang::manifold_bridge::ControlPoint>), String> {
                let (_, query_terms) =
                    parse_query(&query).map_err(|e| format!("Query parse error: {:?}", e))?;
                let mut db =
                    database(&db_src).map_err(|e| format!("Database parse error: {:?}", e))?;
                logs.push(format!("Query terms: {:?}", query_terms));
                logs.push(format!("Database clauses: {:#?}", db));
                let mut resolved =
                    execute(&mut db, query_terms).map_err(|e| format!("Rewrite error: {}", e))?;
                logs.push(format!("Resolved terms: {:?}", resolved));
                let control_points = extract_control_points(&mut resolved, &req.control_point_overrides);
                Ok((resolved, control_points))
            })();

            let log_message = logs.join("\n");
            if !log_message.is_empty() {
                async_world
                    .send_message(CadhrLangOutput {
                        preview_id: None,
                        message: log_message,
                        is_error: false,
                    })
                    .await;
            }

            let (resolved, control_points) = match resolve_result {
                Ok(pair) => pair,
                Err(e) => {
                    bevy::log::error!("Failed to resolve: {}", e);
                    async_world
                        .send_message(CadhrLangOutput {
                            preview_id: Some(preview_id),
                            message: e,
                            is_error: true,
                            })
                        .await;
                    return;
                }
            };

            if resolved.is_empty() {
                let empty_mesh = Mesh::new(
                    PrimitiveTopology::TriangleList,
                    RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
                );
                async_world
                    .send_message(PreviewGenerated {
                        preview_id,
                        query,
                        mesh: empty_mesh,
                        evaluated_nodes: vec![],
                        control_points,
                    })
                    .await;
                return;
            }

            let mesh_result = generate_mesh_and_tree_from_terms(&resolved, &req.include_paths)
                .map_err(|e| format!("Mesh error: {}", e))
                .and_then(|(rs_mesh, evaluated_nodes)| {
                    rs_mesh_to_bevy_mesh(&rs_mesh).map(|m| (m, evaluated_nodes))
                });

            match mesh_result {
                Ok((mesh, evaluated_nodes)) => {
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
                            preview_id: Some(preview_id),
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
fn rs_mesh_to_bevy_mesh(rs_mesh: &RsMesh) -> Result<Mesh, String> {
    let vertices = rs_mesh.vertices();
    let stride = rs_mesh.num_props() as usize;
    if stride != 6 {
        return Err(format!(
            "manifold-rs mesh has unexpected num_props={} (expected 6: xyz+normals)",
            stride
        ));
    }

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

    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);

    Ok(bevy_mesh)
}
