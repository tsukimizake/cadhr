//! cadhr-lang インタープリタ呼び出し層。
//!
//! コード変更時と slider 操作時で再実行すべき処理が違うので、2 つの job に分ける:
//!
//! - [`run_compile_job`] : ソース文字列 → `CompiledProgram` + `MainSignature`。
//!   パース・型推論・slider 抽出・網羅性検査までやる重い処理。
//! - [`run_eval_job`] : `CompiledProgram` + slider 値 → 頂点・インデックス + BOM。
//!   `run_main` + `manifold_bridge::to_mesh_arrays_with_paths` の薄ラッパで、
//!   slider を動かすたびに呼ぶ。`CompiledProgram` は `Clone` なので GUI 側 Model に保持する。

use cadhr_lang::runtime::manifold_bridge::{MeshArrays, to_mesh_arrays_with_paths};
use cadhr_lang::{
    CompiledProgram, Inputs, MainSignature, Span, SliderDecl, Value, compile_with_paths, run_main,
};

use crate::preview::pipeline::Vertex;
use std::collections::HashMap;
use std::path::PathBuf;

// ---------------- compile job ----------------

#[derive(Clone, Debug)]
pub struct CompileJobParams {
    pub source: String,
    pub search_paths: Vec<PathBuf>,
}

#[derive(Clone, Debug)]
pub enum CompileJobResult {
    Success {
        program: CompiledProgram,
        signature: MainSignature,
        diagnostics: Vec<String>,
    },
    Error {
        message: String,
        span: Option<Span>,
        diagnostics: Vec<String>,
    },
}

pub fn run_compile_job(params: CompileJobParams) -> CompileJobResult {
    match compile_with_paths(&params.source, &params.search_paths) {
        Ok(prog) => {
            let signature = prog.main_signature.clone();
            let diagnostics = prog.diagnostics.iter().map(|d| d.message.clone()).collect();
            CompileJobResult::Success {
                program: prog,
                signature,
                diagnostics,
            }
        }
        Err(errs) => {
            let first = errs.first();
            CompileJobResult::Error {
                message: first
                    .map(|d| d.message.clone())
                    .unwrap_or_else(|| "compile failed".into()),
                span: first.map(|d| d.span),
                diagnostics: errs.iter().map(|d| d.message.clone()).collect(),
            }
        }
    }
}

// ---------------- eval job ----------------

#[derive(Clone, Debug)]
pub struct EvalJobParams {
    pub program: CompiledProgram,
    pub slider_values: HashMap<String, f64>,
    /// STL ファイル等を解決するための path。compile_job と同じものを渡す。
    pub search_paths: Vec<PathBuf>,
    /// GUI が control point をドラッグして上書きしたときの (name → position)。
    pub control_overrides: HashMap<String, [f64; 3]>,
}

#[derive(Clone)]
pub enum EvalJobResult {
    Success {
        vertices: Vec<Vertex>,
        indices: Vec<u32>,
        bom: Vec<String>,
        /// `control3d "name" (p3 ..)` で記録された (name, current_position)。
        control_points: Vec<(String, [f64; 3])>,
    },
    Error {
        message: String,
        span: Option<Span>,
    },
}

impl std::fmt::Debug for EvalJobResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EvalJobResult::Success {
                vertices,
                indices,
                bom,
                control_points,
            } => f
                .debug_struct("Success")
                .field("vertices_len", &vertices.len())
                .field("indices_len", &indices.len())
                .field("bom_len", &bom.len())
                .field("cp_len", &control_points.len())
                .finish(),
            EvalJobResult::Error { message, span } => f
                .debug_struct("Error")
                .field("message", message)
                .field("span", span)
                .finish(),
        }
    }
}

pub fn run_eval_job(params: EvalJobParams) -> EvalJobResult {
    let mut inputs = Inputs::default();
    inputs.control_overrides = params.control_overrides.clone();
    for p in &params.program.main_signature.params {
        let v = params.slider_values.get(&p.name).copied();
        inputs
            .values
            .insert(p.name.clone(), build_param_value(p.range.as_ref(), v));
    }

    let output = match run_main(&params.program, &inputs) {
        Ok(o) => o,
        Err(d) => {
            return EvalJobResult::Error {
                message: d.message,
                span: Some(d.span),
            };
        }
    };

    let (vertices, indices) = build_mesh(&output.models, &params.search_paths);
    let bom: Vec<String> = output.bom.iter().map(|v| format!("{v}")).collect();
    EvalJobResult::Success {
        vertices,
        indices,
        bom,
        control_points: output.control_points,
    }
}

/// 衝突チェック: `main` の出力 models をペア毎に intersect して、非空の交差を集める。
/// 結果メッシュは交差領域だけを集めたもの (色で main mesh と区別)。
pub fn run_collision_job(params: EvalJobParams) -> EvalJobResult {
    use cadhr_lang::runtime::manifold_bridge::{evaluate_with_paths, to_mesh_arrays_with_paths};
    use cadhr_lang::Model3D;

    let mut inputs = Inputs::default();
    for p in &params.program.main_signature.params {
        let v = params.slider_values.get(&p.name).copied();
        inputs
            .values
            .insert(p.name.clone(), build_param_value(p.range.as_ref(), v));
    }

    let output = match run_main(&params.program, &inputs) {
        Ok(o) => o,
        Err(d) => {
            return EvalJobResult::Error {
                message: d.message,
                span: Some(d.span),
            };
        }
    };

    let models = &output.models;
    let n = models.len();
    let mut collisions: Vec<Model3D> = Vec::new();
    let mut bom: Vec<String> = Vec::new();
    for i in 0..n {
        let mi = match evaluate_with_paths(&models[i], &params.search_paths) {
            Ok(m) => m,
            Err(_) => continue,
        };
        let mesh = mi.to_mesh();
        if mesh.vertices().is_empty() {
            continue;
        }
        for j in (i + 1)..n {
            // 交差は宣言ツリー上で `Intersect` ノードを作る (lazy 評価のまま)。
            // 結果が空かどうかは bridge で確かめる必要があるが、面倒なので全て積む。
            let inter = Model3D::Intersect(
                Box::new(models[i].clone()),
                Box::new(models[j].clone()),
            );
            // 空かどうか確認: evaluate して vertex 数チェック
            if let Ok(m) = evaluate_with_paths(&inter, &params.search_paths) {
                let mesh = m.to_mesh();
                if !mesh.vertices().is_empty() {
                    collisions.push(inter);
                    bom.push(format!("part #{i} ⊗ part #{j}: collision"));
                }
            }
        }
    }

    if collisions.is_empty() {
        bom.push(format!("no collision among {n} parts"));
    }

    let mut vertices: Vec<Vertex> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    for m in &collisions {
        if let Ok(arr) = to_mesh_arrays_with_paths(m, &params.search_paths) {
            let base = vertices.len() as u32;
            for (p, n) in arr.positions.iter().zip(arr.normals.iter()) {
                vertices.push(Vertex {
                    position: *p,
                    normal: *n,
                    color: [1.0, 0.3, 0.3, 0.9],
                });
            }
            for i in arr.indices {
                indices.push(base + i);
            }
        }
    }

    EvalJobResult::Success {
        vertices,
        indices,
        bom,
        control_points: Vec::new(),
    }
}

fn build_param_value(range: Option<&SliderDecl>, v: Option<f64>) -> Value {
    match (range, v) {
        (Some(r), Some(x)) => {
            if r.elem_ty == cadhr_lang::ElemTy::Int {
                Value::Int(x as i64)
            } else {
                Value::Float(x)
            }
        }
        (Some(r), None) => {
            let mid = (r.lo + r.hi) / 2.0;
            if r.elem_ty == cadhr_lang::ElemTy::Int {
                Value::Int(mid as i64)
            } else {
                Value::Float(mid)
            }
        }
        (None, Some(x)) => Value::Float(x),
        (None, None) => Value::Float(0.0),
    }
}

fn build_mesh(
    models: &[cadhr_lang::Model3D],
    search_paths: &[PathBuf],
) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for m in models {
        let arr = match to_mesh_arrays_with_paths(m, search_paths) {
            Ok(a) => a,
            Err(_) => continue,
        };
        append_mesh(&mut vertices, &mut indices, arr);
    }
    (vertices, indices)
}

fn append_mesh(verts: &mut Vec<Vertex>, idxs: &mut Vec<u32>, arr: MeshArrays) {
    let base = verts.len() as u32;
    for (p, n) in arr.positions.iter().zip(arr.normals.iter()) {
        verts.push(Vertex {
            position: *p,
            normal: *n,
            color: [0.0, 0.0, 0.0, 0.0],
        });
    }
    for i in arr.indices {
        idxs.push(base + i);
    }
}
