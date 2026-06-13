//! cadhr-lang インタープリタ呼び出し層。
//!
//! コード変更時と slider 操作時で再実行すべき処理が違うので、2 つの job に分ける:
//!
//! - [`run_compile_job`] : ソース文字列 → `CompiledProgram`。
//!   パース・型推論・slider 抽出・網羅性検査までやる重い処理。
//! - [`run_eval_job`] : `CompiledProgram` + slider 値 + target binding 名 → 頂点・
//!   インデックス + BOM。`run_binding` + `manifold_bridge::to_mesh_arrays_with_paths`
//!   の薄ラッパで、slider を動かすたびに呼ぶ。`CompiledProgram` は `Clone` なので
//!   GUI 側 Model に保持する。
//! - [`run_collision_job`] : 衝突プレビュー専用。常に `main` の models を pair で
//!   交差判定する。

use cadhr_lang::runtime::manifold_bridge::{MeshArrays, to_mesh_arrays_with_paths};
use cadhr_lang::{
    BindingParam, CompiledProgram, Inputs, Span, Value, compile_with_paths, run_binding,
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
        diagnostics: Vec<String>,
    },
    Error {
        message: String,
        span: Option<Span>,
        diagnostics: Vec<String>,
    },
}

pub fn run_compile_job(params: CompileJobParams) -> CompileJobResult {
    log_compile_request(&params);
    let result = match compile_with_paths(&params.source, &params.search_paths) {
        Ok(prog) => {
            let diagnostics = prog.diagnostics.iter().map(|d| d.message()).collect();
            CompileJobResult::Success {
                program: prog,
                diagnostics,
            }
        }
        Err(errs) => {
            let first = errs.first();
            CompileJobResult::Error {
                message: first
                    .map(|d| d.message())
                    .unwrap_or_else(|| "compile failed".into()),
                span: first.map(|d| d.span()),
                diagnostics: errs.iter().map(|d| d.message()).collect(),
            }
        }
    };
    log_compile_result(&result);
    result
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
    /// 評価する top-level binding の名前。`"main"` がデフォルト。
    pub target: String,
}

#[derive(Clone, Debug)]
pub struct CollisionJobParams {
    pub program: CompiledProgram,
    pub slider_values: HashMap<String, f64>,
    pub search_paths: Vec<PathBuf>,
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
    log_eval_request("eval", &params);
    let inputs = build_inputs(&params.program, &params.target, &params);

    let output = match run_binding(&params.program, &params.target, &inputs) {
        Ok(o) => o,
        Err(d) => {
            let r = EvalJobResult::Error {
                message: d.message(),
                span: Some(d.span()),
            };
            log_eval_result("eval", &r);
            return r;
        }
    };

    let (vertices, indices) = build_mesh(&output.models, &params.search_paths);
    let bom: Vec<String> = output.bom.iter().map(|v| format!("{v}")).collect();
    let result = EvalJobResult::Success {
        vertices,
        indices,
        bom,
        control_points: output.control_points,
    };
    log_eval_result("eval", &result);
    result
}

/// 衝突チェック: `main` の出力 models をペア毎に intersect して、非空の交差を集める。
/// 結果メッシュは交差領域だけを集めたもの (色で main mesh と区別)。target は常に `main`。
pub fn run_collision_job(params: CollisionJobParams) -> EvalJobResult {
    use cadhr_lang::Model3D;
    use cadhr_lang::runtime::manifold_bridge::{evaluate_with_paths, to_mesh_arrays_with_paths};

    log_collision_request(&params);
    let inputs = build_collision_inputs(&params);

    let output = match run_binding(&params.program, "main", &inputs) {
        Ok(o) => o,
        Err(d) => {
            let r = EvalJobResult::Error {
                message: d.message(),
                span: Some(d.span()),
            };
            log_eval_result("collision", &r);
            return r;
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
        if mi.is_empty() {
            continue;
        }
        for j in (i + 1)..n {
            // 交差は宣言ツリー上で `Intersect` ノードを作る (lazy 評価のまま)。
            let inter = Model3D::Intersect(Box::new(models[i].clone()), Box::new(models[j].clone()));
            // 空かどうか確認: evaluate して vertex 数チェック
            if let Ok(m) = evaluate_with_paths(&inter, &params.search_paths) {
                if !m.is_empty() {
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

    let result = EvalJobResult::Success {
        vertices,
        indices,
        bom,
        control_points: Vec::new(),
    };
    log_eval_result("collision", &result);
    result
}

fn build_inputs(prog: &CompiledProgram, target: &str, params: &EvalJobParams) -> Inputs {
    let mut inputs = Inputs::default();
    inputs.control_overrides = params.control_overrides.clone();
    inputs.search_paths = params.search_paths.clone();
    if let Some(sig) = prog.binding_signature(target) {
        for p in &sig.params {
            let v = params.slider_values.get(&p.name).copied();
            inputs
                .values
                .insert(p.name.clone(), build_param_value(p, v));
        }
    }
    inputs
}

fn build_collision_inputs(params: &CollisionJobParams) -> Inputs {
    let mut inputs = Inputs::default();
    inputs.search_paths = params.search_paths.clone();
    if let Some(sig) = params.program.binding_signature("main") {
        for p in &sig.params {
            let v = params.slider_values.get(&p.name).copied();
            inputs
                .values
                .insert(p.name.clone(), build_param_value(p, v));
        }
    }
    inputs
}

fn build_param_value(p: &BindingParam, v: Option<f64>) -> Value {
    match (&p.range, v) {
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

// ---------------- debug logging ----------------

fn log_compile_request(params: &CompileJobParams) {
    eprintln!("───── cadhr-lang compile ─────");
    if !params.search_paths.is_empty() {
        eprintln!("[search_paths]");
        for p in &params.search_paths {
            eprintln!("  {}", p.display());
        }
    }
    eprintln!("[source]");
    for (i, line) in params.source.lines().enumerate() {
        eprintln!("  {:>4} | {line}", i + 1);
    }
}

fn log_compile_result(result: &CompileJobResult) {
    match result {
        CompileJobResult::Success {
            program,
            diagnostics,
        } => {
            eprintln!("[compile OK]");
            let names: Vec<String> = program
                .previewable_bindings()
                .into_iter()
                .map(|b| b.name)
                .collect();
            eprintln!("  previewable bindings: [{}]", names.join(", "));
            for d in diagnostics {
                eprintln!("  diag: {d}");
            }
        }
        CompileJobResult::Error {
            message,
            span,
            diagnostics,
        } => {
            eprintln!("[compile ERROR] {message} (span={span:?})");
            for d in diagnostics {
                eprintln!("  diag: {d}");
            }
        }
    }
    eprintln!("──────────────────────────────");
}

fn log_eval_request(kind: &str, params: &EvalJobParams) {
    eprintln!("───── cadhr-lang {kind} target={} ─────", params.target);
    log_slider_table(&params.slider_values);
    if !params.control_overrides.is_empty() {
        eprintln!("[control_overrides]");
        let mut items: Vec<(&String, &[f64; 3])> = params.control_overrides.iter().collect();
        items.sort_by(|a, b| a.0.cmp(b.0));
        for (k, v) in items {
            eprintln!("  {k} = [{}, {}, {}]", v[0], v[1], v[2]);
        }
    }
}

fn log_collision_request(params: &CollisionJobParams) {
    eprintln!("───── cadhr-lang collision ─────");
    log_slider_table(&params.slider_values);
}

fn log_slider_table(sliders: &HashMap<String, f64>) {
    if sliders.is_empty() {
        return;
    }
    eprintln!("[sliders]");
    let mut items: Vec<(&String, &f64)> = sliders.iter().collect();
    items.sort_by(|a, b| a.0.cmp(b.0));
    for (k, v) in items {
        eprintln!("  {k} = {v}");
    }
}

fn log_eval_result(kind: &str, result: &EvalJobResult) {
    match result {
        EvalJobResult::Success {
            vertices,
            indices,
            bom,
            control_points,
        } => {
            eprintln!(
                "[{kind} OK] verts={} idx={} bom={} cp={}",
                vertices.len(),
                indices.len(),
                bom.len(),
                control_points.len()
            );
            for b in bom {
                eprintln!("  bom: {b}");
            }
            for (n, p) in control_points {
                eprintln!("  cp: {n} = [{}, {}, {}]", p[0], p[1], p[2]);
            }
        }
        EvalJobResult::Error { message, span } => {
            eprintln!("[{kind} ERROR] {message} (span={span:?})");
        }
    }
    eprintln!("──────────────────────────────");
}
