//! 実際の `~/cadhr-proj/loki_home_key/db.cadhr` + `~/cadhr-proj/Bolts/db.cadhr` を
//! 直接ロードして新仕様で評価できるかを smoke check する。
//!
//! 個人環境依存なので、ファイルが無いときは自動的に skip する。

use cadhr_lang::{Inputs, compile_with_paths, run_main};
use std::path::PathBuf;

fn proj_root() -> Option<PathBuf> {
    let home = std::env::var("HOME").ok()?;
    let p = PathBuf::from(home).join("cadhr-proj");
    if p.exists() { Some(p) } else { None }
}

#[test]
fn loki_home_key_from_cadhr_proj() {
    let Some(root) = proj_root() else {
        eprintln!("skip: ~/cadhr-proj が存在しないので smoke test を省略");
        return;
    };
    let main_path = root.join("loki_home_key/db.cadhr");
    if !main_path.exists() {
        eprintln!("skip: {} が無い", main_path.display());
        return;
    }
    let src = std::fs::read_to_string(&main_path).expect("read main");
    let prog = compile_with_paths(&src, &[root.clone()]).expect("compile");
    // 型エラーが warning として残っていないか (signature 付け加えた以降のレグレッション
    // チェック)。Severity::Warning は許容するが、特に "型" "Type" 系のメッセージは
    // 警告でも目立たせる。
    for d in &prog.diagnostics {
        eprintln!("[{:?}] {}", d.severity, d.message);
    }
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0])
        .expect("manifold evaluate");
    assert!(
        !mesh.is_empty(),
        "mesh は空でないはず: {} verts / {} idx",
        mesh.positions.len(),
        mesh.indices.len()
    );
}
