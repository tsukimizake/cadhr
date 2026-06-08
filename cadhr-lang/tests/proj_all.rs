//! `~/cadhr-proj/` 配下の全プロジェクトを実際にロード + 評価する smoke test。
//! ローカル環境専用 (CI では skip)。

use cadhr_lang::{Inputs, compile_with_paths};
use std::path::PathBuf;

fn proj_root() -> Option<PathBuf> {
    let home = std::env::var("HOME").ok()?;
    let p = PathBuf::from(home).join("cadhr-proj");
    if p.exists() { Some(p) } else { None }
}

fn try_project(name: &str) {
    let Some(root) = proj_root() else {
        eprintln!("skip ({name}): ~/cadhr-proj が無い");
        return;
    };
    let main_path = root.join(name).join("db.cadhr");
    if !main_path.exists() {
        eprintln!("skip ({name}): db.cadhr が無い");
        return;
    }
    let src = std::fs::read_to_string(&main_path).expect("read");
    let prog = match compile_with_paths(&src, &[root.clone()]) {
        Ok(p) => p,
        Err(diags) => {
            for d in &diags {
                eprintln!("compile err {name}: {}", d.message());
            }
            panic!("{name}: compile failed");
        }
    };
    for d in &prog.diagnostics {
        eprintln!("{name} [{:?}] {}", d.severity(), d.message());
    }
    let out = cadhr_lang::run_binding(&prog, "main", &Inputs::default())
        .unwrap_or_else(|e| panic!("{name}: run_binding err: {}", e.message()));
    assert!(!out.models.is_empty(), "{name}: no model produced");
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0])
        .unwrap_or_else(|e| panic!("{name}: bridge err: {}", e));
    assert!(
        !mesh.is_empty(),
        "{name}: empty mesh ({} verts / {} idx)",
        mesh.positions.len(),
        mesh.indices.len()
    );
}

#[test]
fn counter_cable_clip() {
    try_project("counter_cable_clip");
}
#[test]
fn desk_foot_cover() {
    try_project("desk_foot_cover");
}
#[test]
fn bed_cable_clip() {
    try_project("bed-cable-clip");
}
#[test]
fn bed_drink_holder() {
    try_project("bed_drink_holder");
}
#[test]
fn battery_case() {
    try_project("battery_case");
}
#[test]
fn wabouchou() {
    try_project("wabouchou");
}
#[test]
fn ive_reararck() {
    try_project("ive_reararck");
}
#[test]
fn ive_rear_doghouse_mount() {
    try_project("ive_rear_doghouse_mount");
}
#[test]
fn loki_home_key() {
    try_project("loki_home_key");
}
