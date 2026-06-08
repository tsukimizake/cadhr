//! cadhr-proj/ から取った代表的なユーザコードのコンパイル + main 実行が通ることを
//! 確認する。

use cadhr_lang::{Inputs, Value, compile, run_binding};

#[test]
fn counter_cable_clip_compiles() {
    let src = include_str!("migration_counter_cable_clip.cadhr");
    let prog = compile(src).expect("compile");
    let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
    assert_eq!(out.models.len(), 1, "1 個の Shape3D が出るはず");
}

#[test]
fn desk_foot_cover_compiles() {
    let src = include_str!("migration_desk_foot_cover.cadhr");
    let prog = compile(src).expect("compile");
    assert_eq!(prog.sliders.len(), 1);
    let mut inputs = Inputs::default();
    inputs.values.insert("x".into(), Value::Float(15.0));
    let out = run_binding(&prog, "main", &inputs).expect("run_binding");
    assert_eq!(out.models.len(), 1);
}

#[test]
fn bed_cable_clip_compiles() {
    let src = include_str!("migration_bed_cable_clip.cadhr");
    let prog = compile(src).expect("compile");
    let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
    assert_eq!(out.models.len(), 1);
}

#[test]
fn bed_drink_holder_compiles() {
    let src = include_str!("migration_bed_drink_holder.cadhr");
    let prog = compile(src).expect("compile");
    let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
    assert_eq!(out.models.len(), 1);
}
