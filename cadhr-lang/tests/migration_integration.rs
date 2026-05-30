//! cadhr-proj/ から取った代表的なユーザコードを新 cadhr-lang 仕様に書き直し、
//! コンパイル + main 実行が通ることを確認する。Phase 10 (migration verification) の
//! 一環。

use cadhr_lang::{Inputs, Value, compile, run_main};

#[test]
fn counter_cable_clip_compiles() {
    let src = include_str!("migration_counter_cable_clip.cadhr");
    let prog = compile(src).expect("compile");
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1, "1 個の Shape3D が出るはず");
}

#[test]
fn desk_foot_cover_compiles() {
    let src = include_str!("migration_desk_foot_cover.cadhr");
    let prog = compile(src).expect("compile");
    assert_eq!(prog.sliders.len(), 1);
    let mut inputs = Inputs::default();
    inputs.values.insert("x".into(), Value::Float(15.0));
    let out = run_main(&prog, &inputs).expect("run_main");
    assert_eq!(out.models.len(), 1);
}

#[test]
fn bed_cable_clip_compiles() {
    let src = include_str!("migration_bed_cable_clip.cadhr");
    let prog = compile(src).expect("compile");
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1);
}

#[test]
fn bed_drink_holder_compiles() {
    let src = include_str!("migration_bed_drink_holder.cadhr");
    let prog = compile(src).expect("compile");
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1);
}
