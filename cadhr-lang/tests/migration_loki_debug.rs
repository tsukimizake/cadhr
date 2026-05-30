//! 個別の extrude_xy / extrude_yz / extrude_xz が想定の位置にメッシュを置くかを
//! 確認するための debug テスト。

use cadhr_lang::{Inputs, compile, run_main};

fn bbox(model: &cadhr_lang::Model3D) -> ((f32, f32, f32), (f32, f32, f32)) {
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(model).unwrap();
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for p in &mesh.positions {
        for i in 0..3 {
            min[i] = min[i].min(p[i]);
            max[i] = max[i].max(p[i]);
        }
    }
    ((min[0], min[1], min[2]), (max[0], max[1], max[2]))
}

fn run(src: &str) -> cadhr_lang::Model3D {
    let prog = compile(src).expect("compile");
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    out.models[0].clone()
}

#[test]
fn extrude_xy_basic() {
    let m = run("main = polygon [p2 0.0 0.0, p2 80.0 0.0, p2 80.0 55.0, p2 0.0 55.0] |> extrude_xy 15.0");
    let (lo, hi) = bbox(&m);
    eprintln!("XY: lo={lo:?} hi={hi:?}");
    assert!(hi.0 > 70.0 && hi.1 > 45.0 && hi.2 > 10.0, "XY 押し出しは x≈80 y≈55 z≈15");
}

#[test]
fn extrude_yz_basic() {
    let m = run("main = polygon [p2 0.0 0.0, p2 55.0 0.0, p2 55.0 15.0, p2 0.0 15.0] |> extrude_yz 80.0");
    let (lo, hi) = bbox(&m);
    eprintln!("YZ: lo={lo:?} hi={hi:?}");
    // YZ 押し出しは y≈55 z≈15 x≈80
    assert!(hi.0 > 70.0, "X 方向に押し出されていてほしい");
    assert!(hi.1 > 45.0, "Y 方向のスケールは 55 程度");
    assert!(hi.2 > 10.0, "Z 方向のスケールは 15 程度");
}

#[test]
fn extrude_xz_basic() {
    let m = run("main = polygon [p2 0.0 0.0, p2 80.0 0.0, p2 80.0 15.0, p2 0.0 15.0] |> extrude_xz 55.0");
    let (lo, hi) = bbox(&m);
    eprintln!("XZ: lo={lo:?} hi={hi:?}");
    assert!(hi.0 > 70.0, "X 方向のスケールは 80 程度");
    assert!(hi.1 > 45.0, "Y 方向 (押し出し方向) のスケールは 55 程度");
    assert!(hi.2 > 10.0, "Z 方向のスケールは 15 程度");
}
