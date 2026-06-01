//! 拡張 builtin の動作テスト。
//! revolve / complex_extrude / sweep_extrude / 2D CSG / center / bezier。

use cadhr_lang::{Inputs, compile, run_main};

fn compile_run(src: &str) -> cadhr_lang::MainOutput {
    let prog = compile(src).expect("compile");
    run_main(&prog, &Inputs::default()).expect("run_main")
}

#[test]
fn revolve_xy_makes_solid() {
    let src = "main = polygon [p2 1.0 0.0, p2 3.0 0.0, p2 3.0 1.0, p2 1.0 1.0] |> revolve_xy 360.0";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(!mesh.is_empty());
}

#[test]
fn complex_extrude_with_twist() {
    let src = "main = polygon [p2 0.0 0.0, p2 4.0 0.0, p2 4.0 4.0, p2 0.0 4.0] |> complex_extrude_xy 10.0 45.0 1.0 1.0";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(!mesh.is_empty());
}

#[test]
fn union2d_produces_non_empty_extrusion() {
    let src = "
        main =
            let
                a = polygon [p2 0.0 0.0, p2 4.0 0.0, p2 4.0 4.0, p2 0.0 4.0]
                b = polygon [p2 2.0 2.0, p2 6.0 2.0, p2 6.0 6.0, p2 2.0 6.0]
                merged = union2d a b
            in
            extrude_xy 1.0 merged
    ";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(!mesh.is_empty());
}

#[test]
fn diff2d_extruded() {
    let src = "
        main =
            let
                outer = polygon [p2 0.0 0.0, p2 8.0 0.0, p2 8.0 8.0, p2 0.0 8.0]
                hole = polygon [p2 2.0 2.0, p2 6.0 2.0, p2 6.0 6.0, p2 2.0 6.0]
            in
            extrude_xy 1.0 (diff2d outer hole)
    ";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(!mesh.is_empty());
}

#[test]
fn bezier_quad_returns_points() {
    let src = "
        main =
            let
                arc = bezier_quad (p2 0.0 0.0) (p2 5.0 10.0) (p2 10.0 0.0) 8
                ring = (p2 0.0 0.0) :: arc
            in
            extrude_xy 1.0 (polygon ring)
    ";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(!mesh.is_empty());
}

#[test]
fn center3d_moves_bbox_to_origin() {
    let src = "main = cube 10.0 10.0 10.0 |> translate3d (p3 0.0 0.0 0.0) (p3 50.0 0.0 0.0) |> center3d (p3 0.0 0.0 0.0)";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    for p in &mesh.positions {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
    }
    // 中心は ~0、cube は 10 mm なので bbox は -5..5
    assert!(min_x < -4.5 && max_x > 4.5, "centered bbox: {min_x}..{max_x}");
}

#[test]
fn sweep_extrude_basic() {
    // 円形 profile を直線 path 沿いに sweep。最小限の動作確認。
    let src = "main = sweep_extrude_xy [p3 0.0 0.0 0.0, p3 0.0 0.0 100.0] (circle 5.0)";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(
        !mesh.is_empty(),
        "sweep should produce verts: {} v / {} i",
        mesh.positions.len(),
        mesh.indices.len()
    );
}

#[test]
fn control3d_passes_point_through_default() {
    let src = "main = cube 1.0 1.0 1.0 |> translate3d (p3 0.0 0.0 0.0) (control3d \"pos\" (p3 5.0 0.0 0.0))";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(!mesh.is_empty());
}

#[test]
fn control3d_records_name_and_default() {
    let src = "main = cube 1.0 1.0 1.0 |> translate3d (p3 0.0 0.0 0.0) (control3d \"corner\" (p3 5.0 7.0 9.0))";
    let prog = cadhr_lang::compile(src).expect("compile");
    let out = cadhr_lang::run_main(&prog, &cadhr_lang::Inputs::default()).expect("run_main");
    assert_eq!(out.control_points.len(), 1);
    assert_eq!(out.control_points[0].0, "corner");
    assert_eq!(out.control_points[0].1, [5.0, 7.0, 9.0]);
}

#[test]
fn control3d_overrides_default_when_inputs_provided() {
    let src = "main = cube 1.0 1.0 1.0 |> translate3d (p3 0.0 0.0 0.0) (control3d \"corner\" (p3 5.0 7.0 9.0))";
    let prog = cadhr_lang::compile(src).expect("compile");
    let mut inputs = cadhr_lang::Inputs::default();
    inputs.control_overrides.insert("corner".into(), [1.0, 2.0, 3.0]);
    let out = cadhr_lang::run_main(&prog, &inputs).expect("run_main");
    assert_eq!(out.control_points[0].1, [1.0, 2.0, 3.0]);
    // メッシュの bbox が override 通り (translate に override 値が伝播している)
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    let mut max_x = f32::NEG_INFINITY;
    for p in &mesh.positions {
        max_x = max_x.max(p[0]);
    }
    // cube は (0,0,0)..(1,1,1) → translate (1,2,3) → (1,2,3)..(2,3,4) → max_x ≈ 2
    assert!((max_x - 2.0).abs() < 0.1, "max_x={max_x}");
}
