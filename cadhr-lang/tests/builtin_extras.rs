//! 拡張 builtin の動作テスト。
//! revolve / complex_extrude / sweep_extrude / 2D CSG / center / bezier。

use cadhr_lang::{Inputs, compile, run_binding};

fn compile_run(src: &str) -> cadhr_lang::MainOutput {
    let prog = compile(src).expect("compile");
    run_binding(&prog, "main", &Inputs::default()).expect("run_binding")
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
            extrude_xy 1.0 (outer |> diff2d hole)
    ";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(!mesh.is_empty());
}

#[test]
fn diff2d_extruded_xz_keeps_hole() {
    // XZ 押し出しの Y 反転で穴 contour の巻き向きが壊れないことの回帰テスト
    let src = "
        main =
            let
                outer = polygon [p2 0.0 0.0, p2 8.0 0.0, p2 8.0 8.0, p2 0.0 8.0]
                hole = polygon [p2 2.0 2.0, p2 6.0 2.0, p2 6.0 6.0, p2 2.0 6.0]
            in
            extrude_xz 1.0 (outer |> diff2d hole)
    ";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    // 穴なしの箱は 12 三角形 (36 indices)。穴があれば必ずそれより多い。
    assert!(mesh.indices.len() > 36, "hole was lost: {} indices", mesh.indices.len());
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
    // center3d は Point3D を返すので translate3d と組み合わせて中心移動を表現する。
    let src = "main =
    let c = cube 10.0 10.0 10.0 |> translate3d (p3 0.0 0.0 0.0) (p3 50.0 0.0 0.0) in
    c |> translate3d (center3d c) (p3 0.0 0.0 0.0)
";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    for p in &mesh.positions {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
    }
    // 中心は ~0、cube は 10 mm なので bbox は -5..5
    assert!(
        min_x < -4.5 && max_x > 4.5,
        "centered bbox: {min_x}..{max_x}"
    );
}

#[test]
fn center2d_with_translate2d_centers_polygon() {
    // 10x10 の正方形を (50,50) 平行移動した後、center2d + translate2d で原点に戻す。
    // 結果を XY で extrude して bbox を見ると -5..5 になるはず。
    let src = "main =
    let sq =
            polygon [p2 0.0 0.0, p2 10.0 0.0, p2 10.0 10.0, p2 0.0 10.0]
                |> translate2d (p2 0.0 0.0) (p2 50.0 50.0)
    in
    sq |> translate2d (center2d sq) (p2 0.0 0.0) |> extrude_xy 1.0
";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;
    for p in &mesh.positions {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
        min_y = min_y.min(p[1]);
        max_y = max_y.max(p[1]);
    }
    assert!(
        min_x < -4.5 && max_x > 4.5 && min_y < -4.5 && max_y > 4.5,
        "centered bbox: x={min_x}..{max_x}, y={min_y}..{max_y}"
    );
}

#[test]
fn sweep_extrude_basic() {
    // 円形 profile を +Z 方向の直線 path 沿いに sweep。最小限の動作確認。
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
fn sweep_extrude_follows_3d_path() {
    // 3D path (0,0,0) → (0,10,0) → (10,10,10) を 1 mm 円で sweep。
    // sweep mesh が全 3 軸方向 (X, Y, Z) で path 端点 (10,10,10) 近くまで届くことで、
    // 単一平面への退化なしに 3D 追従していることを確認する。
    let src = "main = sweep_extrude_xy [p3 0.0 0.0 0.0, p3 0.0 10.0 0.0, p3 10.0 10.0 10.0] (circle 1.0)";
    let out = compile_run(src);
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0]).unwrap();
    assert!(!mesh.is_empty());
    let (mut min_y, mut max_y) = (f32::INFINITY, f32::NEG_INFINITY);
    let (mut min_z, mut max_z) = (f32::INFINITY, f32::NEG_INFINITY);
    let (mut min_x, mut max_x) = (f32::INFINITY, f32::NEG_INFINITY);
    for p in &mesh.positions {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
        min_y = min_y.min(p[1]);
        max_y = max_y.max(p[1]);
        min_z = min_z.min(p[2]);
        max_z = max_z.max(p[2]);
    }
    // path は (0,0,0) から (10,10,10) まで届くので、circle の半径 1 を考慮しても
    // X / Y / Z すべてで 9 を超えるはず。
    assert!(
        max_y > 9.0,
        "Y should reach near 10 along path bend: max_y={max_y}"
    );
    assert!(
        max_z > 9.0,
        "Z should reach near 10 along path bend: max_z={max_z}"
    );
    assert!(
        max_x > 9.0,
        "X should reach near 10 along path bend: max_x={max_x}"
    );
    // 全体 bbox が原点近傍から始まること
    assert!(min_x < 1.0 && min_y < 1.0 && min_z < 1.0);
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
    let out = cadhr_lang::run_binding(&prog, "main", &cadhr_lang::Inputs::default())
        .expect("run_binding");
    assert_eq!(out.control_points.len(), 1);
    assert_eq!(out.control_points[0].0, "corner");
    assert_eq!(out.control_points[0].1, [5.0, 7.0, 9.0]);
}

#[test]
fn control3d_overrides_default_when_inputs_provided() {
    let src = "main = cube 1.0 1.0 1.0 |> translate3d (p3 0.0 0.0 0.0) (control3d \"corner\" (p3 5.0 7.0 9.0))";
    let prog = cadhr_lang::compile(src).expect("compile");
    let mut inputs = cadhr_lang::Inputs::default();
    inputs
        .control_overrides
        .insert("corner".into(), [1.0, 2.0, 3.0]);
    let out = cadhr_lang::run_binding(&prog, "main", &inputs).expect("run_binding");
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
