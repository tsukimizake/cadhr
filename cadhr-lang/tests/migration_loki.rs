//! loki_home_key の migration テスト。
//!
//! 旧版は `#use("../bolts")` で別ディレクトリの bolts library を参照していた。
//! 新版は同じディレクトリに `Bolts/db.cadhr` を置き `import Bolts` する形に変える。
//! 3 平面 sketch を extrude → intersect3d で 3 個の押し出しを合わせて、最後に
//! ボルト穴を diff3d で抜くという旧版の挙動を新 API で再構築する。

use cadhr_lang::{Inputs, compile_with_paths, run_main};
use std::path::PathBuf;

#[test]
fn loki_home_key_migration() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/migration_loki");
    let main_src = std::fs::read_to_string(dir.join("loki_home_key.cadhr")).unwrap();
    let prog = compile_with_paths(&main_src, &[dir]).expect("compile");
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1, "1 個の Shape3D が出るはず");
    // 実際にメッシュが生成できるか manifold-rs に通して確認 (空形状でないこと)
    let mesh = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays(&out.models[0])
        .expect("manifold evaluate");
    assert!(
        !mesh.is_empty(),
        "evaluated mesh は空でないはず: {} verts / {} idx",
        mesh.positions.len(),
        mesh.indices.len()
    );
}
