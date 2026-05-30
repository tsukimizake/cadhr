//! 旧 cadhr-proj の cross-module library パターン (bolts) を新 cadhr-lang の
//! `module Foo exposing (..)` ベースで再現できるかを確認する。
//!
//! 旧版は `#use("../bolts").` で別ディレクトリの db.cadhr を参照していた。新版は
//! `import Bolts exposing (m3)` で `<search_path>/Bolts/db.cadhr` を引く
//! (モジュール = ディレクトリ + db.cadhr の慣習)。

use cadhr_lang::{Inputs, compile_with_paths, run_main};
use std::fs;
use tempfile::tempdir;

const BOLTS_SRC: &str = r#"module Bolts exposing (..)

body r len = cylinder r len

hexhead e s k =
    let
        ne = 0.0 - e
        profile = polygon [ p2 0.0 ne, p2 (0.0 - s / 2.0) (ne / 2.0), p2 (0.0 - s / 2.0) (e / 2.0), p2 0.0 e, p2 (s / 2.0) (e / 2.0), p2 (s / 2.0) (ne / 2.0) ]
    in
    extrude_xy k profile
        |> translate3d (p3 0.0 0.0 0.0) (p3 0.0 0.0 (0.0 - k))

m3 len = union3d (hexhead 6.4 5.5 2.0) (body 1.5 len)

m3_hole len = body 1.5 len
"#;

const MAIN_SRC: &str = r#"import Bolts exposing (m3, m3_hole)

main =
    let
        bolt = m3 20.0
        hole = m3_hole 30.0
    in
    { models = [bolt, hole], bom = [], controls = [] }
"#;

#[test]
fn bolts_library_cross_module() {
    let tmp = tempdir().unwrap();
    fs::create_dir_all(tmp.path().join("Bolts")).unwrap();
    fs::write(tmp.path().join("Bolts/db.cadhr"), BOLTS_SRC).unwrap();
    let prog = compile_with_paths(MAIN_SRC, &[tmp.path().to_path_buf()]).expect("compile");
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 2);
}
