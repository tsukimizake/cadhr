//! Point2D / Point3D は構造的 record 型。`.x` 等のフィールドアクセスと、
//! record リテラルを point として渡せることを確認する。

use cadhr_lang::{Inputs, compile, run_binding};

#[test]
fn point_field_access_when_type_known() {
    // p の型は p2 から record と分かるので、署名なしでも `.x` / `.y` が引ける。
    let src = "main =\n\
        \x20   let\n\
        \x20       p = p2 3.0 4.0\n\
        \x20       q = p3 p.x p.y 0.0\n\
        \x20   in\n\
        \x20   { models = [], bom = [], controls = [] }\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
}

#[test]
fn record_literal_is_accepted_as_point2d() {
    // polygon : List Point2D。record リテラルをそのまま point として渡せる。
    let src = "main =\n\
        \x20   let\n\
        \x20       s = polygon [{ x = 0.0, y = 0.0 }, { x = 10.0, y = 0.0 }, { x = 5.0, y = 8.0 }]\n\
        \x20       m = extrude_xy 3.0 s\n\
        \x20   in\n\
        \x20   { models = [m], bom = [], controls = [] }\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
    let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
    assert_eq!(out.models.len(), 1);
}
