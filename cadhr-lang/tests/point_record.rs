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

#[test]
fn field_access_on_lambda_param_resolved_later() {
    // `map (\p -> p.x) xs` のラムダ本体を推論する時点では p の型はまだ型変数。
    // 要素型 (Point2D) は list 引数との単一化で後から決まる。フィールドアクセスの
    // 解決を SCC 境界に遅延することで、署名なしでも通るべき (旧: NotARecord)。
    let src = "\
map : (a -> b) -> List a -> List b\n\
map f xs =\n\
    case xs of\n\
        | [] -> []\n\
        | x :: rest -> f x :: map f rest\n\
\n\
main =\n\
    let\n\
        pts = [p2 1.0 2.0, p2 3.0 4.0]\n\
        ys = pts |> map (\\p -> p3 p.x p.y 0.0)\n\
    in\n\
    { models = [], bom = [], controls = [] }\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
}

#[test]
fn deferred_field_access_still_rejects_missing_field() {
    // 遅延解決しても、解決後に存在しないフィールドはきちんとエラーになること。
    let src = "\
map : (a -> b) -> List a -> List b\n\
map f xs =\n\
    case xs of\n\
        | [] -> []\n\
        | x :: rest -> f x :: map f rest\n\
\n\
main =\n\
    let\n\
        pts = [p2 1.0 2.0]\n\
        ys = pts |> map (\\p -> p.z)\n\
    in\n\
    { models = [], bom = [], controls = [] }\n";
    let diags = match compile(src) {
        Ok(prog) => prog.diagnostics,
        Err(ds) => ds,
    };
    assert!(
        !diags.is_empty(),
        "存在しないフィールド .z はエラーになるべき"
    );
}
