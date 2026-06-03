//! 型署名のパラメータ型を body 推論の前に束縛することの確認。
//! 署名でパラメータが record 型と分かれば、body 内の `v.x` がフィールドアクセスとして
//! 解決され、「レコードでないものから `.x` を取れません」警告が出ない。

use cadhr_lang::compile;

#[test]
fn signature_drives_param_field_access() {
    let src = "type alias Vec2 = { x : Float, y : Float }\n\
        \n\
        first : Vec2 -> Float\n\
        first v =\n\
        \x20   v.x\n\
        \n\
        main =\n\
        \x20   let r = first { x = 1.0, y = 2.0 } in\n\
        \x20   { models = [], bom = [], controls = [] }\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "署名から param 型が決まり警告が出ないはず: {:?}",
        prog.diagnostics
    );
}
