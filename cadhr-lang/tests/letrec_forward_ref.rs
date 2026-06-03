//! トップレベルの前方参照と再帰。eager な `main` が後方定義の関数を参照でき、
//! 再帰関数 `get` が評価できることを確認する。

use cadhr_lang::{Inputs, compile, run_main};

#[test]
fn main_forward_refs_recursive_function() {
    // main は get より前に定義されているが、get (関数) を前方参照できる。
    // get 1 [10,20,30] == 20 → cube 20.0 1.0 1.0 が 1 個。
    let src = "main =\n\
        \x20   let v = get 1 [10, 20, 30] in\n\
        \x20   { models = [cube (fromInt v) 1.0 1.0], bom = [], controls = [] }\n\
        \n\
        get : Int -> List Int -> Int\n\
        get n xs =\n\
        \x20   case xs of\n\
        \x20       | [] -> 0\n\
        \x20       | x :: rest -> if n == 0 then x else get (n - 1) rest\n";
    let prog = compile(src).expect("compile");
    assert!(prog.diagnostics.is_empty(), "診断は無いはず: {:?}", prog.diagnostics);
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1);
}
