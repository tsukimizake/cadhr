//! `if cond then` / `else` の直後・前に改行を置いて分岐本体を次行に書けること。

use cadhr_lang::compile;

#[test]
fn if_then_else_branches_on_next_lines() {
    let src = "pick : Int -> Int\n\
        pick n =\n\
        \x20   if n == 0 then\n\
        \x20       10\n\
        \x20   else\n\
        \x20       20\n\
        \n\
        main =\n\
        \x20   { models = [cube (fromInt (pick 0)) 1.0 1.0], bom = [], controls = [] }\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
}
