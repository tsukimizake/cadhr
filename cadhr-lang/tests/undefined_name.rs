//! 未定義の変数 / コンストラクタは compile を失敗させる (実行時まで持ち越さない)。
//! 型推論の他のエラーは warning に降格されるが、未定義名だけは fatal 扱い。

use cadhr_lang::compile;

#[test]
fn undefined_variable_is_fatal() {
    let src = "f : Int -> Int\n\
        f x = List.drop x\n\
        \n\
        main = { models = [], bom = [], controls = [] }\n";
    let err = compile(src).expect_err("未定義変数で compile は失敗するはず");
    assert!(
        err.iter().any(|d| d.message.contains("List.drop")),
        "{:?}",
        err.iter().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
}

#[test]
fn undefined_constructor_is_fatal() {
    let src = "main =\n\
        \x20   let v = Nope 1 in\n\
        \x20   { models = [], bom = [], controls = [] }\n";
    let err = compile(src).expect_err("未定義コンストラクタで compile は失敗するはず");
    assert!(
        err.iter().any(|d| d.message.contains("Nope")),
        "{:?}",
        err.iter().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
}
