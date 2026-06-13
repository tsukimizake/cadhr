//! case 式と型推論バグの再現テスト。
//!
//! - `Expr::Case` が以前 TODO のまま fresh 型変数を返すだけで内部を一切型検査
//!   していなかったため、case 本体の型不一致や case 内の if 分岐の型不一致が
//!   素通りしていた。それを検知できるようにしたことの確認。
//! - 算術演算子 `+ - * /` を `Num a => a -> a -> a` に多相化したことで、Std.List.get
//!   のような `index - 1` (Int) も `1.0 - 0.5` (Float) も同じ実装で typecheck できる。

use cadhr_lang::compile;

fn diag_strings(src: &str) -> Vec<String> {
    match compile(src) {
        Ok(prog) => prog
            .diagnostics
            .iter()
            .map(|d| format!("{d:?}"))
            .collect(),
        Err(ds) => ds.iter().map(|d| format!("{d:?}")).collect(),
    }
}

#[test]
fn partial_app_in_recursion_should_fail_typecheck() {
    // get の再帰で default 引数を渡し忘れ → if の then 枝が `a`、else 枝が `a -> a`
    // となるべき型不一致。以前は case が型検査されず素通りしていた。
    let src = "get : Int -> List a -> a -> a
get index xs default =
    case xs of
        [] ->
            default
        x :: rest ->
            if index == 0 then
                x
            else
                get (index - 1) rest

main =
    { models = [cube 1.0 1.0 1.0], bom = [], controls = [] }
";
    let diags = diag_strings(src);
    assert!(
        !diags.is_empty(),
        "再帰の部分適用で型エラーが出るべき"
    );
}

#[test]
fn nonrec_if_branch_mismatch_should_fail() {
    // 関数本体直下の if の分岐型不一致は元々検出されていた (回帰防止)。
    let src = "f : Int -> Int -> Int
f a b =
    if a == 0 then b else f

main =
    { models = [cube 1.0 1.0 1.0], bom = [], controls = [] }
";
    let diags = diag_strings(src);
    assert!(!diags.is_empty(), "if 分岐の型不一致で型エラーが出るべき");
}

#[test]
fn case_branch_mismatch_should_fail() {
    // case の各アーム本体型の不一致が検出されること。
    let src = "g : Int -> Int
g n =
    case n of
        0 -> 1
        _ -> g

main =
    { models = [cube 1.0 1.0 1.0], bom = [], controls = [] }
";
    let diags = diag_strings(src);
    assert!(!diags.is_empty(), "case 分岐の型不一致で型エラーが出るべき");
}

#[test]
fn list_get_with_int_arithmetic_typechecks() {
    // Num a => a -> a -> a 化により Int の `-` も通る。
    // (以前は Sub が Float 固定だったため `index - 1` でエラーになっていた。)
    let src = "get : Int -> List a -> a -> a
get index xs default =
    case xs of
        [] ->
            default
        x :: rest ->
            if index == 0 then
                x
            else
                get (index - 1) rest default

main =
    let v = get 1 [10, 20, 30] 0 in
    { models = [cube (fromInt v) 1.0 1.0], bom = [], controls = [] }
";
    let diags = diag_strings(src);
    assert!(diags.is_empty(), "正しい get は通るべき: {diags:?}");
}

#[test]
fn float_arithmetic_still_works() {
    // Float でも従来通り動くこと。
    let src = "main =
    let v = 1.5 + 2.5 in
    { models = [cube v 1.0 1.0], bom = [], controls = [] }
";
    let diags = diag_strings(src);
    assert!(diags.is_empty(), "Float arithmetic は通るべき: {diags:?}");
}

#[test]
fn num_class_rejects_string() {
    // `Num String` は無いのでエラーになる。
    let src = "main =
    let v = \"a\" + \"b\" in
    { models = [cube 1.0 1.0 1.0], bom = [], controls = [] }
";
    let diags = diag_strings(src);
    assert!(
        diags.iter().any(|d| d.contains("NoInstance")),
        "String への Num 制約は NoInstance になるべき: {diags:?}"
    );
}
