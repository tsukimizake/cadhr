//! トップレベルの前方参照と再帰。eager な `main` が後方定義の関数を参照でき、
//! 再帰関数 `get` が評価できることを確認する。

use cadhr_lang::{Diagnostic, Inputs, compile, run_main};

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
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1);
}

/// main が後方定義の eager な値 (`pzero`) を捕捉クロージャ経由で参照するケース。
/// 元の現象: 評価器が「2 pass + 宣言順」だったため、main の評価中に pzero が
/// rec フレームに未だ無く `未定義の変数 pzero` で fail していた。
/// 修正後は eager 値の依存解析で pzero → main の順に評価され、正しく動く。
#[test]
fn main_forward_refs_eager_top_level_value() {
    let src = "main =\n\
        \x20   let holes = [1.0, 2.0, 3.0] |> mapList (\\h -> shift pzero h)\n\
        \x20       shape = cube 1.0 1.0 1.0\n\
        \x20   in { models = shape :: holes, bom = [], controls = [] }\n\
        \n\
        mapList : (a -> b) -> List a -> List b\n\
        mapList f xs =\n\
        \x20   case xs of\n\
        \x20       | [] -> []\n\
        \x20       | x :: rest -> f x :: mapList f rest\n\
        \n\
        shift : Float -> Float -> Shape3D\n\
        shift base h = cube (base + h) 1.0 1.0\n\
        \n\
        pzero : Float\n\
        pzero = 10.0\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    // cube 1 個 + map で 3 個 = 4 個
    assert_eq!(out.models.len(), 4);
}

/// eager 同士が直接相互参照すると評価不可能なので fatal にする。
/// `a = b; b = a` のような genuine cycle は runtime エラー。
/// signature の有無に依らず SCC ベース型推論を通り、runtime 側で循環を検出する。
#[test]
fn eager_cycle_is_runtime_error() {
    let src = "a = b\nb = a\nmain = { models = [], bom = [], controls = [] }\n";
    let prog = compile(src).expect("compile");
    let err: Result<_, Diagnostic> = run_main(&prog, &Inputs::default());
    let err = err.expect_err("eager 循環は runtime エラーになるはず");
    assert!(
        err.message().contains("循環") || err.message().contains("cycle"),
        "msg: {}",
        err.message()
    );
}

/// signature 無しの後方定義値を前方参照できる (SCC ベース型推論)。
/// 旧実装はソース順 1 パスだったので `UndefinedVar` で fatal だった。
#[test]
fn signature_less_forward_reference_is_typecheckable() {
    let src = "main =\n\
        \x20   { models = [cube side side side], bom = [], controls = [] }\n\
        \n\
        side = 5.0\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1);
}

/// signature 無しの相互再帰関数も SCC ベースなら推論できる。
/// `mapA` / `mapB` は同じ SCC に入って一括 unify される。
/// (この言語の `+ - * /` は Float 限定なので、数値ではなくリスト走査で組む。)
#[test]
fn signature_less_mutual_recursion_is_typecheckable() {
    let src = "main =\n\
        \x20   let xs = mapA [1.0, 2.0, 3.0] in\n\
        \x20   { models = [cube (head_or xs) 1.0 1.0], bom = [], controls = [] }\n\
        \n\
        mapA xs = case xs of\n\
        \x20   | [] -> []\n\
        \x20   | x :: rest -> x :: mapB rest\n\
        mapB xs = case xs of\n\
        \x20   | [] -> []\n\
        \x20   | x :: rest -> x :: mapA rest\n\
        \n\
        head_or xs = case xs of\n\
        \x20   | [] -> 0.0\n\
        \x20   | x :: _ -> x\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1);
}

/// signature 無しの多相ヘルパが正しく generalize される。
/// `id` を Float と Int の両方に適用できることで多相性を確認する。
#[test]
fn signature_less_polymorphic_helper_is_generalized() {
    let src = "main =\n\
        \x20   let a = id 3.0\n\
        \x20       b = id 5\n\
        \x20       c = pickFloat a (fromInt b)\n\
        \x20   in { models = [cube c c c], bom = [], controls = [] }\n\
        \n\
        id x = x\n\
        pickFloat a b = a + b\n";
    let prog = compile(src).expect("compile");
    assert!(
        prog.diagnostics.is_empty(),
        "診断は無いはず: {:?}",
        prog.diagnostics
    );
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 1);
}
