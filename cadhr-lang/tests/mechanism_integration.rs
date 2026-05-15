//! mechanism 統合テスト。
//!
//! `cadhr-lang/tests/mechanism.cadhr` を include_str! で取り込み、平歯車の連鎖を
//! validate + render するワークフローを通しでチェックする。

use cadhr_lang::parse::{database, query};
use cadhr_lang::term_rewrite::execute;

const MECHANISM_SRC: &str = include_str!("mechanism.cadhr");

fn run(extra_db: &str, query_src: &str) -> Result<Vec<String>, String> {
    let full_db = format!("{}\n{}", MECHANISM_SRC, extra_db);
    let mut db = database(&full_db).map_err(|e| format!("parse db: {:?}", e))?;
    let q = query(query_src).map_err(|e| format!("parse query: {:?}", e))?.1;
    let (resolved, _) = execute(&mut db, q).map_err(|e| format!("execute: {}", e))?;
    Ok(resolved.iter().map(|t| format!("{:?}", t)).collect())
}

/// 3 段ギアトレインが規格通り配置されたとき、矛盾なく chain を validate できる。
/// T1 を 0 にすると T2, T3 もすべて 0 になり、各歯車は静止位置で描画される。
/// (歯数 20-40-20: 連動回転が FixedPoint で完全整除になる組み合わせ)
#[test]
fn three_gear_chain_validates_and_renders() {
    let src = r#"
main :- do_mech([
    gp(g1, 1, 20, 0,  0, 0,  5),
    gp(g2, 1, 40, 30, 0, T2, 5),
    gp(g3, 1, 20, 60, 0, T3, 5)
]).
"#;
    let resolved = run(src, "main.").expect("should succeed");
    // 各歯車が translate(rotate(cylinder(...), ...), p(0,0,0), p(x, y, 0)) として 3 個描画される
    let render_count = resolved
        .iter()
        .filter(|s| s.starts_with("translate("))
        .count();
    assert_eq!(
        render_count, 3,
        "expected 3 translated cylinders, got {:#?}",
        resolved
    );
}

/// 駆動歯車の回転角を非ゼロにすると、最初の隣接ペアの噛み合い比で T2 が連動する。
#[test]
fn driver_rotation_propagates_to_first_pair() {
    let src = r#"
main(T1) :- do_mech([
    gp(g1, 1, 20, 0,  0, T1, 5),
    gp(g2, 1, 40, 30, 0, T2, 5)
]).
"#;
    let resolved = run(src, "main(20).").expect("should succeed");
    let joined = resolved.join("\n");
    // T1=20, T2 = -Z1*T1/Z2 = -20*20/40 = -10
    assert!(joined.contains("-10"), "expected T2=-10 in: {}", joined);
}

/// 駆動歯車の回転を 3 段チェーンで伝播させる (完全整除ケース)。
/// Z1=20, Z2=40 → T2 = -Z1*T1/Z2 = -20*20/40 = -10
/// Z2=40, Z3=20 → T3 = -Z2*T2/Z3 = -40*(-10)/20 = 20
#[test]
fn driver_rotation_propagates_through_three_gear_chain() {
    let src = r#"
main(T1) :- do_mech([
    gp(g1, 1, 20, 0,  0, T1, 5),
    gp(g2, 1, 40, 30, 0, T2, 5),
    gp(g3, 1, 20, 60, 0, T3, 5)
]).
"#;
    let resolved = run(src, "main(20).").expect("should succeed");
    let joined = resolved.join("\n");
    assert!(joined.contains("-10"), "expected T2=-10 in: {}", joined);
    assert!(
        joined.contains("0, 0, 20)"),
        "expected T3=20 in g3's rotate(...) in: {}",
        joined
    );
}

/// 非完全整除ケース: 歯数比が割り切れない (20-40-30) 場合でも、Rational
/// による厳密演算で T3 が連鎖伝播する。
/// 真の値 40/3 が Display で 13.33 と丸められて表示されるが、内部は厳密。
#[test]
fn driver_rotation_propagates_with_non_exact_division() {
    let src = r#"
main(T1) :- do_mech([
    gp(g1, 1, 20, 0,  0, T1, 5),
    gp(g2, 1, 40, 30, 0, T2, 5),
    gp(g3, 1, 30, 65, 0, T3, 5)
]).
"#;
    let resolved = run(src, "main(20).").expect("should succeed");
    let joined = resolved.join("\n");
    // T1=20, T2=-10 (exact), T3 = -40*(-10)/30 = 40/3 = 13.33 (rounded display)
    assert!(joined.contains("-10"), "expected T2=-10 in: {}", joined);
    assert!(
        joined.contains("13.33"),
        "expected T3≈13.33 in: {}",
        joined
    );
}

/// 5 段チェーン (20-30-40-25-30) — 隣接歯数比が割り切れない組み合わせを
/// 連発しても誤差が累積せず、最終 T が厳密に解ける。
/// 真値:
///   T1 = 30
///   T2 = -Z1*T1/Z2 = -20*30/30 = -20
///   T3 = -Z2*T2/Z3 = -30*(-20)/40 = 15
///   T4 = -Z3*T3/Z4 = -40*15/25 = -24
///   T5 = -Z4*T4/Z5 = -25*(-24)/30 = 20
///
/// 中心距離は 1*(Z1+Z2)/2 = 25, 35, 32.5, 27.5 → 累積位置 0, 25, 60, 92.5, 120
/// 92.5 は 0.5 で表せるので問題なく扱える (Rational なら厳密)。
#[test]
fn five_gear_chain_no_error_accumulation() {
    let src = r#"
main(T1) :- do_mech([
    gp(g1, 1, 20, 0,    0, T1, 5),
    gp(g2, 1, 30, 25,   0, T2, 5),
    gp(g3, 1, 40, 60,   0, T3, 5),
    gp(g4, 1, 25, 92.5, 0, T4, 5),
    gp(g5, 1, 30, 120,  0, T5, 5)
]).
"#;
    let resolved = run(src, "main(30).").expect("should succeed");
    let joined = resolved.join("\n");
    // T2=-20, T3=15, T4=-24, T5=20 のすべてが正確に解ける
    assert!(
        joined.contains(", 0, -20),"),
        "expected T2=-20 in: {}",
        joined
    );
    assert!(
        joined.contains(", 0, 15),"),
        "expected T3=15 in: {}",
        joined
    );
    assert!(
        joined.contains(", 0, -24),"),
        "expected T4=-24 in: {}",
        joined
    );
    assert!(
        joined.contains(", 0, 20),"),
        "expected T5=20 in: {}",
        joined
    );
}

/// 真の矛盾は依然として検出される。歯車のモジュールを誤って異なるものにすると
/// mesh/2 の head パターンで unify が失敗する。
/// 一方、Rational 化で「丸め誤差由来の偽の不整合」が出ない (FixedPoint 時代の
/// tolerance ロジックが原因で起きえた偽陽性が完全に解消する)。
#[test]
fn rational_does_not_introduce_false_contradictions() {
    // 距離・角度がすべて非整除な値だが整合している。Rational なら通過する。
    // (旧 FixedPoint では DistSq*DistSq vs R*R の丸めで偽の不整合が出る可能性があった)
    let src = r#"
main(T1) :- do_mech([
    gp(g1, 1, 7,  0,    0, T1, 5),
    gp(g2, 1, 11, 9,    0, T2, 5)
]).
"#;
    // Z1+Z2 = 18, M=1, center distance = 9. 9*9 = 81. DistSq = 9*9 = 81. 一致。
    let resolved = run(src, "main(0).").expect("should succeed");
    let render_count = resolved
        .iter()
        .filter(|s| s.starts_with("translate("))
        .count();
    assert_eq!(render_count, 2, "expected 2 gears rendered");
}

/// 中心距離を規格から外すと assert_eq("gear center distance", ...) で失敗する。
/// g2 を X=31 に置く (期待値 30)。エラーメッセージにラベルと数値が含まれる。
#[test]
fn distance_mismatch_reports_labeled_error() {
    let src = r#"
main :- do_mech([
    gp(g1, 1, 20, 0,  0, 0,  5),
    gp(g2, 1, 40, 31, 0, T2, 5)
]).
"#;
    let err = run(src, "main.").expect_err("distance mismatch should fail");
    assert!(
        err.contains("gear center distance"),
        "expected label 'gear center distance' in: {}",
        err
    );
    // 期待 900, 実測 961
    assert!(
        err.contains("900") && err.contains("961"),
        "expected expected/actual values 900/961 in: {}",
        err
    );
}

/// モジュール不一致は head のパターンマッチで弾かれる。
/// (gp(_, M, ...) を使う mesh/2 の head で同じ M を要求するため、unify 失敗となる)
#[test]
fn module_mismatch_fails_to_unify_mesh_head() {
    let src = r#"
main :- do_mech([
    gp(g1, 1, 20, 0,  0, 0,  5),
    gp(g2, 2, 20, 30, 0, T2, 5)
]).
"#;
    let err = run(src, "main.").expect_err("module mismatch should fail");
    // mesh/2 の head の M で構造マッチが起きないため、no clause matches となる
    assert!(
        err.contains("no clause matches") || err.contains("unify"),
        "expected unify/no-clause error: {}",
        err
    );
}
