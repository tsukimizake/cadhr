//! mechanism ライブラリの統合テスト。
//!
//! `cadhr-lang/library/mechanism/db.cadhr` を `#use` で読み込み、平歯車の連鎖を
//! validate + render するワークフローを通しでチェックする。

use std::collections::HashSet;
use std::path::PathBuf;

use cadhr_lang::parse::{FileRegistry, database, query};
use cadhr_lang::module::resolve_modules;
use cadhr_lang::term_rewrite::execute;

fn library_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("library")
}

fn run(db_src: &str, query_src: &str) -> Result<Vec<String>, String> {
    let clauses = database(db_src).map_err(|e| format!("parse db: {:?}", e))?;
    let mut visited = HashSet::new();
    let mut registry = FileRegistry::new();
    let resolved_clauses =
        resolve_modules(clauses, &[library_root()], &mut visited, &mut registry)
            .map_err(|e| format!("resolve modules: {}", e))?;
    let mut db = resolved_clauses;
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
#use("mechanism").
main :- mechanism::do_mech([
    mechanism::gp(g1, 1, 20, 0,  0, 0,  5),
    mechanism::gp(g2, 1, 40, 30, 0, T2, 5),
    mechanism::gp(g3, 1, 20, 60, 0, T3, 5)
]).
"#;
    let resolved = run(src, "main.").expect("should succeed");
    // 各歯車が translate(rotate(cylinder(...), ...), x, y, 0) として 3 個描画される
    let render_count = resolved.iter().filter(|s| s.starts_with("translate(")).count();
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
#use("mechanism").
main(T1) :- mechanism::do_mech([
    mechanism::gp(g1, 1, 20, 0,  0, T1, 5),
    mechanism::gp(g2, 1, 40, 30, 0, T2, 5)
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
#use("mechanism").
main(T1) :- mechanism::do_mech([
    mechanism::gp(g1, 1, 20, 0,  0, T1, 5),
    mechanism::gp(g2, 1, 40, 30, 0, T2, 5),
    mechanism::gp(g3, 1, 20, 60, 0, T3, 5)
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

/// 非完全整除ケース: 歯数比が割り切れない (20-40-30) 場合でも、線形ソルバの
/// 丸め許容により T3 が近似値 (13.33) で束縛されて連鎖が完了する。
/// 真の値 13.333... を FixedPoint 2 桁に丸めた結果。
#[test]
fn driver_rotation_propagates_with_non_exact_division() {
    let src = r#"
#use("mechanism").
main(T1) :- mechanism::do_mech([
    mechanism::gp(g1, 1, 20, 0,  0, T1, 5),
    mechanism::gp(g2, 1, 40, 30, 0, T2, 5),
    mechanism::gp(g3, 1, 30, 65, 0, T3, 5)
]).
"#;
    let resolved = run(src, "main(20).").expect("should succeed");
    let joined = resolved.join("\n");
    // T1=20, T2=-10 (exact), T3 = -40*(-10)/30 = 13.333... → 13.33 (rounded)
    assert!(joined.contains("-10"), "expected T2=-10 in: {}", joined);
    assert!(
        joined.contains("13.33"),
        "expected T3≈13.33 in: {}",
        joined
    );
}

/// 中心距離を規格から外すと assert_eq("gear center distance", ...) で失敗する。
/// g2 を X=31 に置く (期待値 30)。エラーメッセージにラベルと数値が含まれる。
#[test]
fn distance_mismatch_reports_labeled_error() {
    let src = r#"
#use("mechanism").
main :- mechanism::do_mech([
    mechanism::gp(g1, 1, 20, 0,  0, 0,  5),
    mechanism::gp(g2, 1, 40, 31, 0, T2, 5)
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
#use("mechanism").
main :- mechanism::do_mech([
    mechanism::gp(g1, 1, 20, 0,  0, 0,  5),
    mechanism::gp(g2, 2, 20, 30, 0, T2, 5)
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
