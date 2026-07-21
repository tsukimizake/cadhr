//! cadhr-lang CLI。
//!
//! ```text
//! cadhr-lang check <path>...
//! ```
//!
//! `<path>` には db.cadhr ファイル・プロジェクトディレクトリ (直下に db.cadhr) ・
//! プロジェクト群のルート (子ディレクトリごとに db.cadhr) のいずれかを渡せる。
//! 各プロジェクトを compile し、`main` binding があれば実行してメッシュ評価まで行う。
//! `main` が無いものはライブラリとして compile 検査のみ。1 つでも失敗すれば exit 1。

use std::path::{Path, PathBuf};

use cadhr_lang::syntax::ast::Decl;
use cadhr_lang::{Inputs, Severity, compile_with_paths, run_binding};

enum CheckOutcome {
    Main {
        models: usize,
        triangles: usize,
        bom: usize,
        control_points: Vec<(String, [f64; 3])>,
        warnings: usize,
    },
    Library {
        warnings: usize,
    },
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.split_first() {
        Some((cmd, rest)) if cmd == "check" && !rest.is_empty() => {
            let code = run_check(rest);
            std::process::exit(code);
        }
        _ => {
            eprintln!("usage: cadhr-lang check <path>...");
            eprintln!("  <path>: db.cadhr / プロジェクトディレクトリ / プロジェクト群ルート");
            std::process::exit(2);
        }
    }
}

fn run_check(paths: &[String]) -> i32 {
    let mut targets: Vec<PathBuf> = Vec::new();
    for p in paths {
        let path = PathBuf::from(p);
        if !path.exists() {
            eprintln!("not found: {}", path.display());
            return 2;
        }
        targets.extend(collect_db_files(&path));
    }
    if targets.is_empty() {
        eprintln!("db.cadhr が見つかりませんでした");
        return 2;
    }

    let mut ok = 0usize;
    let mut fail = 0usize;
    for db_path in &targets {
        match check_one(db_path) {
            Ok(CheckOutcome::Main {
                models,
                triangles,
                bom,
                control_points,
                warnings,
            }) => {
                println!(
                    "[ok]   {} (models={models}, tris={triangles}, bom={bom}, cp={}, warn={warnings})",
                    db_path.display(),
                    control_points.len(),
                );
                for (name, p) in &control_points {
                    println!("         cp: {name} = ({:.1}, {:.1}, {:.1})", p[0], p[1], p[2]);
                }
                ok += 1;
            }
            Ok(CheckOutcome::Library { warnings }) => {
                println!(
                    "[lib]  {} (compile-only; no main, warn={warnings})",
                    db_path.display()
                );
                ok += 1;
            }
            Err(e) => {
                println!("[fail] {}: {e}", db_path.display());
                fail += 1;
            }
        }
    }
    println!("\nsummary: {ok} ok, {fail} fail");
    if fail > 0 { 1 } else { 0 }
}

/// path から検査対象の db.cadhr を集める。
/// - ファイル → それ自体
/// - db.cadhr を直下に持つディレクトリ → その db.cadhr
/// - それ以外のディレクトリ → `<子ディレクトリ>/db.cadhr` を走査
fn collect_db_files(path: &Path) -> Vec<PathBuf> {
    if path.is_file() {
        return vec![path.to_path_buf()];
    }
    let direct = path.join("db.cadhr");
    if direct.is_file() {
        return vec![direct];
    }
    let mut found: Vec<PathBuf> = std::fs::read_dir(path)
        .into_iter()
        .flatten()
        .filter_map(|e| e.ok())
        .map(|e| e.path().join("db.cadhr"))
        .filter(|p| p.is_file())
        .collect();
    found.sort();
    found
}

/// GUI (`search_paths`) と同じ規則: プロジェクトの親ディレクトリ (隣接プロジェクト・
/// 共有 Std 用) とプロジェクト自身。fallback として同梱 std を最後に足す。
fn search_paths_for(db_path: &Path) -> Vec<PathBuf> {
    let mut paths = Vec::new();
    if let Some(project_dir) = db_path.parent() {
        if let Some(root) = project_dir.parent() {
            paths.push(root.to_path_buf());
        }
        paths.push(project_dir.to_path_buf());
    }
    paths.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("std"));
    paths
}

fn check_one(db_path: &Path) -> Result<CheckOutcome, String> {
    let src = std::fs::read_to_string(db_path).map_err(|e| format!("read: {e}"))?;
    let search_paths = search_paths_for(db_path);

    let prog = compile_with_paths(&src, &search_paths).map_err(|diags| {
        diags
            .iter()
            .map(|d| format!("{:?}: {} (span={:?})", d.severity(), d.message(), d.span()))
            .collect::<Vec<_>>()
            .join("; ")
    })?;

    let warnings = prog
        .diagnostics
        .iter()
        .filter(|d| d.severity() != Severity::Error)
        .count();
    for d in &prog.diagnostics {
        eprintln!(
            "         diag: {} ({:?}, span={:?})",
            d.message(),
            d.severity(),
            d.span()
        );
    }

    let main_module = &prog.unit.modules[prog.unit.main_index].module;
    let has_main = main_module.decls.iter().any(|d| {
        matches!(d, Decl::Value(v) | Decl::Var(v) if v.name == "main")
    });
    if !has_main {
        return Ok(CheckOutcome::Library { warnings });
    }

    let inputs = Inputs {
        search_paths: search_paths.clone(),
        ..Default::default()
    };
    let out = run_binding(&prog, "main", &inputs).map_err(|d| format!("run main: {}", d.message()))?;

    let mut triangles = 0usize;
    #[cfg(feature = "manifold")]
    for m in &out.models {
        let arr = cadhr_lang::runtime::manifold_bridge::to_mesh_arrays_with_paths(m, &search_paths)
            .map_err(|e| format!("mesh: {e:?}"))?;
        triangles += arr.indices.len() / 3;
    }

    Ok(CheckOutcome::Main {
        models: out.models.len(),
        triangles,
        bom: out.bom.len(),
        control_points: out.control_points,
        warnings,
    })
}
