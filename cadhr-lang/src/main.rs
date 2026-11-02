//! cadhr-lang CLI。
//!
//! ```text
//! cadhr-lang check <path>... [--binding <name>]...
//! ```
//!
//! `<path>` には db.cadhr ファイル・プロジェクトディレクトリ (直下に db.cadhr) ・
//! プロジェクト群のルート (子ディレクトリごとに db.cadhr) のいずれかを渡せる。
//! 各プロジェクトを compile し、binding を実行してメッシュ評価まで行う。
//! 実行する binding は `--binding` で指定 (複数可)。未指定なら `main` (無ければ
//! ライブラリとして compile 検査のみ)。1 つでも失敗すれば exit 1。
//! sketch の書き戻し先が曖昧なハンドル軸は warning として表示する。

mod export;

use std::path::{Path, PathBuf};

use cadhr_lang::syntax::ast::Decl;
use cadhr_lang::{CompiledProgram, Inputs, Severity, compile_with_paths, run_binding, sketch};
use clap::Parser;

#[derive(Parser)]
#[command(name = "cadhr-lang")]
enum Cli {
    /// compile + 実行検査。sketch の書き戻し先が曖昧な軸は warning 表示する
    Check {
        /// db.cadhr / プロジェクトディレクトリ / プロジェクト群ルート
        #[arg(required = true)]
        paths: Vec<PathBuf>,
        /// 実行する binding (複数可)。未指定なら main (無ければ compile 検査のみ)
        #[arg(long, short)]
        binding: Vec<String>,
    },
    #[command(about = "binding を 3MF ファイルに出力する")]
    Export {
        #[arg(help = "db.cadhr / プロジェクトディレクトリ")]
        path: PathBuf,
        #[arg(long, short, default_value = "main", help = "出力する binding")]
        binding: String,
        #[arg(long, short, help = "出力先の 3MF ファイル")]
        output: PathBuf,
    },
}

enum CheckOutcome {
    Run {
        results: Vec<BindingResult>,
        warnings: usize,
    },
    Library {
        warnings: usize,
    },
}

struct BindingResult {
    binding: String,
    models: usize,
    triangles: usize,
    bom: usize,
    control_points: Vec<(String, [f64; 3])>,
}

fn main() {
    std::process::exit(match Cli::parse() {
        Cli::Check { paths, binding } => run_check(&paths, &binding),
        Cli::Export {
            path,
            binding,
            output,
        } => run_export(&path, &binding, &output),
    });
}

fn run_export(path: &Path, binding: &str, output: &Path) -> i32 {
    let db_path = if path.is_dir() {
        path.join("db.cadhr")
    } else {
        path.to_path_buf()
    };
    if !db_path.is_file() {
        eprintln!("not found: {}", db_path.display());
        return 2;
    }
    match export_one(&db_path, binding, output) {
        Ok(()) => {
            println!(
                "[ok]   {} ({binding}) -> {}",
                db_path.display(),
                output.display()
            );
            0
        }
        Err(e) => {
            eprintln!("[fail] {}: {e}", db_path.display());
            1
        }
    }
}

fn export_one(db_path: &Path, binding: &str, output: &Path) -> Result<(), String> {
    let search_paths = search_paths_for(db_path);
    let (prog, _) = compile_file(db_path, &search_paths)?;
    let inputs = Inputs {
        search_paths,
        ..Default::default()
    };
    let out = run_binding(&prog, binding, &inputs)
        .map_err(|d| format!("run {binding}: {}", d.message()))?;
    let data = export::models_to_threemf(&out.models, &inputs.search_paths)
        .map_err(|e| format!("3MF ({binding}): {e}"))?;
    std::fs::write(output, data).map_err(|e| format!("write {}: {e}", output.display()))
}

fn run_check(paths: &[PathBuf], bindings: &[String]) -> i32 {
    let mut targets: Vec<PathBuf> = Vec::new();
    for path in paths {
        if !path.exists() {
            eprintln!("not found: {}", path.display());
            return 2;
        }
        targets.extend(collect_db_files(path));
    }
    if targets.is_empty() {
        eprintln!("db.cadhr が見つかりませんでした");
        return 2;
    }

    let mut ok = 0usize;
    let mut fail = 0usize;
    for db_path in &targets {
        match check_one(db_path, bindings) {
            Ok(CheckOutcome::Run { results, warnings }) => {
                println!("[ok]   {} (warn={warnings})", db_path.display());
                for r in &results {
                    println!(
                        "         {}: models={}, tris={}, bom={}, cp={}",
                        r.binding,
                        r.models,
                        r.triangles,
                        r.bom,
                        r.control_points.len(),
                    );
                    for (name, p) in &r.control_points {
                        println!(
                            "           cp: {name} = ({:.1}, {:.1}, {:.1})",
                            p[0], p[1], p[2]
                        );
                    }
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

/// `offset` を含む行の 1 始まり行番号。
fn line_of(src: &str, offset: usize) -> usize {
    src[..offset.min(src.len())].matches('\n').count() + 1
}

fn compile_file(
    db_path: &Path,
    search_paths: &[PathBuf],
) -> Result<(CompiledProgram, usize), String> {
    let src = std::fs::read_to_string(db_path).map_err(|e| format!("read: {e}"))?;
    let prog = compile_with_paths(&src, search_paths).map_err(|diags| {
        diags
            .iter()
            .map(|d| format!("{:?}: {} (span={:?})", d.severity(), d.message(), d.span()))
            .collect::<Vec<_>>()
            .join("; ")
    })?;

    let mut warnings = prog
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

    // sketch ハンドルの書き戻し先が曖昧な軸を warning として表示する。
    let main_module = &prog.unit.modules[prog.unit.main_index].module;
    for w in sketch::writeback_ambiguities(main_module) {
        eprintln!(
            "         warn: {}:{}: {}",
            db_path.display(),
            line_of(&src, w.span.start),
            w.message
        );
        warnings += 1;
    }

    Ok((prog, warnings))
}

fn check_one(db_path: &Path, bindings: &[String]) -> Result<CheckOutcome, String> {
    let search_paths = search_paths_for(db_path);
    let (prog, warnings) = compile_file(db_path, &search_paths)?;
    let main_module = &prog.unit.modules[prog.unit.main_index].module;

    // 実行する binding: 指定があればそれ、無ければ main (無ければ compile 検査のみ)。
    let run_targets: Vec<String> = if bindings.is_empty() {
        let has_main = main_module
            .decls
            .iter()
            .any(|d| matches!(d, Decl::Value(v) if v.name == "main"));
        if !has_main {
            return Ok(CheckOutcome::Library { warnings });
        }
        vec!["main".to_string()]
    } else {
        bindings.to_vec()
    };

    let inputs = Inputs {
        search_paths: search_paths.clone(),
        ..Default::default()
    };
    let mut results = Vec::new();
    for binding in &run_targets {
        let out = run_binding(&prog, binding, &inputs)
            .map_err(|d| format!("run {binding}: {}", d.message()))?;

        let mut triangles = 0usize;
        for m in &out.models {
            let arr =
                cadhr_lang::runtime::manifold_bridge::to_mesh_arrays_with_paths(m, &search_paths)
                    .map_err(|e| format!("mesh ({binding}): {e:?}"))?;
            triangles += arr.indices.len() / 3;
        }

        results.push(BindingResult {
            binding: binding.clone(),
            models: out.models.len(),
            triangles,
            bom: out.bom.len(),
            control_points: out.control_points,
        });
    }

    Ok(CheckOutcome::Run { results, warnings })
}
