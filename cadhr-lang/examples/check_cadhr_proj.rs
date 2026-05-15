//! cadhr-proj 配下の db.cadhr が新 translate API でパース・実行できるか確認する。
//!
//! 実行: cargo run --example check_cadhr_proj
//!
//! cadhr-proj は本リポ外なのでテストには入れず、移行時の確認用としてのみ提供。

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use cadhr_lang::manifold_bridge::Model3D;
use cadhr_lang::module::resolve_modules;
use cadhr_lang::parse::{Clause, FileRegistry, Term, database, query};
use cadhr_lang::term_rewrite::execute;

enum CheckResult {
    Main { terms: usize, shapes: usize },
    Library,
}

fn check_one(db_path: &Path) -> Result<CheckResult, String> {
    let src = std::fs::read_to_string(db_path).map_err(|e| format!("read: {}", e))?;
    let clauses = database(&src).map_err(|e| format!("parse db: {:?}", e))?;
    let mut visited = HashSet::new();
    let mut registry = FileRegistry::new();
    // include_paths: db.cadhr の親ディレクトリ。#use("../bolts") は親 → 隣接 dir
    // という相対指定なので、project_dir を渡せば cadhr-proj/<other> を辿れる。
    let project_dir = db_path
        .parent()
        .ok_or_else(|| "no parent dir".to_string())?
        .to_path_buf();
    let include_paths = vec![project_dir];
    let mut db = resolve_modules(clauses, &include_paths, &mut visited, &mut registry)
        .map_err(|e| format!("resolve modules: {}", e))?;
    // main がなければライブラリ扱い (parse だけ通れば OK)
    let has_main = db.iter().any(|c| match c {
        Clause::Fact(t) | Clause::Rule { head: t, .. } => {
            matches!(t, Term::Struct { functor, .. } if functor == "main")
        }
        Clause::Use { .. } => false,
    });
    if !has_main {
        return Ok(CheckResult::Library);
    }
    let (_, q) = query("main.").map_err(|e| format!("parse query: {:?}", e))?;
    let (resolved, _) = execute(&mut db, q).map_err(|e| format!("execute: {}", e))?;
    let shape_count = resolved
        .iter()
        .filter(|t| Model3D::from_term(t).is_ok())
        .count();
    Ok(CheckResult::Main {
        terms: resolved.len(),
        shapes: shape_count,
    })
}

fn main() {
    let project_root = PathBuf::from(std::env::var("HOME").expect("HOME")).join("cadhr-proj");
    if !project_root.is_dir() {
        eprintln!("not found: {:?}", project_root);
        std::process::exit(1);
    }
    let mut entries: Vec<_> = std::fs::read_dir(&project_root)
        .expect("readdir")
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_dir())
        .filter_map(|e| {
            let p = e.path().join("db.cadhr");
            if p.exists() { Some(p) } else { None }
        })
        .collect();
    entries.sort();
    let mut ok = 0usize;
    let mut fail = 0usize;
    for db_path in &entries {
        let rel = db_path
            .strip_prefix(&project_root)
            .unwrap_or(db_path)
            .display();
        match check_one(db_path) {
            Ok(CheckResult::Main { terms, shapes }) => {
                println!("[ok]   {} (terms={}, shapes={})", rel, terms, shapes);
                ok += 1;
            }
            Ok(CheckResult::Library) => {
                println!("[lib]  {} (parse-only; no main)", rel);
                ok += 1;
            }
            Err(e) => {
                println!("[fail] {}: {}", rel, e);
                fail += 1;
            }
        }
    }
    println!("\nsummary: {} ok, {} fail", ok, fail);
    if fail > 0 {
        std::process::exit(1);
    }
}
