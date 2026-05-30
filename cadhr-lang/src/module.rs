//! モジュール解決 (`#use` / `import` 解決層)。
//!
//! `import Std.Gears exposing (involute)` のようなディレクティブから、サーチパス
//! 配下の `Std/Gears.cadhr` を読み込み、AST を返す。再帰的な import を辿り、
//! 依存順に並んだ `Vec<Module>` を生成する。
//!
//! 設計:
//! - main モジュールは「無名」(`header = None`) でも OK。ファイル名から推測しない。
//! - import される側は **必ず** `module Foo.Bar exposing (...)` ヘッダ付き。
//!   ヘッダの `name.segments` が import 側のドット連結と一致しなければエラー。
//! - 循環 import は検出してエラーにする (今は単純な on-the-fly DFS で stack を持つ)。
//! - search_paths 内の最初の hit を採用 (順序が優先度)。

use crate::diagnostic::{Diagnostic, Span};
use crate::syntax::ast::{Module, ModuleName};
use crate::syntax::parse::parse;
use std::collections::HashMap;
use std::path::PathBuf;

/// 解決済みコンパイル単位。`modules` は依存順 (深い import が先頭、main は末尾)。
#[derive(Debug)]
pub struct ResolvedUnit {
    pub modules: Vec<LoadedModule>,
    /// main モジュール (`modules` の最後の要素) のインデックス。
    pub main_index: usize,
}

#[derive(Debug)]
pub struct LoadedModule {
    /// 「ドット連結された」モジュール名。匿名 main は `""`。
    pub qualified_name: String,
    pub module: Module,
    /// このモジュールを読み込んだファイルパス (テスト / エラー表示用)。
    pub source_path: Option<PathBuf>,
}

pub struct Resolver {
    search_paths: Vec<PathBuf>,
}

impl Resolver {
    pub fn new(search_paths: Vec<PathBuf>) -> Self {
        Self { search_paths }
    }

    /// 文字列ソースから main を始点に解決する。
    pub fn resolve_from_source(&self, src: &str) -> Result<ResolvedUnit, Vec<Diagnostic>> {
        let main_module = parse(src)?;
        self.resolve(main_module, None, "")
    }

    /// AST が既に手元にある場合に解決する (LSP の in-memory フラグメント用)。
    pub fn resolve(
        &self,
        main_module: Module,
        main_path: Option<PathBuf>,
        main_qname: &str,
    ) -> Result<ResolvedUnit, Vec<Diagnostic>> {
        let mut state = State {
            search_paths: &self.search_paths,
            loaded: HashMap::new(),
            order: Vec::new(),
            stack: Vec::new(),
            diag: Vec::new(),
        };
        // import を先に再帰解決
        for imp in main_module.imports.clone() {
            let qname = join_name(&imp.module);
            state.load_qualified(&qname, imp.span);
        }
        // main を最後に push
        let main_qname = main_qname.to_string();
        if !state.loaded.contains_key(&main_qname) {
            state.order.push(main_qname.clone());
            state.loaded.insert(
                main_qname.clone(),
                LoadedModule {
                    qualified_name: main_qname.clone(),
                    module: main_module,
                    source_path: main_path,
                },
            );
        } else {
            // 既に同名 module が読まれている = 通常はあり得ない。エラー。
            state.diag.push(Diagnostic::error(
                Span::empty(),
                format!("モジュール名衝突: `{main_qname}` が複数定義されています"),
            ));
        }

        if !state.diag.is_empty() {
            return Err(state.diag);
        }
        let modules: Vec<LoadedModule> = state
            .order
            .iter()
            .filter_map(|n| state.loaded.remove(n))
            .collect();
        let main_index = modules
            .iter()
            .position(|m| m.qualified_name == main_qname)
            .ok_or_else(|| {
                vec![Diagnostic::error(
                    Span::empty(),
                    "main module が解決結果に含まれない (resolver bug)".to_string(),
                )]
            })?;
        Ok(ResolvedUnit {
            modules,
            main_index,
        })
    }
}

struct State<'a> {
    search_paths: &'a [PathBuf],
    /// 既に読み込み済みのモジュール (qualified name → module)。
    loaded: HashMap<String, LoadedModule>,
    /// 読み込み順 (依存順)。
    order: Vec<String>,
    /// 現在 DFS で辿っている import スタック (循環検出用)。
    stack: Vec<String>,
    diag: Vec<Diagnostic>,
}

impl<'a> State<'a> {
    fn load_qualified(&mut self, qname: &str, import_span: Span) {
        if self.loaded.contains_key(qname) {
            return;
        }
        if self.stack.iter().any(|s| s == qname) {
            self.diag.push(Diagnostic::error(
                import_span,
                format!(
                    "モジュール循環: {} → {qname}",
                    self.stack.join(" → ")
                ),
            ));
            return;
        }
        let (src, path) = match self.read_module_source(qname, import_span) {
            Some(x) => x,
            None => return, // diag は read_module_source 側で push 済み
        };
        let module = match parse(&src) {
            Ok(m) => m,
            Err(diags) => {
                self.diag.extend(diags);
                return;
            }
        };

        // モジュールヘッダの名前と import パスを照合
        if let Some(header) = &module.header {
            let declared = join_name(&header.name);
            if declared != qname {
                self.diag.push(Diagnostic::error(
                    header.span,
                    format!(
                        "モジュール名不一致: import 側は `{qname}` ですが、ファイルは `{declared}` を宣言しています"
                    ),
                ));
                // 続行はする (ヘッダ照合ミスは致命じゃない場面もある)
            }
        }

        // 子の import を先に解決
        self.stack.push(qname.to_string());
        for imp in module.imports.clone() {
            let child = join_name(&imp.module);
            self.load_qualified(&child, imp.span);
        }
        self.stack.pop();

        self.order.push(qname.to_string());
        self.loaded.insert(
            qname.to_string(),
            LoadedModule {
                qualified_name: qname.to_string(),
                module,
                source_path: Some(path),
            },
        );
    }

    fn read_module_source(&mut self, qname: &str, span: Span) -> Option<(String, PathBuf)> {
        // 各モジュールはディレクトリ単位: `<search>/Foo/Bar/db.cadhr`。これは
        // セッション (cadhr-proj/<project>/db.cadhr) とも揃った形にするため。
        let rel = qname.replace('.', "/") + "/db.cadhr";
        for base in self.search_paths {
            let candidate = base.join(&rel);
            if candidate.exists() {
                match std::fs::read_to_string(&candidate) {
                    Ok(s) => return Some((s, candidate)),
                    Err(e) => {
                        self.diag.push(Diagnostic::error(
                            span,
                            format!("モジュール `{qname}` の読み込みに失敗: {e}"),
                        ));
                        return None;
                    }
                }
            }
        }
        self.diag.push(Diagnostic::error(
            span,
            format!(
                "モジュール `{qname}` が見つかりません (search_paths: {})",
                self.search_paths
                    .iter()
                    .map(|p| p.display().to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        ));
        None
    }
}

pub fn join_name(n: &ModuleName) -> String {
    n.segments.join(".")
}

/// `Foo.Bar` 形式の文字列から `ModuleName` をでっちあげる (テスト / 内部用)。
pub fn parse_qualified(s: &str) -> ModuleName {
    ModuleName {
        segments: s.split('.').map(|s| s.to_string()).collect(),
        span: Span::empty(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::Path;
    use tempfile::tempdir;

    fn write_module(dir: &Path, qname: &str, content: &str) -> PathBuf {
        // Resolver は `<base>/<qname>/db.cadhr` を探すので、テストもその構造で書く。
        let rel = qname.replace('.', "/") + "/db.cadhr";
        let path = dir.join(&rel);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).unwrap();
        }
        fs::write(&path, content).unwrap();
        path
    }

    #[test]
    fn loads_single_import() {
        let tmp = tempdir().unwrap();
        write_module(
            tmp.path(),
            "Std",
            "module Std exposing (..)\n\nx = 42\n",
        );
        let resolver = Resolver::new(vec![tmp.path().to_path_buf()]);
        let unit = resolver
            .resolve_from_source("import Std\n\nmain = 1")
            .unwrap();
        assert_eq!(unit.modules.len(), 2);
        assert_eq!(unit.modules[0].qualified_name, "Std");
        assert_eq!(unit.modules[unit.main_index].qualified_name, "");
    }

    #[test]
    fn loads_nested_path() {
        let tmp = tempdir().unwrap();
        write_module(
            tmp.path(),
            "Std.Gears",
            "module Std.Gears exposing (..)\n\ninvolute = 1\n",
        );
        let resolver = Resolver::new(vec![tmp.path().to_path_buf()]);
        let unit = resolver
            .resolve_from_source("import Std.Gears\n\nmain = 1")
            .unwrap();
        assert!(unit
            .modules
            .iter()
            .any(|m| m.qualified_name == "Std.Gears"));
    }

    #[test]
    fn detects_cycle() {
        let tmp = tempdir().unwrap();
        write_module(
            tmp.path(),
            "A",
            "module A exposing (..)\n\nimport B\n\nx = 1\n",
        );
        write_module(
            tmp.path(),
            "B",
            "module B exposing (..)\n\nimport A\n\ny = 2\n",
        );
        let resolver = Resolver::new(vec![tmp.path().to_path_buf()]);
        let err = resolver
            .resolve_from_source("import A\n\nmain = 1")
            .unwrap_err();
        assert!(
            err.iter().any(|d| d.message.contains("循環")),
            "{:?}",
            err
        );
    }

    #[test]
    fn header_name_mismatch_reported() {
        let tmp = tempdir().unwrap();
        write_module(
            tmp.path(),
            "Foo",
            "module Bar exposing (..)\n\nx = 1\n",
        );
        let resolver = Resolver::new(vec![tmp.path().to_path_buf()]);
        let err = resolver
            .resolve_from_source("import Foo\n\nmain = 1")
            .unwrap_err();
        assert!(
            err.iter().any(|d| d.message.contains("モジュール名不一致")),
            "{:?}",
            err
        );
    }
}
