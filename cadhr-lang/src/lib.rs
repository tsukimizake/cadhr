//! cadhr-lang: Elm-like CAD DSL.
//!
//! GUI から使う高レベル API は本 module 直下に置く:
//!   - [`compile`] / [`CompiledProgram`]
//!   - [`run_main`] / [`MainOutput`] / [`Inputs`]
//!   - [`SliderDecl`] (`main` 引数に紐付く GUI スライダー仕様)
//!   - [`Diagnostic`] / [`Span`]
//!
//! `manifold` feature を有効にした場合は `runtime::manifold_bridge` 経由で
//! `Model3D` を実際の 3D メッシュにできる。

pub mod diagnostic;
pub mod module;
pub mod runtime;
pub mod sema;
pub mod syntax;

pub use diagnostic::{Diagnostic, RelatedInfo, Severity, Span};
pub use module::{LoadedModule, ResolvedUnit, Resolver};
pub use runtime::value::{Model3D, Value};
pub use sema::slider::{ElemTy, SliderDecl};

use std::collections::HashMap;
use std::path::PathBuf;

/// `compile()` が返す処理単位。`unit` は依存解決済みの全モジュール、`sliders` は
/// GUI 用、`diagnostics` は型推論や網羅性検査などの warning。
///
/// `Clone` を実装しているので、GUI 側はコンパイル結果を Model に保持して
/// slider 操作のたびに `run_main` だけ呼び直すパターンが取れる。
pub struct CompiledProgram {
    pub unit: ResolvedUnit,
    pub sliders: Vec<SliderDecl>,
    pub diagnostics: Vec<Diagnostic>,
    pub main_signature: MainSignature,
}

impl Clone for CompiledProgram {
    fn clone(&self) -> Self {
        Self {
            unit: ResolvedUnit {
                modules: self
                    .unit
                    .modules
                    .iter()
                    .map(|m| LoadedModule {
                        qualified_name: m.qualified_name.clone(),
                        module: m.module.clone(),
                        source_path: m.source_path.clone(),
                    })
                    .collect(),
                main_index: self.unit.main_index,
            },
            sliders: self.sliders.clone(),
            diagnostics: self.diagnostics.clone(),
            main_signature: self.main_signature.clone(),
        }
    }
}

impl std::fmt::Debug for CompiledProgram {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CompiledProgram")
            .field("modules", &self.unit.modules.len())
            .field("sliders", &self.sliders.len())
            .field("diagnostics", &self.diagnostics.len())
            .field("main_signature", &self.main_signature)
            .finish()
    }
}

/// `main` の引数情報。GUI は param 順に slider を表示する。
#[derive(Clone, Debug, Default)]
pub struct MainSignature {
    pub params: Vec<MainParam>,
}

/// `main` の 1 引数。`slider` decl と紐付いていれば `range` がセットされる。
#[derive(Clone, Debug)]
pub struct MainParam {
    pub name: String,
    /// `slider name = lo..hi` で指定された範囲。GUI で値を作るとき使う。
    pub range: Option<SliderDecl>,
}

/// `run_main()` への入力。`main` 引数の名前 → 値 を渡す。
/// `control_overrides` は GUI ドラッグで上書きされた control point の現在値。
#[derive(Clone, Debug, Default)]
pub struct Inputs {
    pub values: HashMap<String, Value>,
    pub control_overrides: HashMap<String, [f64; 3]>,
}

/// `run_main()` の出力。`main` の戻り値が record なら各 field を取り出す。
/// `control_points` は eval 中に `control3d` / `control2d` builtin が呼ばれた際に
/// 記録された (name, current_position) のリスト。
#[derive(Clone, Debug, Default)]
pub struct MainOutput {
    pub models: Vec<Model3D>,
    pub bom: Vec<Value>,
    pub controls: Vec<Value>,
    pub control_points: Vec<(String, [f64; 3])>,
}

/// パース + import 解決 + 型推論 + slider 抽出 + 網羅性検査 をまとめる。
/// 型推論エラーは Warning に変換し、パース・モジュール解決の致命的エラーだけ Err にする。
/// `search_paths` は import で参照する `.cadhr` を探すディレクトリ群。
pub fn compile(src: &str) -> Result<CompiledProgram, Vec<Diagnostic>> {
    compile_with_paths(src, &[])
}

pub fn compile_with_paths(
    src: &str,
    search_paths: &[PathBuf],
) -> Result<CompiledProgram, Vec<Diagnostic>> {
    let resolver = Resolver::new(search_paths.to_vec());
    let unit = resolver.resolve_from_source(src)?;

    let mut diagnostics: Vec<Diagnostic> = Vec::new();
    let registry = sema::builtin::registry();
    let (_schemes, infer_diag) = sema::infer::infer_unit(&unit, &registry);
    for d in infer_diag {
        if d.severity == Severity::Error {
            let mut dd = d;
            dd.severity = Severity::Warning;
            diagnostics.push(dd);
        } else {
            diagnostics.push(d);
        }
    }

    let main_module = &unit.modules[unit.main_index].module;
    let (sliders, slider_diag) = sema::slider::extract_sliders(main_module);
    diagnostics.extend(slider_diag);

    for lm in &unit.modules {
        diagnostics.extend(sema::exhaustive::check_module(&lm.module));
    }

    let main_signature = build_main_signature(main_module, &sliders);

    Ok(CompiledProgram {
        unit,
        sliders,
        diagnostics,
        main_signature,
    })
}

fn build_main_signature(module: &syntax::ast::Module, sliders: &[SliderDecl]) -> MainSignature {
    use syntax::ast::{Decl, Pattern};
    let mut params = Vec::new();
    for d in &module.decls {
        if let Decl::Value(v) = d {
            if v.name == "main" {
                for p in &v.params {
                    let name = match p {
                        Pattern::Var(n, _) => n.clone(),
                        _ => continue,
                    };
                    let range = sliders.iter().find(|s| s.name == name).cloned();
                    params.push(MainParam { name, range });
                }
                break;
            }
        }
    }
    MainSignature { params }
}

/// `main` 関数を実行する。`inputs` に main 引数名 → 値 のマップを渡す。
/// 各引数について、`inputs` に値があればそれを使い、無ければ slider の中央値、
/// それも無ければ `Value::Float(0.0)` を渡す。
pub fn run_main(prog: &CompiledProgram, inputs: &Inputs) -> Result<MainOutput, Diagnostic> {
    // GUI からの control point override を thread-local にセット (eval 中 builtin が見る)。
    runtime::builtin::set_control_overrides(inputs.control_overrides.clone());

    let reg = runtime::builtin::registry();
    let mut ev = runtime::eval::Evaluator::new(&reg);
    let mut envs = ev
        .eval_unit(&prog.unit)
        .map_err(|e| e.into_iter().next().unwrap_or_else(|| Diagnostic::error(Span::empty(), "evaluator が空のエラーを返した")))?;
    let main_qname = prog.unit.modules[prog.unit.main_index].qualified_name.clone();
    let env = envs
        .remove(&main_qname)
        .ok_or_else(|| Diagnostic::error(Span::empty(), "main モジュールの env が見つかりません"))?;

    let mut cur = env
        .lookup("main")
        .cloned()
        .ok_or_else(|| Diagnostic::error(Span::empty(), "main 関数が定義されていません"))?;

    for p in &prog.main_signature.params {
        let v = inputs
            .values
            .get(&p.name)
            .cloned()
            .unwrap_or_else(|| default_for_param(p));
        cur = ev.apply(cur, v, Span::empty())?;
    }

    let mut out = unpack_main_output(cur)?;
    // eval 中に builtin が記録した control point を Output に詰める。
    out.control_points = runtime::builtin::take_recorded_controls();
    Ok(out)
}

fn default_for_param(p: &MainParam) -> Value {
    if let Some(r) = &p.range {
        let mid = (r.lo + r.hi) / 2.0;
        match r.elem_ty {
            ElemTy::Int => Value::Int(mid as i64),
            ElemTy::Float => Value::Float(mid),
        }
    } else {
        Value::Float(0.0)
    }
}

fn unpack_main_output(v: Value) -> Result<MainOutput, Diagnostic> {
    match v {
        Value::Record(fields) => {
            let mut out = MainOutput::default();
            for (n, val) in fields {
                match n.as_str() {
                    "models" => {
                        if let Value::List(vs) = val {
                            for x in vs {
                                if let Value::Shape3D(m) = x {
                                    out.models.push(m);
                                }
                            }
                        }
                    }
                    "bom" => {
                        if let Value::List(vs) = val {
                            out.bom = vs;
                        }
                    }
                    "controls" => {
                        if let Value::List(vs) = val {
                            out.controls = vs;
                        }
                    }
                    _ => {}
                }
            }
            Ok(out)
        }
        Value::Shape3D(m) => Ok(MainOutput {
            models: vec![m],
            ..Default::default()
        }),
        Value::List(vs) => {
            let mut out = MainOutput::default();
            for x in vs {
                if let Value::Shape3D(m) = x {
                    out.models.push(m);
                }
            }
            Ok(out)
        }
        other => Err(Diagnostic::error(
            Span::empty(),
            format!(
                "main の戻り値が record / Shape3D / List Shape3D でない: {other}"
            ),
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compile_and_run_bolt_like() {
        let src = "main length = cube 10.0 10.0 length";
        let prog = compile(src).expect("compile");
        let mut inputs = Inputs::default();
        inputs.values.insert("length".to_string(), Value::Float(30.0));
        let out = run_main(&prog, &inputs).expect("run_main");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn slider_default_used() {
        let src = "main n = cube 1.0 1.0 1.0\nslider n = 6.0 .. 80.0";
        let prog = compile(src).expect("compile");
        assert_eq!(prog.main_signature.params.len(), 1);
        assert!(prog.main_signature.params[0].range.is_some());
        let out = run_main(&prog, &Inputs::default()).expect("run_main");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn main_output_record() {
        let src = "main = { models = [cube 1.0 1.0 1.0], bom = [], controls = [] }";
        let prog = compile(src).expect("compile");
        let out = run_main(&prog, &Inputs::default()).expect("run_main");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn non_exhaustive_case_reports_warning() {
        let src = "f x = case x of | True -> 1";
        let prog = compile(src).expect("compile");
        assert!(
            prog.diagnostics
                .iter()
                .any(|d| d.message.contains("False") && d.severity == Severity::Warning),
            "expected non-exhaustive case warning, got {:?}",
            prog.diagnostics
        );
    }

    #[test]
    fn import_resolves_and_runs() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Lib.cadhr"),
            "module Lib exposing (..)\n\nside = 5.0\n",
        )
        .unwrap();
        let src = "import Lib exposing (side)\n\nmain = cube side side side";
        let prog = compile_with_paths(src, &[tmp.path().to_path_buf()]).expect("compile");
        let out = run_main(&prog, &Inputs::default()).expect("run_main");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn import_qualified_only() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Lib.cadhr"),
            "module Lib exposing (..)\n\nside = 5.0\n",
        )
        .unwrap();
        let src = "import Lib\n\nmain = cube Lib.side Lib.side Lib.side";
        let prog = compile_with_paths(src, &[tmp.path().to_path_buf()]).expect("compile");
        let out = run_main(&prog, &Inputs::default()).expect("run_main");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn std_module_loads() {
        let std_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("std");
        let src = "import Std exposing (Output)\n\nmain = { models = [], bom = [], controls = [] }";
        let prog = compile_with_paths(src, &[std_path]).expect("compile");
        let out = run_main(&prog, &Inputs::default()).expect("run_main");
        assert_eq!(out.models.len(), 0);
    }

    #[test]
    fn import_with_alias() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Lib.cadhr"),
            "module Lib exposing (..)\n\nside = 5.0\n",
        )
        .unwrap();
        let src = "import Lib as L\n\nmain = cube L.side L.side L.side";
        let prog = compile_with_paths(src, &[tmp.path().to_path_buf()]).expect("compile");
        let out = run_main(&prog, &Inputs::default()).expect("run_main");
        assert_eq!(out.models.len(), 1);
    }
}
