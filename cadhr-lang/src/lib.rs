//! cadhr-lang: Elm-like CAD DSL.
//!
//! GUI から使う高レベル API は本 module 直下に置く:
//!   - [`compile`] / [`CompiledProgram`]
//!   - [`run_binding`] / [`MainOutput`] / [`Inputs`]
//!   - [`BindingSignature`] / [`BindingParam`] (任意 top-level binding に対する GUI 用引数情報)
//!   - [`SliderDecl`] (引数に紐付く GUI スライダー仕様)
//!   - [`Diagnostic`] / [`Span`]
//!
//! `manifold` feature を有効にした場合は `runtime::manifold_bridge` 経由で
//! `Model3D` を実際の 3D メッシュにできる。
//!
//! `main` を特別扱いしない: `run_binding(prog, "main", inputs)` が旧 `run_main` の役割
//! を果たし、それ以外の top-level binding (Shape3D / List Shape3D / `{ models, .. }` を
//! 返すもの) も同じ API で評価できる。

pub mod diagnostic;
pub mod module;
pub mod runtime;
pub mod sema;
pub mod sketch;
pub mod syntax;

pub use diagnostic::{Diagnostic, RelatedInfo, Severity, Span};
pub use module::{LoadedModule, ResolvedUnit, Resolver};
pub use runtime::value::{Model3D, Value};
pub use sema::slider::{ElemTy, SliderDecl};
pub use sema::ty::Scheme;

use sema::ty::Type;
use std::collections::HashMap;
use std::path::PathBuf;

/// `compile()` が返す処理単位。`unit` は依存解決済みの全モジュール、`sliders` は
/// GUI 用、`diagnostics` は型推論や網羅性検査などの warning。
///
/// `Clone` を実装しているので、GUI 側はコンパイル結果を Model に保持して
/// slider 操作のたびに `run_binding` だけ呼び直すパターンが取れる。
///
/// `schemes` は型推論結果 (qname → 局所名 → Scheme)。GUI が任意の binding
/// 名に対する引数情報を引くために使う。
pub struct CompiledProgram {
    pub unit: ResolvedUnit,
    pub sliders: Vec<SliderDecl>,
    pub diagnostics: Vec<Diagnostic>,
    pub schemes: HashMap<String, HashMap<String, Scheme>>,
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
            schemes: self.schemes.clone(),
        }
    }
}

impl std::fmt::Debug for CompiledProgram {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CompiledProgram")
            .field("modules", &self.unit.modules.len())
            .field("sliders", &self.sliders.len())
            .field("diagnostics", &self.diagnostics.len())
            .finish()
    }
}

/// 任意 top-level binding の引数情報。GUI は param 順に slider / 数値入力を表示する。
#[derive(Clone, Debug)]
pub struct BindingSignature {
    pub name: String,
    pub params: Vec<BindingParam>,
}

/// binding の 1 引数。`slider <binding>.<param>` decl があれば `range` がセットされる。
#[derive(Clone, Debug)]
pub struct BindingParam {
    pub name: String,
    pub range: Option<SliderDecl>,
}

/// `run_binding()` への入力。binding 引数の名前 → 値 を渡す。
/// `control_overrides` は GUI ドラッグで上書きされた control point の現在値。
/// `search_paths` は `center3d` などが内部で manifold 評価する際の STL 検索パス。
#[derive(Clone, Debug, Default)]
pub struct Inputs {
    pub values: HashMap<String, Value>,
    pub control_overrides: HashMap<String, [f64; 3]>,
    pub search_paths: Vec<std::path::PathBuf>,
}

/// `run_binding()` の出力。binding の戻り値が record なら各 field を取り出す。
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

    // sketch DSL の制約検査。構文的な検査なので型推論より先に fail fast する
    // (違反した sketch ブロックは GUI と紐付けられないため error 扱い)。
    let mut sketch_diag: Vec<Diagnostic> = Vec::new();
    for lm in &unit.modules {
        sketch_diag.extend(sema::sketch::check_module(&lm.module));
    }
    if !sketch_diag.is_empty() {
        return Err(sketch_diag);
    }

    let mut diagnostics: Vec<Diagnostic> = Vec::new();
    let registry = sema::builtin::registry();
    let (schemes, infer_diag) = sema::infer::infer_unit(&unit, &registry);
    let has_fatal = !infer_diag.is_empty();
    for d in infer_diag {
        diagnostics.push(d);
    }
    if has_fatal {
        return Err(diagnostics);
    }

    let main_module = &unit.modules[unit.main_index].module;
    let (sliders, slider_diag) = sema::slider::extract_sliders(main_module);
    diagnostics.extend(slider_diag);

    for lm in &unit.modules {
        diagnostics.extend(sema::exhaustive::check_module(&lm.module));
    }

    Ok(CompiledProgram {
        unit,
        sliders,
        diagnostics,
        schemes,
    })
}

impl CompiledProgram {
    /// GUI の combo_box 候補生成用。main module の top-level `Decl::Value` のうち、
    /// 戻り型 (引数を剥がした後) がレンダリング可能なものを宣言順で返す。
    /// レンダリング可能 = `Shape3D` / `List Shape3D` / `{ models : List Shape3D, .. }`
    /// を含む record / それらに unify される型変数。
    pub fn previewable_bindings(&self) -> Vec<BindingSignature> {
        self.bindings_with_return_filter(is_renderable_return)
    }

    /// GUI の 2D sketch 用。戻り型 (引数を剥がした後) が `Shape2D`、または
    /// `Shape2D` の field を 1 つ以上持つ record の top-level binding を宣言順で返す。
    pub fn shape2d_bindings(&self) -> Vec<BindingSignature> {
        self.bindings_with_return_filter(is_shape2d_return)
    }

    /// GUI の SketchV2 workspace 用。body が `sketch .. end` そのものである
    /// 引数なし top-level binding の名前を宣言順で返す (双方向編集可能な binding)。
    pub fn sketch_block_bindings(&self) -> Vec<String> {
        let main_lm = &self.unit.modules[self.unit.main_index];
        sketch::sketch_binding_names(&main_lm.module)
    }

    fn bindings_with_return_filter(&self, pred: impl Fn(&Type) -> bool) -> Vec<BindingSignature> {
        let main_lm = &self.unit.modules[self.unit.main_index];
        let main_qname = main_lm.qualified_name.clone();
        let module_schemes = match self.schemes.get(&main_qname) {
            Some(s) => s,
            None => return Vec::new(),
        };
        let mut out = Vec::new();
        for d in &main_lm.module.decls {
            let v = match d {
                syntax::ast::Decl::Value(v) => v,
                _ => continue,
            };
            let scheme = match module_schemes.get(&v.name) {
                Some(s) => s,
                None => continue,
            };
            // 引数を剥がした戻り型を取る (Pattern::Var 以外を含む場合はスキップ)。
            let mut ret = &scheme.ty;
            let mut ok = true;
            for p in &v.params {
                if !matches!(p, syntax::ast::Pattern::Var(_, _)) {
                    ok = false;
                    break;
                }
                if let Type::Arrow(_, to) = ret {
                    ret = to.as_ref();
                } else {
                    // 引数より型が「足りない」: 推論できなかった等
                    ok = false;
                    break;
                }
            }
            if !ok || !pred(ret) {
                continue;
            }
            let params = collect_params(v, &self.sliders);
            out.push(BindingSignature {
                name: v.name.clone(),
                params,
            });
        }
        out
    }

    /// 名前を指定して binding の引数 signature を引く。`previewable_bindings`
    /// フィルタ内外を問わず、`Pattern::Var` のみで構成されていれば signature を返す。
    /// 引数の型は GUI からは Float/Int として扱う (slider decl があれば range を付与)。
    pub fn binding_signature(&self, name: &str) -> Option<BindingSignature> {
        let main_lm = &self.unit.modules[self.unit.main_index];
        for d in &main_lm.module.decls {
            let v = match d {
                syntax::ast::Decl::Value(v) => v,
                _ => continue,
            };
            if v.name != name {
                continue;
            }
            if !v.params.iter().all(|p| matches!(p, syntax::ast::Pattern::Var(_, _))) {
                return None;
            }
            return Some(BindingSignature {
                name: v.name.clone(),
                params: collect_params(v, &self.sliders),
            });
        }
        None
    }
}

fn collect_params(v: &syntax::ast::ValueDecl, sliders: &[SliderDecl]) -> Vec<BindingParam> {
    v.params
        .iter()
        .filter_map(|p| match p {
            syntax::ast::Pattern::Var(n, _) => Some(n.clone()),
            _ => None,
        })
        .map(|name| {
            let range = sliders
                .iter()
                .find(|s| s.binding == v.name && s.param == name)
                .cloned();
            BindingParam { name, range }
        })
        .collect()
}

fn is_shape2d_return(ty: &Type) -> bool {
    let is_shape2d = |t: &Type| matches!(t, Type::Con(n, a) if n == "Shape2D" && a.is_empty());
    match ty {
        Type::Record(fields) => fields.iter().any(|(_, t)| is_shape2d(t)),
        _ => is_shape2d(ty),
    }
}

fn is_renderable_return(ty: &Type) -> bool {
    match ty {
        Type::Con(name, args) if name == "Shape3D" && args.is_empty() => true,
        Type::Con(name, args) if name == "List" && args.len() == 1 => {
            matches!(&args[0], Type::Con(n, a) if n == "Shape3D" && a.is_empty())
        }
        Type::Record(fields) => fields.iter().any(|(n, _)| n == "models"),
        _ => false,
    }
}

/// 任意の top-level binding を実行する。`binding` に `"main"` を渡せば旧 `run_main`
/// 相当。`inputs.values` から binding の各引数値を取り、不足分は slider 中央値 /
/// `Value::Float(0.0)` で補完する。
pub fn run_binding(
    prog: &CompiledProgram,
    binding: &str,
    inputs: &Inputs,
) -> Result<MainOutput, Diagnostic> {
    let cur = eval_binding_value(prog, binding, inputs)?;
    let mut out = unpack_main_output(cur)?;
    // eval 中に builtin が記録した control point を Output に詰める。
    out.control_points = runtime::builtin::take_recorded_controls();
    Ok(out)
}

/// 戻り型が `Shape2D` (または Shape2D field を持つ record) の binding を評価し、
/// 輪郭 polygon 群 (穴を含む) を返す。record は全 Shape2D field の輪郭を連結する。
/// GUI の 2D sketch が参照表示に使う。
#[cfg(feature = "manifold")]
pub fn run_binding_2d(
    prog: &CompiledProgram,
    binding: &str,
    inputs: &Inputs,
) -> Result<Vec<Vec<[f64; 2]>>, Diagnostic> {
    let cur = eval_binding_value(prog, binding, inputs)?;
    let _ = runtime::builtin::take_recorded_controls();
    match cur {
        Value::Shape2D(m) => Ok(runtime::manifold_bridge::shape2d_contours(&m)),
        Value::Record(fields) => {
            let mut contours = Vec::new();
            for (_, v) in fields {
                if let Value::Shape2D(m) = v {
                    contours.extend(runtime::manifold_bridge::shape2d_contours(&m));
                }
            }
            Ok(contours)
        }
        other => Err(Diagnostic::runtime(
            Span::empty(),
            format!("binding の戻り値が Shape2D でない: {other}"),
        )),
    }
}

/// binding を lookup し、引数を `inputs` (不足分はデフォルト値) で適用した値を返す。
fn eval_binding_value(
    prog: &CompiledProgram,
    binding: &str,
    inputs: &Inputs,
) -> Result<Value, Diagnostic> {
    // GUI からの control point override を thread-local にセット (eval 中 builtin が見る)。
    runtime::builtin::set_control_overrides(inputs.control_overrides.clone());
    // center3d などが Manifold 評価する際に STL 検索パスを引けるようセット。
    runtime::builtin::set_include_paths(inputs.search_paths.clone());

    let reg = runtime::builtin::registry();
    let mut ev = runtime::eval::Evaluator::new(&reg);
    let mut envs = ev.eval_unit(&prog.unit).map_err(|e| {
        e.into_iter()
            .next()
            .unwrap_or_else(|| Diagnostic::runtime(Span::empty(), "evaluator が空のエラーを返した"))
    })?;
    let main_qname = prog.unit.modules[prog.unit.main_index]
        .qualified_name
        .clone();
    let env = envs.remove(&main_qname).ok_or_else(|| {
        Diagnostic::runtime(Span::empty(), "main モジュールの env が見つかりません")
    })?;

    let mut cur = env.lookup(binding).ok_or_else(|| {
        Diagnostic::runtime(
            Span::empty(),
            format!("binding `{binding}` が定義されていません"),
        )
    })?;

    let sig = prog.binding_signature(binding);
    if let Some(sig) = sig {
        for p in &sig.params {
            let v = inputs
                .values
                .get(&p.name)
                .cloned()
                .unwrap_or_else(|| default_for_param(p));
            cur = ev.apply(cur, v, Span::empty())?;
        }
    }
    Ok(cur)
}

fn default_for_param(p: &BindingParam) -> Value {
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
        other => Err(Diagnostic::runtime(
            Span::empty(),
            format!("binding の戻り値が record / Shape3D / List Shape3D でない: {other}"),
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
        inputs
            .values
            .insert("length".to_string(), Value::Float(30.0));
        let out = run_binding(&prog, "main", &inputs).expect("run_binding");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn slider_default_used() {
        let src = "main n = cube 1.0 1.0 1.0\nslider main.n = 6.0 .. 80.0";
        let prog = compile(src).expect("compile");
        let sig = prog.binding_signature("main").expect("main signature");
        assert_eq!(sig.params.len(), 1);
        assert!(sig.params[0].range.is_some());
        let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn slider_range_scoped_to_binding() {
        // 同名引数 `length` を持つ 2 binding。slider は binding ごとに別範囲を付与する。
        let src = "\
main length = cube 10.0 10.0 length
honi length = sphere length
slider main.length = 6.0 .. 80.0
slider honi.length = 1.0 .. 10.0
";
        let prog = compile(src).expect("compile");
        let main_range = prog.binding_signature("main").unwrap().params[0]
            .range
            .clone()
            .expect("main.length range");
        let honi_range = prog.binding_signature("honi").unwrap().params[0]
            .range
            .clone()
            .expect("honi.length range");
        assert_eq!((main_range.lo, main_range.hi), (6.0, 80.0));
        assert_eq!((honi_range.lo, honi_range.hi), (1.0, 10.0));
    }

    #[test]
    fn main_output_record() {
        let src = "main = { models = [cube 1.0 1.0 1.0], bom = [], controls = [] }";
        let prog = compile(src).expect("compile");
        let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn non_exhaustive_case_reports_warning() {
        let src = "f x = case x of\n    True -> 1";
        let prog = compile(src).expect("compile");
        assert!(
            prog.diagnostics
                .iter()
                .any(|d| d.message().contains("False") && d.severity() == Severity::Warning),
            "expected non-exhaustive case warning, got {:?}",
            prog.diagnostics
        );
    }

    fn write_lib(tmp: &tempfile::TempDir, src: &str) {
        // Resolver は `<base>/Lib/db.cadhr` を探す。
        std::fs::create_dir_all(tmp.path().join("Lib")).unwrap();
        std::fs::write(tmp.path().join("Lib/db.cadhr"), src).unwrap();
    }

    #[test]
    fn import_resolves_and_runs() {
        let tmp = tempfile::tempdir().unwrap();
        write_lib(&tmp, "module Lib exposing (..)\n\nside = 5.0\n");
        let src = "import Lib exposing (side)\n\nmain = cube side side side";
        let prog = compile_with_paths(src, &[tmp.path().to_path_buf()]).expect("compile");
        let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn import_qualified_only() {
        let tmp = tempfile::tempdir().unwrap();
        write_lib(&tmp, "module Lib exposing (..)\n\nside = 5.0\n");
        let src = "import Lib\n\nmain = cube Lib.side Lib.side Lib.side";
        let prog = compile_with_paths(src, &[tmp.path().to_path_buf()]).expect("compile");
        let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn std_module_loads() {
        let std_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("std");
        let src = "import Std exposing (Output)\n\nmain = { models = [], bom = [], controls = [] }";
        let prog = compile_with_paths(src, &[std_path]).expect("compile");
        let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
        assert_eq!(out.models.len(), 0);
    }

    #[test]
    fn import_with_alias() {
        let tmp = tempfile::tempdir().unwrap();
        write_lib(&tmp, "module Lib exposing (..)\n\nside = 5.0\n");
        let src = "import Lib as L\n\nmain = cube L.side L.side L.side";
        let prog = compile_with_paths(src, &[tmp.path().to_path_buf()]).expect("compile");
        let out = run_binding(&prog, "main", &Inputs::default()).expect("run_binding");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn previewable_bindings_lists_shape3d_decls() {
        let src = "\
plate = cube 20.0 20.0 1.0
hex_head r = cylinder r 5.0
helper x = x + 1.0
main = plate
";
        let prog = compile(src).expect("compile");
        let bindings = prog.previewable_bindings();
        let names: Vec<&str> = bindings.iter().map(|b| b.name.as_str()).collect();
        assert!(names.contains(&"plate"), "plate should be previewable; got {names:?}");
        assert!(names.contains(&"hex_head"), "hex_head should be previewable; got {names:?}");
        assert!(names.contains(&"main"), "main should be previewable; got {names:?}");
        assert!(!names.contains(&"helper"), "helper (Float -> Float) should not be previewable; got {names:?}");
    }

    #[test]
    fn shape2d_bindings_lists_shape2d_decls_only() {
        let src = "\
profile = polygon (segments [p2 0.0 0.0, p2 4.0 0.0, p2 0.0 4.0])
disc r = circle r
sk1 = { poly1 = profile, circ1 = circle 2.0 }
plate = cube 20.0 20.0 1.0
main = plate
";
        let prog = compile(src).expect("compile");
        let names: Vec<String> = prog.shape2d_bindings().into_iter().map(|b| b.name).collect();
        assert_eq!(
            names,
            vec![
                "profile".to_string(),
                "disc".to_string(),
                "sk1".to_string()
            ]
        );
    }

    #[cfg(feature = "manifold")]
    #[test]
    fn run_binding_2d_returns_contours() {
        let src = "\
profile = polygon (segments [p2 0.0 0.0, p2 4.0 0.0, p2 0.0 4.0])
main = extrude_xy 1.0 profile
";
        let prog = compile(src).expect("compile");
        let contours =
            run_binding_2d(&prog, "profile", &Inputs::default()).expect("run_binding_2d");
        assert_eq!(contours.len(), 1);
        assert_eq!(contours[0].len(), 3);
    }

    #[cfg(feature = "manifold")]
    #[test]
    fn run_binding_2d_concatenates_record_fields() {
        let src = "\
sk1 = { poly1 = polygon (segments [p2 0.0 0.0, p2 4.0 0.0, p2 0.0 4.0]), pt1 = p2 1.0 1.0, poly2 = polygon (segments [p2 10.0 0.0, p2 14.0 0.0, p2 10.0 4.0]) }
main = extrude_xy 1.0 sk1.poly1
";
        let prog = compile(src).expect("compile");
        let contours =
            run_binding_2d(&prog, "sk1", &Inputs::default()).expect("run_binding_2d");
        assert_eq!(contours.len(), 2);
    }

    #[cfg(feature = "manifold")]
    #[test]
    fn sketch_block_compiles_and_evaluates() {
        let src = "sk =\n    sketch\n        var x1 = 0.0\n        let y1 = 3.0\n        poly1 = polygon (segments [p2 x1 y1, p2 4.0 y1, p2 x1 7.0])\n        circ1 = circle 2.0 |> translate2d (p2 0.0 0.0) (p2 4.0 5.0)\n    in\n    { poly1 = poly1, circ1 = circ1 }\n    end\n\nmain = extrude_xy 1.0 sk.poly1\n";
        let prog = compile(src).expect("compile");
        let contours = run_binding_2d(&prog, "sk", &Inputs::default()).expect("run_binding_2d");
        assert_eq!(contours.len(), 2);
    }

    #[test]
    fn sketch_validation_errors_are_fatal() {
        // var の右辺が Int リテラル → sketch DSL 違反で compile が Err になる。
        let src = "sk =\n    sketch\n        var x = 1\n        p = p2 x x\n    in\n    p\n    end\n";
        let err = compile(src).expect_err("expected sketch validation error");
        assert!(
            err.iter().any(|d| d.message().contains("sketch")),
            "diags: {err:?}"
        );
    }

    #[test]
    fn run_binding_to_named_decl() {
        let src = "\
plate = cube 20.0 20.0 1.0
main = plate
";
        let prog = compile(src).expect("compile");
        let out = run_binding(&prog, "plate", &Inputs::default()).expect("run_binding");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn run_binding_with_args() {
        let src = "\
hex_head r = cylinder r 5.0
main = hex_head 4.0
";
        let prog = compile(src).expect("compile");
        let mut inputs = Inputs::default();
        inputs.values.insert("r".to_string(), Value::Float(3.0));
        let out = run_binding(&prog, "hex_head", &inputs).expect("run_binding");
        assert_eq!(out.models.len(), 1);
    }

    #[test]
    fn run_binding_not_found_errors() {
        let src = "main = cube 1.0 1.0 1.0";
        let prog = compile(src).expect("compile");
        let result = run_binding(&prog, "no_such_binding", &Inputs::default());
        assert!(result.is_err(), "expected Err for missing binding");
    }

    #[test]
    fn previewable_excludes_non_renderable_returns() {
        let src = "\
constant_int = 42
main = cube 1.0 1.0 1.0
";
        let prog = compile(src).expect("compile");
        let bindings = prog.previewable_bindings();
        let names: Vec<&str> = bindings.iter().map(|b| b.name.as_str()).collect();
        assert!(
            !names.contains(&"constant_int"),
            "Int constant should not be previewable; got {names:?}"
        );
    }
}
