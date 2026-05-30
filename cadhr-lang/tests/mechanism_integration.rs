//! mechanism.cadhr の統合テスト。旧 Prolog 版の逆方向制約解法と違い、
//! `next_theta` 関数で forward 計算するだけ。3 段歯車 train が組み上がるかを確認する。

use cadhr_lang::{Inputs, Value, compile, run_main};

const SRC: &str = include_str!("mechanism.cadhr");

#[test]
fn gear_train_runs_with_default_slider() {
    let prog = compile(SRC).expect("compile");
    let out = run_main(&prog, &Inputs::default()).expect("run_main");
    assert_eq!(out.models.len(), 3, "3 個の Shape3D が出るはず");
}

#[test]
fn gear_train_runs_with_explicit_theta() {
    let prog = compile(SRC).expect("compile");
    let mut inputs = Inputs::default();
    inputs.values.insert("t1".into(), Value::Float(45.0));
    let out = run_main(&prog, &inputs).expect("run_main");
    assert_eq!(out.models.len(), 3);
}

#[test]
fn slider_is_extracted() {
    let prog = compile(SRC).expect("compile");
    assert_eq!(prog.sliders.len(), 1);
    assert_eq!(prog.sliders[0].name, "t1");
    assert_eq!(prog.sliders[0].lo, 0.0);
    assert_eq!(prog.sliders[0].hi, 360.0);
}
