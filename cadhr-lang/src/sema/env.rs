//! 型環境: 識別子 → 推論スキーム のマップ。
//!
//! 推論中は可変型変数 (`InferTy`) を共有するため `InferScheme` を保持する。
//! 単一化はセルを破壊的に書き換えるので、env を clone しても自由変数の同一性は
//! 保たれる (Rc 共有)。

use crate::sema::infer_ty::InferScheme;
use std::collections::HashMap;

/// 型環境。`infer` 中に変数束縛をスタックするのに使う (immutable update スタイル)。
#[derive(Clone, Debug, Default)]
pub struct TypeEnv {
    bindings: HashMap<String, InferScheme>,
}

impl TypeEnv {
    pub fn new() -> Self {
        Self::default()
    }

    /// 名前 → スキームを 1 個追加した新しい環境を返す (元は変更しない)。
    pub fn extend(&self, name: &str, scheme: InferScheme) -> Self {
        let mut new = self.clone();
        new.bindings.insert(name.to_string(), scheme);
        new
    }

    pub fn lookup(&self, name: &str) -> Option<&InferScheme> {
        self.bindings.get(name)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&String, &InferScheme)> {
        self.bindings.iter()
    }
}
