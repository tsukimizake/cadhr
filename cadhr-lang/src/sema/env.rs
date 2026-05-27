//! 型環境: 識別子 → スキーム のマップ。

use crate::sema::ty::{Scheme, TyVar};
use std::collections::{HashMap, HashSet};

/// 型環境。`infer` 中に変数束縛をスタックするのに使う (immutable update スタイル)。
#[derive(Clone, Debug, Default)]
pub struct TypeEnv {
    bindings: HashMap<String, Scheme>,
}

impl TypeEnv {
    pub fn new() -> Self {
        Self::default()
    }

    /// 名前 → スキームを 1 個追加した新しい環境を返す (元は変更しない)。
    pub fn extend(&self, name: &str, scheme: Scheme) -> Self {
        let mut new = self.clone();
        new.bindings.insert(name.to_string(), scheme);
        new
    }

    /// 複数束縛を一度に追加。
    pub fn extend_many<I, S>(&self, iter: I) -> Self
    where
        I: IntoIterator<Item = (S, Scheme)>,
        S: Into<String>,
    {
        let mut new = self.clone();
        for (n, sc) in iter {
            new.bindings.insert(n.into(), sc);
        }
        new
    }

    pub fn lookup(&self, name: &str) -> Option<&Scheme> {
        self.bindings.get(name)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&String, &Scheme)> {
        self.bindings.iter()
    }

    /// 環境内のすべての自由変数。generalize で「環境に出現しない自由変数のみ全称化」
    /// するために使う。
    pub fn free_vars(&self) -> HashSet<TyVar> {
        let mut out = HashSet::new();
        for scheme in self.bindings.values() {
            out.extend(scheme.free_vars());
        }
        out
    }
}
