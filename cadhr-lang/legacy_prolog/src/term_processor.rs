use crate::parse::Term;

pub struct BuiltinFunctorSet {
    pub functors: &'static [(&'static str, &'static [usize])],
    /// true の場合、term_rewrite が引数を再帰的に解決する。
    /// false の場合、引数はそのまま保持される（データ用 functor 向け）。
    pub resolve_args: bool,
}
inventory::collect!(BuiltinFunctorSet);

pub trait TermProcessor<S = ()> {
    type Output;
    type Error;
    fn process(&self, terms: &[Term<S>]) -> Result<Self::Output, Self::Error>;
}

pub fn is_builtin_functor(name: &str) -> bool {
    inventory::iter::<BuiltinFunctorSet>()
        .flat_map(|set| set.functors.iter())
        .any(|(n, _)| *n == name)
}

pub fn is_builtin_functor_with_arity(name: &str, arity: usize) -> bool {
    inventory::iter::<BuiltinFunctorSet>()
        .flat_map(|set| set.functors.iter())
        .any(|(n, arities)| *n == name && arities.contains(&arity))
}

pub fn should_resolve_args(name: &str) -> bool {
    inventory::iter::<BuiltinFunctorSet>()
        .filter(|set| set.functors.iter().any(|(n, _)| *n == name))
        .all(|set| set.resolve_args)
}

pub fn all_builtin_functors() -> Vec<(&'static str, &'static [usize])> {
    inventory::iter::<BuiltinFunctorSet>()
        .flat_map(|set| set.functors.iter().copied())
        .collect()
}
