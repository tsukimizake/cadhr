use std::collections::HashSet;
use std::fmt;
use std::path::{Path, PathBuf};

use crate::parse::{Clause, FileRegistry, Term, database};
use crate::term_processor::is_builtin_functor;

#[derive(Debug)]
pub enum ModuleError {
    FileNotFound {
        module_path: String,
        searched: Vec<PathBuf>,
    },
    CyclicDependency {
        path: PathBuf,
    },
    ParseError {
        path: PathBuf,
        message: String,
    },
    IoError {
        path: PathBuf,
        error: std::io::Error,
    },
}

impl fmt::Display for ModuleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ModuleError::FileNotFound {
                module_path,
                searched,
            } => {
                write!(f, "Module '{}' not found. Searched:", module_path)?;
                for p in searched {
                    write!(f, "\n  {}", p.display())?;
                }
                Ok(())
            }
            ModuleError::CyclicDependency { path } => {
                write!(f, "Cyclic dependency detected: {}", path.display())
            }
            ModuleError::ParseError { path, message } => {
                write!(f, "Parse error in {}: {}", path.display(), message)
            }
            ModuleError::IoError { path, error } => {
                write!(f, "IO error reading {}: {}", path.display(), error)
            }
        }
    }
}

impl std::error::Error for ModuleError {}

pub fn resolve_modules(
    clauses: Vec<Clause>,
    include_paths: &[PathBuf],
    visited: &mut HashSet<PathBuf>,
    file_registry: &mut FileRegistry,
) -> Result<Vec<Clause>, ModuleError> {
    let mut result = Vec::new();

    for clause in clauses {
        match clause {
            Clause::Use { path, expose, .. } => {
                let resolved = resolve_use(&path, &expose, include_paths, visited, file_registry)?;
                result.extend(resolved);
            }
            other => result.push(other),
        }
    }

    Ok(result)
}

fn find_module_file(module_path: &str, include_paths: &[PathBuf]) -> Option<PathBuf> {
    let trimmed = module_path.trim_end_matches('/');
    for dir in include_paths {
        let candidate = dir.join(trimmed).join("db.cadhr");
        if candidate.is_file() {
            return Some(candidate);
        }
    }
    None
}

fn module_name_from_path(path: &str) -> String {
    let trimmed = path.trim_end_matches('/');
    Path::new(trimmed)
        .file_stem()
        .unwrap_or_default()
        .to_string_lossy()
        .into_owned()
}

fn resolve_use(
    module_path: &str,
    expose: &[String],
    include_paths: &[PathBuf],
    visited: &mut HashSet<PathBuf>,
    file_registry: &mut FileRegistry,
) -> Result<Vec<Clause>, ModuleError> {
    let file_path =
        find_module_file(module_path, include_paths).ok_or_else(|| ModuleError::FileNotFound {
            module_path: module_path.to_string(),
            searched: include_paths
                .iter()
                .map(|p| p.join(module_path.trim_end_matches('/')).join("db.cadhr"))
                .collect(),
        })?;

    let canonical = file_path.canonicalize().map_err(|e| ModuleError::IoError {
        path: file_path.clone(),
        error: e,
    })?;

    if !visited.insert(canonical.clone()) {
        return Err(ModuleError::CyclicDependency { path: canonical });
    }

    let source = std::fs::read_to_string(&file_path).map_err(|e| ModuleError::IoError {
        path: file_path.clone(),
        error: e,
    })?;

    let fid = file_registry.register(file_path.display().to_string(), source.clone());

    let mut clauses = database(&source).map_err(|e| ModuleError::ParseError {
        path: file_path.clone(),
        message: format!("{:?}", e),
    })?;

    for clause in &mut clauses {
        set_file_id_in_clause(clause, fid);
    }

    let child_include_paths: Vec<PathBuf> = file_path
        .parent()
        .map(|p| vec![p.to_path_buf()])
        .unwrap_or_default();

    let clauses = resolve_modules(clauses, &child_include_paths, visited, file_registry)?;

    let module_name = module_name_from_path(module_path);
    let expose_set: HashSet<&str> = expose.iter().map(|s| s.as_str()).collect();

    let mut result = Vec::new();
    for clause in clauses {
        let prefixed = prefix_clause(&clause, &module_name);
        result.push(prefixed);

        if let Some(functor) = clause_head_functor(&clause) {
            if expose_set.contains(functor.as_str()) {
                result.push(clause);
            }
        }
    }

    Ok(result)
}

fn clause_head_functor(clause: &Clause) -> Option<String> {
    match clause {
        Clause::Fact(term) => term_functor(term),
        Clause::Rule { head, .. } => term_functor(head),
        Clause::Use { .. } => None,
    }
}

fn term_functor(term: &Term) -> Option<String> {
    match term {
        Term::Struct { functor, .. } => Some(functor.clone()),
        _ => None,
    }
}

fn prefix_clause(clause: &Clause, module_name: &str) -> Clause {
    match clause {
        Clause::Fact(term) => Clause::Fact(prefix_term(term, module_name)),
        Clause::Rule { head, body } => Clause::Rule {
            head: prefix_term(head, module_name),
            body: body.iter().map(|t| prefix_term(t, module_name)).collect(),
        },
        Clause::Use { .. } => clause.clone(),
    }
}

fn prefix_term(term: &Term, module_name: &str) -> Term {
    match term {
        Term::Struct {
            functor,
            args,
            span,
        } => {
            let prefixed_functor = if is_builtin_functor(functor) {
                functor.clone()
            } else {
                format!("{}::{}", module_name, functor)
            };
            Term::Struct {
                functor: prefixed_functor,
                args: args.iter().map(|a| prefix_term(a, module_name)).collect(),
                span: *span,
            }
        }
        Term::List { items, tail } => Term::List {
            items: items.iter().map(|i| prefix_term(i, module_name)).collect(),
            tail: tail.as_ref().map(|t| Box::new(prefix_term(t, module_name))),
        },
        Term::InfixExpr { op, left, right } => Term::InfixExpr {
            op: *op,
            left: Box::new(prefix_term(left, module_name)),
            right: Box::new(prefix_term(right, module_name)),
        },
        Term::Eq { left, right } => Term::Eq {
            left: Box::new(prefix_term(left, module_name)),
            right: Box::new(prefix_term(right, module_name)),
        },
        _ => term.clone(),
    }
}

fn set_file_id_in_clause(clause: &mut Clause, file_id: u16) {
    match clause {
        Clause::Fact(term) => set_file_id_in_term(term, file_id),
        Clause::Rule { head, body } => {
            set_file_id_in_term(head, file_id);
            for b in body {
                set_file_id_in_term(b, file_id);
            }
        }
        Clause::Use { span, .. } => {
            if let Some(s) = span {
                s.file_id = file_id;
            }
        }
    }
}

fn set_file_id_in_term(term: &mut Term, file_id: u16) {
    match term {
        Term::Var { span, .. } => {
            if let Some(s) = span {
                s.file_id = file_id;
            }
        }
        Term::Struct { args, span, .. } => {
            if let Some(s) = span {
                s.file_id = file_id;
            }
            for a in args {
                set_file_id_in_term(a, file_id);
            }
        }
        Term::InfixExpr { left, right, .. } => {
            set_file_id_in_term(left, file_id);
            set_file_id_in_term(right, file_id);
        }
        Term::List { items, tail } => {
            for i in items {
                set_file_id_in_term(i, file_id);
            }
            if let Some(t) = tail {
                set_file_id_in_term(t, file_id);
            }
        }
        Term::Eq { left, right } => {
            set_file_id_in_term(left, file_id);
            set_file_id_in_term(right, file_id);
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    /// ファイルを配置し、最初のディレクトリに対する #use を resolve して
    /// 結果の clause から functor 名を収集して返す。
    /// files: &[("path/to/db.cadhr", "contents")]
    /// use_path: #use に渡すパス
    /// expose: expose するfunctor名
    fn resolve_test(
        files: &[(&str, &str)],
        use_path: &str,
        expose: &[&str],
    ) -> Result<Vec<String>, ModuleError> {
        let dir = tempfile::tempdir().unwrap();
        for (path, contents) in files {
            let full = dir.path().join(path);
            fs::create_dir_all(full.parent().unwrap()).unwrap();
            fs::write(&full, contents).unwrap();
        }

        let clauses = vec![Clause::Use {
            path: use_path.to_string(),
            expose: expose.iter().map(|s| s.to_string()).collect(),
            span: None,
        }];

        let mut visited = HashSet::new();
        let result = resolve_modules(
            clauses,
            &[dir.path().to_path_buf()],
            &mut visited,
            &mut FileRegistry::new(),
        )?;

        let functors: Vec<String> = result
            .iter()
            .filter_map(|c| match c {
                Clause::Fact(Term::Struct { functor, .. }) => Some(functor.clone()),
                Clause::Rule { head: Term::Struct { functor, .. }, .. } => Some(functor.clone()),
                _ => None,
            })
            .collect();
        Ok(functors)
    }

    #[test]
    fn test_basic_module_resolution() {
        let functors = resolve_test(
            &[("bolts/db.cadhr", "m5(X) :- size(X, 5).\nsize(small, 3).\n")],
            "bolts",
            &[],
        )
        .unwrap();

        assert!(functors.contains(&"bolts::m5".to_string()));
        assert!(functors.contains(&"bolts::size".to_string()));
    }

    #[test]
    fn test_expose() {
        let functors = resolve_test(
            &[("bolts/db.cadhr", "m5(5).\nm6(6).\n")],
            "bolts",
            &["m5"],
        )
        .unwrap();

        assert!(functors.contains(&"bolts::m5".to_string()));
        assert!(functors.contains(&"m5".to_string()));
        assert!(functors.contains(&"bolts::m6".to_string()));
        assert!(!functors.contains(&"m6".to_string()));
    }

    #[test]
    fn test_cyclic_dependency() {
        let result = resolve_test(
            &[
                ("a/db.cadhr", "#use(\"../b\").\nfoo(1).\n"),
                ("b/db.cadhr", "#use(\"../a\").\nbar(2).\n"),
            ],
            "a",
            &[],
        );
        assert!(matches!(result, Err(ModuleError::CyclicDependency { .. })));
    }

    #[test]
    fn test_file_not_found() {
        let result = resolve_test(&[], "nonexistent", &[]);
        assert!(matches!(result, Err(ModuleError::FileNotFound { .. })));
    }

    #[test]
    fn test_nested_module() {
        let functors = resolve_test(
            &[("sub/parts/db.cadhr", "bolt(1).\n")],
            "sub/parts",
            &[],
        )
        .unwrap();

        assert!(functors.contains(&"parts::bolt".to_string()));
    }

    #[test]
    fn test_non_use_clauses_preserved() {
        let clauses = vec![Clause::Fact(Term::Struct {
            functor: "hello".to_string(),
            args: vec![],
            span: None,
        })];

        let mut visited = HashSet::new();
        let result = resolve_modules(clauses, &[], &mut visited, &mut FileRegistry::new()).unwrap();
        assert_eq!(result.len(), 1);
        assert!(
            matches!(&result[0], Clause::Fact(Term::Struct { functor, .. }) if functor == "hello")
        );
    }
}
