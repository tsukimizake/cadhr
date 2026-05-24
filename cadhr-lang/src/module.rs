use std::collections::{HashMap, HashSet};
use std::fmt;
use std::path::{Path, PathBuf};

use crate::parse::{database, Clause, FileRegistry, RecordField, Term};
use crate::term_processor::{is_builtin_functor, BuiltinFunctorSet};

// `__make_record__(RecordNameStr, Spec, R)` は auto-generated `make_NAME` rule の
// 本体から呼ばれる builtin。term_rewrite が intercept して handle_make_record に
// 委譲する。引数を builtin 側で自動 resolve すると Spec list 内の field 名 struct
// (`x(3)` 等) が再帰展開されてしまうため、resolve_args は false にする。
inventory::submit! {
    BuiltinFunctorSet {
        functors: &[("__make_record__", &[3_usize] as &[usize])],
        resolve_args: false,
    }
}

/// `:- record NAME(...).` を実行時参照しやすい形に保管したもの。
/// `make_NAME/2` builtin の引数解決と `NAME_FIELD/2` / `set_FIELD_of_NAME/3`
/// 自動生成の両方で参照される。
#[derive(Clone, Debug, PartialEq)]
pub struct RecordDecl {
    pub name: String,
    pub fields: Vec<RecordField>,
}

/// `#use` 解決の状態を管理する。
///
/// `in_progress` は現在 DFS で解決中のファイル集合 (真の cycle 検出用)。
/// `loaded` は読み込み済みファイル → そのファイルの内部 #use 解決後の clauses
/// (prefix/expose は適用前) のキャッシュ。ダイヤモンドインポート (A から std を
/// `#use` し、B からも std と A を `#use` するようなケース) で同じファイルが
/// 複数回 import されても、cache を返すことで再 parse を回避しつつ cycle と
/// 誤検出しないようにする。
pub struct ModuleResolver {
    in_progress: HashSet<PathBuf>,
    loaded: HashMap<PathBuf, Vec<Clause>>,
    /// resolve_modules を通じて収集された record 宣言。グローバル名前空間。
    /// 同名 decl の重複は field 構造が一致すれば idempotent、不一致ならエラー。
    pub record_decls: HashMap<String, RecordDecl>,
}

impl ModuleResolver {
    pub fn new() -> Self {
        Self {
            in_progress: HashSet::new(),
            loaded: HashMap::new(),
            record_decls: HashMap::new(),
        }
    }
}

impl Default for ModuleResolver {
    fn default() -> Self {
        Self::new()
    }
}

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
    /// 同名 record decl が異なる field 構造で複数定義された。
    /// ダイヤモンドインポートで同一ソース由来の場合は idempotent なので発生しない。
    RecordDeclConflict {
        name: String,
        first: Vec<RecordField>,
        second: Vec<RecordField>,
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
            ModuleError::RecordDeclConflict {
                name,
                first,
                second,
            } => {
                let field_summary = |fs: &[RecordField]| {
                    fs.iter()
                        .map(|f| f.name.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                };
                write!(
                    f,
                    "record decl conflict for `{}`: first=({}), second=({})",
                    name,
                    field_summary(first),
                    field_summary(second),
                )
            }
        }
    }
}

impl std::error::Error for ModuleError {}

pub fn resolve_modules(
    clauses: Vec<Clause>,
    include_paths: &[PathBuf],
    resolver: &mut ModuleResolver,
    file_registry: &mut FileRegistry,
) -> Result<Vec<Clause>, ModuleError> {
    resolve_modules_inner(clauses, include_paths, resolver, file_registry)
}

fn resolve_modules_inner(
    clauses: Vec<Clause>,
    include_paths: &[PathBuf],
    resolver: &mut ModuleResolver,
    file_registry: &mut FileRegistry,
) -> Result<Vec<Clause>, ModuleError> {
    let mut result = Vec::new();

    for clause in clauses {
        match clause {
            Clause::Use { path, expose, .. } => {
                let resolved = resolve_use(&path, &expose, include_paths, resolver, file_registry)?;
                result.extend(resolved);
            }
            Clause::RecordDecl { name, fields, span } => {
                register_record_decl(resolver, &name, &fields)?;
                // decl 自体も clauses に残す。term_rewrite で make_NAME builtin が参照する。
                result.push(Clause::RecordDecl {
                    name: name.clone(),
                    fields: fields.clone(),
                    span,
                });
                // record 宣言と同じ位置に getter/setter Fact を inline 生成する。
                // 通常の rule と同じく、外側 resolve_use の prefix_clause を通って
                // モジュール名で prefix される。これにより `make_X` / `X_field` /
                // `set_field_of_X` も通常の述語と同じ namespace 規則に従う。
                result.extend(generate_predicates_for_record(&name, &fields));
            }
            other => result.push(other),
        }
    }

    Ok(result)
}

/// 1 つの record decl について、`make_NAME/2` constructor rule、
/// `NAME_FIELD/2` getter Fact、`set_FIELD_of_NAME/3` setter Fact を
/// auto-generate して返す。`record` 宣言と同じ位置に inline 注入するので、
/// 外側のモジュール prefix が自然に適用される (= 通常の rule と同じ namespace
/// 規則に従う)。`make_NAME` の本体は builtin `__make_record__` を呼ぶだけで、
/// 実際の構築は term_rewrite が行う。
fn generate_predicates_for_record(name: &str, fields: &[RecordField]) -> Vec<Clause> {
    let mut result = Vec::new();

    // make_NAME(Spec, R) :- __make_record__("NAME", Spec, R).
    // record 名を StringLit にすることで、prefix されてもデータが変わらない。
    result.push(Clause::Rule {
        head: struct_term(
            &format!("make_{}", name),
            vec![named_var("Spec"), named_var("R")],
        ),
        body: vec![struct_term(
            "__make_record__",
            vec![
                Term::StringLit {
                    value: name.to_string(),
                },
                named_var("Spec"),
                named_var("R"),
            ],
        )],
        return_type: None,
    });

    let arity = fields.len();
    for (i, field) in fields.iter().enumerate() {
        // Getter: NAME_FIELD(NAME(_, ..., V, ..., _), V).
        // V を i 番目に、他は無名 (`_`)。
        let mut record_args = (0..arity).map(|_| anon_var()).collect::<Vec<Term>>();
        record_args[i] = named_var("V");
        let record_pat = struct_term(name, record_args);
        result.push(Clause::Fact {
            head: struct_term(
                &format!("{}_{}", name, field.name),
                vec![record_pat, named_var("V")],
            ),
            return_type: None,
        });

        // Setter: set_FIELD_of_NAME(V, NAME(_/Old, Others...), NAME(V, SameOthers...)).
        // i 番目以外は同じ変数 (両 record で共有)、i 番目は新値 V。
        let mut r0_args = Vec::with_capacity(arity);
        let mut r1_args = Vec::with_capacity(arity);
        for j in 0..arity {
            if j == i {
                r0_args.push(anon_var()); // 旧値は破棄
                r1_args.push(named_var("V"));
            } else {
                let shared = named_var(&format!("F{}", j));
                r0_args.push(shared.clone());
                r1_args.push(shared);
            }
        }
        let r0 = struct_term(name, r0_args);
        let r1 = struct_term(name, r1_args);
        result.push(Clause::Fact {
            head: struct_term(
                &format!("set_{}_of_{}", field.name, name),
                vec![named_var("V"), r0, r1],
            ),
            return_type: None,
        });
    }
    result
}

fn anon_var() -> Term {
    Term::Var {
        name: "_".to_string(),
        scope: (),
        default_value: None,
        min: None,
        max: None,
        span: None,
        type_annotation: None,
    }
}

fn named_var(name: &str) -> Term {
    Term::Var {
        name: name.to_string(),
        scope: (),
        default_value: None,
        min: None,
        max: None,
        span: None,
        type_annotation: None,
    }
}

fn struct_term(functor: &str, args: Vec<Term>) -> Term {
    Term::Struct {
        functor: functor.to_string(),
        args,
        span: None,
    }
}

/// record decl をテーブルに登録する。同名既存 decl があれば field 構造を比較して
/// 完全一致なら idempotent (黙って許容)、不一致ならエラー。
fn register_record_decl(
    resolver: &mut ModuleResolver,
    name: &str,
    fields: &[RecordField],
) -> Result<(), ModuleError> {
    if let Some(existing) = resolver.record_decls.get(name) {
        if existing.fields == fields {
            return Ok(());
        }
        return Err(ModuleError::RecordDeclConflict {
            name: name.to_string(),
            first: existing.fields.clone(),
            second: fields.to_vec(),
        });
    }
    resolver.record_decls.insert(
        name.to_string(),
        RecordDecl {
            name: name.to_string(),
            fields: fields.to_vec(),
        },
    );
    Ok(())
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
    resolver: &mut ModuleResolver,
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

    // 既にロード済みなら parse をスキップしてキャッシュを使う (ダイヤモンドインポート対応)。
    // 同じファイルを別経路で `#use` した場合でも、prefix/expose は呼び出しごとに
    // 都度適用するため、キャッシュには prefix 前の状態を保持する。
    let inner_clauses = if let Some(cached) = resolver.loaded.get(&canonical) {
        cached.clone()
    } else {
        if !resolver.in_progress.insert(canonical.clone()) {
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

        // 子ファイル内の `#use(...)` を解決するときの include path:
        // 「子ファイルの親 dir」と「呼び出し側で指定された include_paths」を併用する。
        // 後者を入れておかないと、ライブラリ (std など) が深いネスト #use 経由から
        // 見えなくなる。
        let mut child_include_paths: Vec<PathBuf> = file_path
            .parent()
            .map(|p| vec![p.to_path_buf()])
            .unwrap_or_default();
        child_include_paths.extend(include_paths.iter().cloned());

        let clauses =
            resolve_modules_inner(clauses, &child_include_paths, resolver, file_registry)?;

        resolver.in_progress.remove(&canonical);
        resolver.loaded.insert(canonical.clone(), clauses.clone());
        clauses
    };

    let module_name = module_name_from_path(module_path);
    let expose_set: HashSet<&str> = expose.iter().map(|s| s.as_str()).collect();

    let mut result = Vec::new();
    for clause in inner_clauses {
        let prefixed = prefix_clause(&clause, &module_name, &resolver.record_decls);
        result.push(prefixed);

        if let Some(functor) = clause_head_functor(&clause) {
            if is_functor_exposed(&functor, &expose_set, &resolver.record_decls) {
                result.push(clause);
            }
        }
    }

    Ok(result)
}

/// `functor` が expose 設定で公開されるか判定する。
fn is_functor_exposed(
    functor: &str,
    expose_set: &HashSet<&str>,
    record_decls: &HashMap<String, RecordDecl>,
) -> bool {
    if expose_set.contains(functor) {
        return true;
    }
    if let Some(record_name) = functor.strip_prefix("make_") {
        if expose_set.contains(record_name) && record_decls.contains_key(record_name) {
            return true;
        }
    }
    for (record_name, decl) in record_decls {
        if !expose_set.contains(record_name.as_str()) {
            continue;
        }
        for field in &decl.fields {
            if functor == format!("{}_{}", record_name, field.name)
                || functor == format!("set_{}_of_{}", field.name, record_name)
            {
                return true;
            }
        }
    }
    false
}

fn clause_head_functor(clause: &Clause) -> Option<String> {
    match clause {
        Clause::Fact { head, .. } => term_functor(head),
        Clause::Rule { head, .. } => term_functor(head),
        Clause::Use { .. } | Clause::RecordDecl { .. } => None,
    }
}

fn term_functor(term: &Term) -> Option<String> {
    match term {
        Term::Struct { functor, .. } => Some(functor.clone()),
        _ => None,
    }
}

fn prefix_clause(
    clause: &Clause,
    module_name: &str,
    record_decls: &HashMap<String, RecordDecl>,
) -> Clause {
    match clause {
        Clause::Fact { head, return_type } => Clause::Fact {
            head: prefix_term(head, module_name, record_decls),
            return_type: return_type.clone(),
        },
        Clause::Rule {
            head,
            body,
            return_type,
        } => Clause::Rule {
            head: prefix_term(head, module_name, record_decls),
            body: body
                .iter()
                .map(|t| prefix_term(t, module_name, record_decls))
                .collect(),
            return_type: return_type.clone(),
        },
        // RecordDecl はグローバル decl テーブルに乗るので prefix しない。
        // Use directive はクライアントが直接使うことがないのでそのまま。
        Clause::Use { .. } | Clause::RecordDecl { .. } => clause.clone(),
    }
}

/// record の型名そのもの (= record decl で宣言された名前) は global namespace に
/// 属する。`prefix_term` でこの名前を struct functor として見つけたら prefix しない。
/// これは「record TYPE は global、record OPERATION (make/getter/setter) はモジュール
/// 規則に従う」というモデル: 異なる経路 (user 直接構築 vs module 内 `make_X` 経由) で
/// 構築された同型 record 同士が unify できるようにするための最小限の例外。
fn is_record_constructor(functor: &str, record_decls: &HashMap<String, RecordDecl>) -> bool {
    record_decls.contains_key(functor)
}

fn prefix_term(
    term: &Term,
    module_name: &str,
    record_decls: &HashMap<String, RecordDecl>,
) -> Term {
    match term {
        Term::Struct {
            functor,
            args,
            span,
        } => {
            let prefixed_functor = if is_builtin_functor(functor)
                || is_record_constructor(functor, record_decls)
            {
                functor.clone()
            } else {
                format!("{}::{}", module_name, functor)
            };
            Term::Struct {
                functor: prefixed_functor,
                args: args
                    .iter()
                    .map(|a| prefix_term(a, module_name, record_decls))
                    .collect(),
                span: *span,
            }
        }
        Term::List { items, tail } => Term::List {
            items: items
                .iter()
                .map(|i| prefix_term(i, module_name, record_decls))
                .collect(),
            tail: tail
                .as_ref()
                .map(|t| Box::new(prefix_term(t, module_name, record_decls))),
        },
        Term::InfixExpr { op, left, right } => Term::InfixExpr {
            op: *op,
            left: Box::new(prefix_term(left, module_name, record_decls)),
            right: Box::new(prefix_term(right, module_name, record_decls)),
        },
        Term::Eq { left, right } => Term::Eq {
            left: Box::new(prefix_term(left, module_name, record_decls)),
            right: Box::new(prefix_term(right, module_name, record_decls)),
        },
        _ => term.clone(),
    }
}

fn set_file_id_in_clause(clause: &mut Clause, file_id: u16) {
    match clause {
        Clause::Fact { head, .. } => set_file_id_in_term(head, file_id),
        Clause::Rule { head, body, .. } => {
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
        Clause::RecordDecl { fields, span, .. } => {
            if let Some(s) = span {
                s.file_id = file_id;
            }
            for field in fields {
                if let Some(default) = &mut field.default {
                    set_file_id_in_term(default, file_id);
                }
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

        let mut resolver = ModuleResolver::new();
        let result = resolve_modules(
            clauses,
            &[dir.path().to_path_buf()],
            &mut resolver,
            &mut FileRegistry::new(),
        )?;

        let functors: Vec<String> = result
            .iter()
            .filter_map(|c| match c {
                Clause::Fact {
                    head: Term::Struct { functor, .. },
                    ..
                } => Some(functor.clone()),
                Clause::Rule {
                    head: Term::Struct { functor, .. },
                    ..
                } => Some(functor.clone()),
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

    /// ダイヤモンドインポートで同じ record decl が複数経路から来ても、
    /// field 構造が一致するので idempotent に扱われる。
    #[test]
    fn test_diamond_import_record_decl_idempotent() {
        let dir = tempfile::tempdir().unwrap();
        let std_path = dir.path().join("std/db.cadhr");
        let a_path = dir.path().join("a/db.cadhr");
        let top_path = dir.path().join("top/db.cadhr");
        for (full, contents) in &[
            (&std_path, ":- record output(models=[], bom=[]).\n"),
            (&a_path, "#use(\"../std\").\n"),
            (&top_path, "#use(\"../std\").\n#use(\"../a\").\n"),
        ] {
            fs::create_dir_all(full.parent().unwrap()).unwrap();
            fs::write(full, contents).unwrap();
        }

        let clauses = vec![Clause::Use {
            path: "top".to_string(),
            expose: vec![],
            span: None,
        }];

        let mut resolver = ModuleResolver::new();
        let _ = resolve_modules(
            clauses,
            &[dir.path().to_path_buf()],
            &mut resolver,
            &mut FileRegistry::new(),
        )
        .expect("diamond record decl should be idempotent");
        assert!(resolver.record_decls.contains_key("output"));
    }

    /// std 経由で record を `#use` し、`expose([point])` で派生述語まで自動 expose されることを確認する。
    /// expose リストに record 名を 1 つ書けば make_NAME/NAME_FIELD/set_FIELD_of_NAME が
    /// 全て unprefixed で見えるようになる。
    #[test]
    fn test_record_predicates_callable_from_use_module() {
        use crate::term_rewrite::execute;
        use crate::parse::{database, query};
        let dir = tempfile::tempdir().unwrap();
        let std_path = dir.path().join("std/db.cadhr");
        fs::create_dir_all(std_path.parent().unwrap()).unwrap();
        fs::write(&std_path, ":- record point(x=0, y=0).\n").unwrap();

        // user db: std を expose([point]) で use し、派生述語が unprefixed で使えるか確認。
        let user_src = "#use(\"std\", expose([point])).\n\
                        test :- make_point([x(3), y(4)], R), point_x(R, X), assert_eq(X, 3).";
        let user_clauses = database(user_src).expect("parse user db");
        let mut resolver = ModuleResolver::new();
        let mut registry = FileRegistry::new();
        let mut db = resolve_modules(
            user_clauses,
            &[dir.path().to_path_buf()],
            &mut resolver,
            &mut registry,
        )
        .expect("resolve modules");

        let q = query("test.").expect("parse query").1;
        execute(&mut db, q).expect("test should succeed");
    }

    /// expose なしでは派生述語に `std::` prefix が必要であることを確認する。
    #[test]
    fn test_record_predicates_require_qualification_without_expose() {
        use crate::parse::{database, query};
        use crate::term_rewrite::execute;
        let dir = tempfile::tempdir().unwrap();
        let std_path = dir.path().join("std/db.cadhr");
        fs::create_dir_all(std_path.parent().unwrap()).unwrap();
        fs::write(&std_path, ":- record point(x=0, y=0).\n").unwrap();

        // expose なし、bare 呼び出し → 失敗するはず
        let user_src = "#use(\"std\").\n\
                        test :- make_point([x(3), y(4)], R), point_x(R, X), assert_eq(X, 3).";
        let user_clauses = database(user_src).expect("parse user db");
        let mut resolver = ModuleResolver::new();
        let mut db = resolve_modules(
            user_clauses,
            &[dir.path().to_path_buf()],
            &mut resolver,
            &mut FileRegistry::new(),
        )
        .expect("resolve modules");
        let q = query("test.").expect("parse query").1;
        assert!(
            execute(&mut db, q).is_err(),
            "bare make_point/point_x without expose should fail"
        );

        // qualified 呼び出しなら expose なしでも動く
        let user_src2 = "#use(\"std\").\n\
                         test :- std::make_point([x(3), y(4)], R), std::point_x(R, X), assert_eq(X, 3).";
        let user_clauses2 = database(user_src2).expect("parse user db");
        let mut resolver2 = ModuleResolver::new();
        let mut db2 = resolve_modules(
            user_clauses2,
            &[dir.path().to_path_buf()],
            &mut resolver2,
            &mut FileRegistry::new(),
        )
        .expect("resolve modules");
        let q2 = query("test.").expect("parse query").1;
        execute(&mut db2, q2).expect("qualified call should succeed");
    }

    /// モジュール内 rule body から make_X / X_FIELD を呼び出せることを確認する。
    /// module 内の `make_point` は prefix された `std::make_point` 経由で auto-rule
    /// に解決し、その body の `__make_record__` が builtin として動く。
    #[test]
    fn test_record_predicates_callable_from_inside_module() {
        use crate::parse::{database, query};
        use crate::term_rewrite::execute;
        let dir = tempfile::tempdir().unwrap();
        let std_path = dir.path().join("std/db.cadhr");
        fs::create_dir_all(std_path.parent().unwrap()).unwrap();
        fs::write(
            &std_path,
            ":- record point(x=0, y=0).\n\
             helper_build(X_VAL, Y_VAL, R) :- make_point([x(X_VAL), y(Y_VAL)], R).\n\
             helper_x(R, X) :- point_x(R, X).\n",
        )
        .unwrap();

        // 敢えて expose せず、qualified で helper を呼び出す。helper の中で bare の
        // `make_point` / `point_x` が prefix されて `std::make_point` / `std::point_x`
        // になっていても、それぞれ auto-rule にマッチして動く。
        let user_src = "#use(\"std\").\n\
                        test :- std::helper_build(3, 4, R), std::helper_x(R, X), assert_eq(X, 3).";
        let user_clauses = database(user_src).expect("parse user db");
        let mut resolver = ModuleResolver::new();
        let mut db = resolve_modules(
            user_clauses,
            &[dir.path().to_path_buf()],
            &mut resolver,
            &mut FileRegistry::new(),
        )
        .expect("resolve modules");
        let q = query("test.").expect("parse query").1;
        execute(&mut db, q).expect("module-internal record use should succeed");
    }

    /// 同名 record を field 構造の異なる形で 2 つ定義するとエラー。
    #[test]
    fn test_record_decl_conflict() {
        let dir = tempfile::tempdir().unwrap();
        let a_path = dir.path().join("a/db.cadhr");
        let b_path = dir.path().join("b/db.cadhr");
        let top_path = dir.path().join("top/db.cadhr");
        for (full, contents) in &[
            (&a_path, ":- record output(models=[]).\n"),
            (&b_path, ":- record output(models=[], bom=[]).\n"),
            (&top_path, "#use(\"../a\").\n#use(\"../b\").\n"),
        ] {
            fs::create_dir_all(full.parent().unwrap()).unwrap();
            fs::write(full, contents).unwrap();
        }

        let clauses = vec![Clause::Use {
            path: "top".to_string(),
            expose: vec![],
            span: None,
        }];

        let mut resolver = ModuleResolver::new();
        let result = resolve_modules(
            clauses,
            &[dir.path().to_path_buf()],
            &mut resolver,
            &mut FileRegistry::new(),
        );
        assert!(matches!(result, Err(ModuleError::RecordDeclConflict { .. })));
    }

    /// ダイヤモンドインポート: top が a と std を use、a も std を use する。
    /// 古い実装では visited が DFS 後に削除されないため cyclic と誤検出していた。
    /// loaded キャッシュを使うように修正したので成功するはず。
    #[test]
    fn test_diamond_import() {
        let functors = resolve_test(
            &[
                ("std/db.cadhr", "shared(1).\n"),
                ("a/db.cadhr", "#use(\"../std\").\nfoo(2).\n"),
                ("top/db.cadhr", "#use(\"../std\").\n#use(\"../a\").\nbar(3).\n"),
            ],
            "top",
            &[],
        )
        .unwrap();

        // top 経由で std::shared が直接、A::std::shared が a 経由で取れる。
        // 同じソース由来でも別名空間に居るだけなのでエラーにならない。
        assert!(functors.iter().any(|f| f == "top::std::shared"));
        assert!(functors.iter().any(|f| f == "top::a::std::shared"));
        assert!(functors.iter().any(|f| f == "top::a::foo"));
        assert!(functors.iter().any(|f| f == "top::bar"));
    }

    #[test]
    fn test_non_use_clauses_preserved() {
        let clauses = vec![Clause::Fact {
            head: Term::Struct {
                functor: "hello".to_string(),
                args: vec![],
                span: None,
            },
            return_type: None,
        }];

        let mut resolver = ModuleResolver::new();
        let result =
            resolve_modules(clauses, &[], &mut resolver, &mut FileRegistry::new()).unwrap();
        assert_eq!(result.len(), 1);
        assert!(matches!(
            &result[0],
            Clause::Fact { head: Term::Struct { functor, .. }, .. } if functor == "hello"
        ));
    }
}
