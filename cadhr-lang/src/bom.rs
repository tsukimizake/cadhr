use crate::parse::{Term, term_as_number};
use crate::term_processor::{BuiltinFunctorSet, TermProcessor};

inventory::submit! {
    BuiltinFunctorSet {
        functors: &[("bom", &[2_usize] as &[usize])],
        resolve_args: false,
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum BomPropertyValue {
    Num(f64),
    Str(String),
}

impl BomPropertyValue {
    pub fn to_json_value(&self) -> serde_json::Value {
        match self {
            BomPropertyValue::Num(n) => serde_json::json!(*n),
            BomPropertyValue::Str(s) => serde_json::json!(s),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct BomEntry {
    pub name: String,
    pub properties: Vec<(String, BomPropertyValue)>,
}

impl BomEntry {
    pub fn to_json_value(&self) -> serde_json::Value {
        let mut props = serde_json::Map::new();
        for (key, val) in &self.properties {
            props.insert(key.clone(), val.to_json_value());
        }
        serde_json::json!({
            "name": self.name,
            "properties": props,
        })
    }
}

pub fn bom_entries_to_json(entries: &[BomEntry]) -> String {
    let arr: Vec<serde_json::Value> = entries.iter().map(|e| e.to_json_value()).collect();
    serde_json::to_string_pretty(&arr).unwrap()
}

#[derive(Debug, Clone)]
pub enum BomError {
    InvalidName(String),
    InvalidProperty(String),
}

impl std::fmt::Display for BomError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BomError::InvalidName(s) => write!(f, "BOM: invalid name: {}", s),
            BomError::InvalidProperty(s) => write!(f, "BOM: invalid property: {}", s),
        }
    }
}

pub struct BomExtractor;

impl<S> TermProcessor<S> for BomExtractor {
    type Output = Vec<BomEntry>;
    type Error = BomError;

    fn process(&self, terms: &[Term<S>]) -> Result<Self::Output, Self::Error> {
        terms
            .iter()
            .filter_map(|term| match term {
                Term::Struct { functor, args, .. } if functor == "bom" && args.len() == 2 => {
                    Some(bom_entry_from_args(args))
                }
                _ => None,
            })
            .collect()
    }
}

fn bom_entry_from_args<S>(args: &[Term<S>]) -> Result<BomEntry, BomError> {
    let name = match &args[0] {
        Term::StringLit { value } => value.clone(),
        other => return Err(BomError::InvalidName(format!("{:?}", other))),
    };

    let properties = match &args[1] {
        Term::List { items, .. } => items
            .iter()
            .map(property_from_term)
            .collect::<Result<Vec<_>, _>>()?,
        other => return Err(BomError::InvalidProperty(format!("{:?}", other))),
    };

    Ok(BomEntry { name, properties })
}

fn property_from_term<S>(term: &Term<S>) -> Result<(String, BomPropertyValue), BomError> {
    match term {
        Term::Struct { functor, args, .. } if args.len() == 1 => {
            let value = if let Some((r, _)) = term_as_number(&args[0]) {
                BomPropertyValue::Num(r.to_f64())
            } else if let Term::StringLit { value } = &args[0] {
                BomPropertyValue::Str(value.clone())
            } else {
                return Err(BomError::InvalidProperty(format!(
                    "{}({:?})",
                    functor, args[0]
                )));
            };
            Ok((functor.clone(), value))
        }
        other => Err(BomError::InvalidProperty(format!("{:?}", other))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{database, query};
    use crate::term_rewrite::execute;

    #[test]
    fn test_bom_extraction() {
        let db_src = "";
        let query_src = r#"bom("aluminum", [len(100)])."#;
        let (_, query_terms) = query(query_src).unwrap();
        let mut db = database(db_src).unwrap();
        let (resolved, _) = execute(&mut db, query_terms).unwrap();

        let entries = BomExtractor.process(&resolved).unwrap();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].name, "aluminum");
        assert_eq!(
            entries[0].properties,
            vec![("len".to_string(), BomPropertyValue::Num(100.0))]
        );
    }

    #[test]
    fn test_bom_with_string_property() {
        let db_src = "";
        let query_src = r#"bom("bolt", [material("steel"), count(4)])."#;
        let (_, query_terms) = query(query_src).unwrap();
        let mut db = database(db_src).unwrap();
        let (resolved, _) = execute(&mut db, query_terms).unwrap();

        let entries = BomExtractor.process(&resolved).unwrap();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].name, "bolt");
        assert_eq!(entries[0].properties.len(), 2);
        assert_eq!(
            entries[0].properties[0],
            (
                "material".to_string(),
                BomPropertyValue::Str("steel".to_string())
            )
        );
        assert_eq!(
            entries[0].properties[1],
            ("count".to_string(), BomPropertyValue::Num(4.0))
        );
    }

    #[test]
    fn test_bom_mixed_with_mesh_terms() {
        let db_src = "";
        let query_src = r#"cube(10, 20, 30), bom("plate", [thickness(5)])."#;
        let (_, query_terms) = query(query_src).unwrap();
        let mut db = database(db_src).unwrap();
        let (resolved, _) = execute(&mut db, query_terms).unwrap();

        let entries = BomExtractor.process(&resolved).unwrap();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].name, "plate");
    }

    /// `main(..., OUT)` の OUT 引数 (リスト) を取り出して平坦化する。
    fn unpack_main_out_list<S: Clone>(resolved: &[Term<S>]) -> Vec<Term<S>> {
        for t in resolved {
            if let Term::Struct { functor, args, .. } = t
                && functor == "main"
            {
                if let Some(Term::List { items, .. }) = args.last() {
                    return items.clone();
                }
            }
        }
        Vec::new()
    }

    #[test]
    fn test_bom_via_rule() {
        // main の最後の引数 (リスト) に bom 項を含めて runtime に届ける。
        let db_src =
            r#"main(L, OUT) :- OUT = [cube(L, L, L), bom("frame", [len(L)])]."#;
        let query_src = "main(60, OUT).";
        let (_, query_terms) = query(query_src).unwrap();
        let mut db = database(db_src).unwrap();
        let (resolved, _) = execute(&mut db, query_terms).unwrap();

        let items = unpack_main_out_list(&resolved);
        let entries = BomExtractor.process(&items).unwrap();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].name, "frame");
        assert_eq!(
            entries[0].properties,
            vec![("len".to_string(), BomPropertyValue::Num(60.0))]
        );
    }

    #[test]
    fn test_bom_via_rule_with_query_param() {
        use crate::parse::{collect_query_params, substitute_query_params};
        use crate::term_rewrite::infer_query_param_ranges;

        let db_src =
            r#"main(L, OUT) :- 50<L<2000, OUT = [cube(L, L, L), bom("frame", [len(L)])]."#;
        let query_src = "main(60, OUT).";
        let (_, query_terms) = query(query_src).unwrap();
        let mut db = database(db_src).unwrap();

        let mut query_params = collect_query_params(&query_terms);
        infer_query_param_ranges(&query_terms, &db, &mut query_params).unwrap();
        let substituted = substitute_query_params(
            &query_terms,
            &std::collections::HashMap::from([("L".to_string(), 60.0)]),
        );

        let (resolved, _) = execute(&mut db, substituted).unwrap();
        let items = unpack_main_out_list(&resolved);
        let entries = BomExtractor.process(&items).unwrap();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].name, "frame");
        assert_eq!(
            entries[0].properties,
            vec![("len".to_string(), BomPropertyValue::Num(60.0))]
        );
    }

    #[test]
    fn test_no_bom_terms() {
        let db_src = "";
        let query_src = "cube(10, 20, 30).";
        let (_, query_terms) = query(query_src).unwrap();
        let mut db = database(db_src).unwrap();
        let (resolved, _) = execute(&mut db, query_terms).unwrap();

        let entries = BomExtractor.process(&resolved).unwrap();
        assert!(entries.is_empty());
    }
}
