use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{
    Ident, Token, Type, braced, parenthesized,
    parse_macro_input,
};

// ── Parsed representation ──

struct ManifoldEntry {
    name_override: Option<String>,
    no_variant: bool,
    also_arities: Vec<usize>,
    variant_name: Ident,
    body: VariantBody,
}

enum VariantBody {
    Unit,
    Struct(Vec<(Ident, Type)>),
    Tuple(Vec<Type>),
}

struct ManifoldDef {
    entries: Vec<ManifoldEntry>,
}

// ── Parsing ──

impl Parse for ManifoldDef {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut entries = Vec::new();
        while !input.is_empty() {
            entries.push(input.parse::<ManifoldEntry>()?);
            let _ = input.parse::<Token![;]>();
        }
        Ok(ManifoldDef { entries })
    }
}

impl Parse for ManifoldEntry {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut name_override = None;
        let mut no_variant = false;
        let mut also_arities = Vec::new();

        while input.peek(Token![@]) {
            input.parse::<Token![@]>()?;
            let attr_name: Ident = input.parse()?;
            match attr_name.to_string().as_str() {
                "name" => {
                    let content;
                    parenthesized!(content in input);
                    let lit: syn::LitStr = content.parse()?;
                    name_override = Some(lit.value());
                }
                "no_variant" => {
                    no_variant = true;
                }
                "also_arity" => {
                    let content;
                    parenthesized!(content in input);
                    while !content.is_empty() {
                        let lit: syn::LitInt = content.parse()?;
                        also_arities.push(lit.base10_parse::<usize>()?);
                        let _ = content.parse::<Token![,]>();
                    }
                }
                other => {
                    return Err(syn::Error::new(
                        attr_name.span(),
                        format!("unknown annotation @{}", other),
                    ));
                }
            }
        }

        let variant_name: Ident = input.parse()?;

        let body = if input.peek(syn::token::Brace) {
            let content;
            braced!(content in input);
            let mut fields = Vec::new();
            while !content.is_empty() {
                let field_name: Ident = content.parse()?;
                content.parse::<Token![:]>()?;
                let field_type: Type = content.parse()?;
                fields.push((field_name, field_type));
                let _ = content.parse::<Token![,]>();
            }
            VariantBody::Struct(fields)
        } else if input.peek(syn::token::Paren) {
            let content;
            parenthesized!(content in input);
            let mut types = Vec::new();
            while !content.is_empty() {
                let ty: Type = content.parse()?;
                types.push(ty);
                let _ = content.parse::<Token![,]>();
            }
            VariantBody::Tuple(types)
        } else {
            VariantBody::Unit
        };

        Ok(ManifoldEntry {
            name_override,
            no_variant,
            also_arities,
            variant_name,
            body,
        })
    }
}

// ── Code generation ──

fn to_lowercase_functor(entry: &ManifoldEntry) -> String {
    match &entry.name_override {
        Some(s) => s.clone(),
        None => entry.variant_name.to_string().to_lowercase(),
    }
}

fn primary_arity(body: &VariantBody) -> usize {
    match body {
        VariantBody::Unit => 0,
        VariantBody::Struct(fields) => fields.len(),
        VariantBody::Tuple(types) => types.len(),
    }
}

#[proc_macro]
pub fn define_manifold_expr(input: TokenStream) -> TokenStream {
    let def = parse_macro_input!(input as ManifoldDef);

    // 1. ManifoldExpr enum variants (skip @no_variant)
    let variants: Vec<TokenStream2> = def
        .entries
        .iter()
        .filter(|e| !e.no_variant)
        .map(|e| {
            let name = &e.variant_name;
            match &e.body {
                VariantBody::Unit => quote! { #name },
                VariantBody::Struct(fields) => {
                    let field_defs: Vec<_> = fields
                        .iter()
                        .map(|(fname, fty)| quote! { #fname: #fty })
                        .collect();
                    quote! { #name { #(#field_defs),* } }
                }
                VariantBody::Tuple(types) => {
                    quote! { #name( #(#types),* ) }
                }
            }
        })
        .collect();

    // 2. BUILTIN_FUNCTORS table (all entries including @no_variant)
    let functor_entries: Vec<TokenStream2> = def
        .entries
        .iter()
        .map(|e| {
            let name = to_lowercase_functor(e);
            let pa = primary_arity(&e.body);
            let mut arities = vec![pa];
            for &a in &e.also_arities {
                if !arities.contains(&a) {
                    arities.push(a);
                }
            }
            let arity_lits: Vec<_> = arities.iter().map(|a| quote! { #a }).collect();
            quote! { (#name, &[ #(#arity_lits),* ] as &[usize]) }
        })
        .collect();

    // 3. ManifoldTag enum (all entries) + FromStr
    let tag_variants: Vec<TokenStream2> = def
        .entries
        .iter()
        .map(|e| {
            let name = &e.variant_name;
            quote! { #name }
        })
        .collect();

    let from_str_arms: Vec<TokenStream2> = def
        .entries
        .iter()
        .map(|e| {
            let functor_name = to_lowercase_functor(e);
            let variant = &e.variant_name;
            quote! { #functor_name => Ok(ManifoldTag::#variant) }
        })
        .collect();

    let output = quote! {
        #[derive(Debug, Clone)]
        pub enum ManifoldExpr {
            #(#variants),*
        }

        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum ManifoldTag {
            #(#tag_variants),*
        }

        impl ::std::str::FromStr for ManifoldTag {
            type Err = ();
            fn from_str(s: &str) -> Result<Self, ()> {
                match s {
                    #(#from_str_arms,)*
                    _ => Err(()),
                }
            }
        }

        pub const BUILTIN_FUNCTORS: &[(&str, &[usize])] = &[
            #(#functor_entries),*
        ];

        pub fn is_builtin_functor(functor: &str) -> bool {
            BUILTIN_FUNCTORS.iter().any(|(name, _)| *name == functor)
        }
    };

    output.into()
}
