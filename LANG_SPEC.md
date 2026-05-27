# cadhr-lang v2 仕様 (Elm-like 関数型)

> 旧 v1 (Prolog ベース) はこのファイルの履歴版を参照。本ドキュメントは
> v1 を全廃する破壊的リデザイン。

## 1. リデザインの動機

cadhr-lang v1 は Prolog ライクで unification + clause + backtracking
を中核とした DSL だった。`Z1*T1 + Z2*T2 = 0` のような **等式で未束縛
変数を逆算** する書き方が中心だったが、

- 三角関数・無理数が絡む座標や図形の unify は浮動小数点誤差で破綻する
- 変数同士の高速 unify は一般には不可能 (筋が悪い)
- 静的型情報を持たないので LSP / evaluator / GUI で型を扱えない

これを解消するため、v2 では **Elm-like な静的型付き関数型** にリデザイン
する。Prolog の clause / unify / backtrack を捨て、関数評価 + HM 型推論
ベースに作り直す。

## 2. 確定方針 (リデザインで不変)

- 構文は **Elm 流**:
  - `module ... exposing (...)`, `import`, `type`, `type alias`
  - `let ... in`, `case ... of`, `|>`
  - 大文字始まりが型 / コンストラクタ、小文字始まりが変数 / 関数
  - curried 関数適用 (`cube 10 10 length`)
- 逆方向制約は **完全廃止**。`=` は単方向 let 束縛のみ
- ranged variable (GUI スライダー仕様) は **値レベルの `Range a` 型** で扱う
  (詳細は §3.5)
- 数値は **Float (内部 f64) 統一**。Rational は range inference の内部用途のみ
- ADT (`type Shape = Cube Float Float Float | Sphere Float | ...`) と
  `case` 網羅性検査を入れる
- 型推論は **HM + let-polymorphism**。top-level シグネチャは warn-only で推奨
- Curry を採用。引数 4 個以上のコンストラクタは **record 引数** が推奨イディオム
- builtin は `BuiltinRegistry` に 1 entry / 関数で集約 (名前 / 型 / docs / 評価)

## 3. 言語仕様の輪郭

### 3.1 サンプル

```elm
module Bolt exposing (main)

import Std exposing (Output, empty3d)

type alias Output =
    { models : List Shape3D
    , bom : List BomEntry
    , controls : List Control
    }

main : Float -> Int -> Output
main length holes =
    let
        bolt =
            cube 10 10 length
                |> translate3d (p3 0 0 0) (p3 0 0 5)
    in
    { models = [ bolt ]
    , bom = [ Length length ]
    , controls = []
    }

slider length = 6.0..80.0
slider holes  = 1..5
```

### 3.2 リテラル

- 数値: `42`, `3.14`, `-1.5` (要素型は context で `Int` / `Float`)
- 文字列: `"hello"`
- 真偽値: `True`, `False`
- リスト: `[1, 2, 3]`, `[]`
- Range: `1..10`, `6.0..80.0` (両端含む閉区間)
- レコード: `{ field = expr, ... }`, `{ r | field = expr }`
- タプルは採用しない (record で表現)

### 3.3 関数

```elm
cube : Float -> Float -> Float -> Shape3D
cube x y z = ...

addBolt : Float -> List Shape3D -> List Shape3D
addBolt length models =
    cube 10 10 length :: models
```

- 全関数は curried
- `f x y z` で適用、`f x y` で部分適用
- パイプ `|>` を多用 (`expr |> f a` は `f a expr` と等価)

### 3.4 型

```
Type ::= Int | Float | String | Bool
       | List a
       | Range a
       | Record { field : Type, ... }
       | Shape3D | Shape2D | PlacedShape2D | Path2D
       | Point2D | Point3D | Plane
       | a -> b
       | a (TypeVar)
       | Forall a. T (シグネチャレベル)
```

- 推論は HM + let-poly + generalization
- 多相は rank-1 (シグネチャ位置の `a` は全称量化)
- top-level シグネチャは warn-only で推奨 (省略可)

### 3.5 ranged variable と slider

```elm
boltLength : Range Float
boltLength = 1.0..100.0

shortBolt : Range Float
shortBolt = intersect boltLength (1.0..30.0)

main : Float -> Output
main length = ...

slider length = shortBolt
```

- `Range a` は組み込み型 (`a` は `Int` または `Float`)
- リテラル `lo..hi` は **両端含む閉区間** (Elm の `List.range` と一致)
- 集合演算: `intersect : Range a -> Range a -> Range a` を builtin で提供
  - `union` は結果が連続区間にならないケースがあるため将来課題
- `slider name = range_expr` は top-level decl の 1 種:
  - `name` は `main` の引数名と一致する必要がある
  - 右辺は `Range a` 型の **コンパイル時定数式**:
    - リテラル `1..10`
    - 別 `Range` 定数の参照 (`slider length = boltLength`)
    - builtin 集合演算 (`intersect (1..100) (5..50)`)
  - 右辺が `main` の引数値や他の slider 値に依存していたらエラー
    (本仕様では slider 同士は独立。将来 slider 間依存は別フェーズで検討)

### 3.6 main の規約

- `main : ... -> Output` (引数列が GUI スライダー仕様)
- `Output` は std で定義:
  ```elm
  type alias Output =
      { models : List Shape3D
      , bom : List BomEntry
      , controls : List Control
      }
  ```
- 各引数のスライダー範囲は `slider name = expr` で別途宣言
- 引数の現在値は `previews.json` に保存される

### 3.7 module / import

```elm
module Foo exposing (a, b)
module Foo exposing (..)

import Std exposing (Output, empty3d)
import Std as S
import Std.Gears
```

- モジュール `Foo` は search path 配下の `Foo.cadhr` に対応
- ドット区切りの階層 (`Std.Gears`) はディレクトリ階層 (`Std/Gears.cadhr`)
- `expose([...])` は `exposing (...)`、`expose(..)` は `exposing (..)` に対応
- builtin はモジュール prefix を受けない (現状通り)

## 4. 公開 API

```rust
pub fn compile(
    src: &str,
    search_paths: &[PathBuf],
) -> Result<CompiledProgram, Vec<Diagnostic>>;

pub fn main_signature(p: &CompiledProgram) -> &MainSignature;

pub struct MainSignature {
    pub args: Vec<MainArg>,
    pub return_type: Type,
}

pub struct MainArg {
    pub name: String,
    pub ty: Type,
    pub slider: Option<SliderDecl>,
}

pub struct SliderDecl {
    pub lo: f64,
    pub hi: f64,
    pub elem_ty: ElemTy, // Int or Float
}

pub struct Inputs {
    pub values: HashMap<String, f64>,            // main の各引数の現在値
    pub control_overrides: HashMap<String, Point3D>,
}

pub fn run_main(
    p: &CompiledProgram,
    inputs: &Inputs,
) -> Result<MainOutput, Diagnostic>;

pub struct MainOutput {
    pub models: Vec<Shape3D>,
    pub bom: Vec<BomEntry>,
    pub controls: Vec<Control>,
}
```

## 5. crate 構成

```
cadhr-lang/                    # 言語コア
  src/
    lib.rs                     # 公開 API: compile / run_main / MainSignature
    syntax/                    # Phase 1
      ast.rs / lex.rs / parse.rs / pretty.rs
    sema/                      # Phase 2
      resolve.rs / typecheck.rs / exhaustive.rs / slider.rs
    runtime/                   # Phase 3
      value.rs / eval.rs / builtin.rs
    cad/                       # 旧 cadhr-lang 由来の言語非依存層
      manifold_bridge.rs / constraint.rs / rational.rs
      bom.rs / collision.rs / sweep.rs / bezier.rs / assertions.rs
    module.rs                  # ModuleSearch
    diagnostic.rs              # 統一 Diagnostic 型
  legacy_prolog/               # 旧 v1 実装 (参考用、cargo はビルドしない)

cadhr/        (root, GUI)      # iced。Phase 4 で新 API に接続
cadhr-lsp/    (submodule)      # tower-lsp。Phase 8 で更新
tree-sitter-cadhr-lang/ (submodule)  # Phase 7 で grammar.js を Elm 流に書き換え
```

## 6. 実装フェーズ

詳細は `/Users/tsukimizake/.claude/plans/refactored-launching-seahorse.md` を参照。

- Phase 0: 準備 (legacy 退避 / chumsky 追加 / 本仕様書)
- Phase 1: AST + Parser (chumsky)
- Phase 2: 名前解決 + HM 型推論
- Phase 3: Evaluator + Builtin bridge
- Phase 4: Range 型 + Ranged variable + GUI 接合
- Phase 5: ADT + Pattern Match の網羅運用
- Phase 6: Module / Import
- Phase 7: tree-sitter grammar 更新
- Phase 8: cadhr-lsp 更新
- Phase 9: std + tests 書き直し
- Phase 10: cadhr-proj 等の確認

## 7. 旧版 (v1, Prolog) との関係

旧 v1 仕様 (このファイルの履歴版) は `cadhr-lang/legacy_prolog/` のコードに
対応する。v2 は v1 の clause / unify / backtrack / 等式制約を全廃する破壊的
変更。互換性は意識しない (cadhr-proj 等のユーザコードは Phase 10 で個別に
書き直す)。
