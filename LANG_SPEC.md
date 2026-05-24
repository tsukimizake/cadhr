# cadhr-lang 再設計仕様 (v1)

> 既存実装は段階的拡張の結果、**型の不在** に起因する複数の症状が積み重なって
> いる。本ドキュメントは「全て再実装する」前提で、ドメイン (パラメトリック 3D CAD)
> に必要十分な仕様を再定義する。

## 1. 再実装の動機 — 吐き気の正体

問題の根は **型システムが無いこと**。そこから連鎖して複数の症状が出ている。

### 1.1 型と型推論の不在 (根本)

- AST は `Term::Var / Number / Struct / List / StringLit / InfixExpr / Eq` のみで、値の型 (`Number` / `Shape3D` / `Shape2D` / `Path2D` / `Plane` / `List(T)` / `Record(R)`) を区別する手段が無い
- 引数が `Shape2D` であるべき場所に `Path2D` を渡しても parse は通り、`manifold_bridge::evaluate()` の深いところで実行時エラー (`polygon_rings_or_err`) になる
- ビルトインは `BUILTIN_FUNCTORS: &[(&str, &[usize])]` で名前と arity だけ、引数の型は文字列マッチ + 個別 parser (`Model2D::parse_2d_op_arg` 等) で都度確認
- LSP の hover/completion は型を答えられないので「関数名と arity」しか提示できない

### 1.2 型ごとの empty 値が無い

`std/db.cadhr`:

```prolog
:- record output(models=[], bom=[], controls=[]).
```

`models` は `List(Shape3D)`、`bom` は `List(BomEntry)`、`controls` は `List(Control)`。
本来 default は型ごとに違うはずだが、全部 `[]` で書かざるを得ない。これは型不在の症状。

新仕様では `Model2D::empty` / `Model3D::empty` のような **型固有の zero / unit 値** を
言語レベルで提供する。

### 1.3 record の魔法 (型不在の症状)

`:- record output(models=[], bom=[], controls=[]).` 1 行で:

- `make_output/2` (term_rewrite 内 builtin として intercept)
- `output_models/2`, `output_bom/2`, `output_controls/2` (auto-generated Fact)
- `set_models_of_output/3`, ... (auto-generated Fact)

を生成。これらは `#use` の prefix を受けずグローバル。静的な field access (`p.x`) が
あれば全て不要。

### 1.4 ビルトイン登録の分散 (型不在の症状)

1 ビルトイン追加で 6 ファイルを編集する:

| ファイル | 何を書く |
|----------|---------|
| `manifold_bridge.rs` の `BUILTIN_FUNCTORS` | 名前と arity リスト |
| 同じく `FunctorTag` enum | バリアント |
| 同じく `FromStr` / `Display` impl | 文字列との往復 |
| 同じく `Manifold` 構築 enum | データ持ち |
| 同じく `evaluate()` の match arm | 実行ロジック |
| `cadhr-lsp/src/completion.rs` & `hover.rs` | LSP 用 docs |

型 (引数型と戻り型) があれば名前・型・実装・docs を 1 entry に集約できる。

### 1.5 範囲・デフォルトの過剰な構文 (default は廃止)

現状の `Term::Var` に詰まっている annotation:

- `X@25` (default)
- `0<X<10` (range)
- `0<=X@20<=50` (両方)

このうち **default** (`@N`) は本来「セッションごとに変わる値」であり、コードに書く
べきではない。GUI の preview window で操作 → `previews.json` に保存する形に移行。

**range** (`0<=X<=80` 等) は「パーツ仕様」として残す。例えば M5 ボルトの長さは規格上
6..=80mm という制約はコードで表現したい。シグネチャの引数位置に inline で書ける。

## 2. 設計目標

1. **型を持つ**: AST に型注釈を持たせ、推論する。Shape2D/Path2D 等のドメイン型を区別
2. **型ごとの empty 値**: `[]` を default に書かされない
3. **静的な record**: field access はネイティブ syntax。`make_X` 等の auto-generation 廃止
4. **ビルトイン 1 entry**: 名前・型・評価・docs を集約
5. **default annotation 廃止**: `X@25` の構文を捨て、値は GUI/json で管理
6. **range は維持**: シグネチャ引数位置に inline (`6<=Length<=80`)。パーツ仕様として
7. **panic over fallback**: 仕様外の入力は明確にエラー


## 3. 言語の輪郭

cadhr-lang は **Prolog 構文の上に CAD ドメイン型を載せた DSL**。表層は今と同じ
(`atom`, `var`, `struct`, `list`, `:-` rule, `?- query`) だが、AST / 評価器 / ビルトイン
が型を理解する。

- **値の型**: `Number` (有理数), `Atom`, `String`, `Bool`,
  `List(T)`, `Record(R)`,
  `Shape2D`, `Shape3D`, `PlacedShape2D`, `Path2D`,
  `Point2D`, `Point3D`, `Plane`
- **関数 (= rule)**: 引数の型 (range で型情報を兼ねる場合あり) と戻り値の型を持つ
- **型推論**: シグネチャは型必須。関数本体内のみ推論

## 4. 構文

### 4.1 字句

```
atom        ::= [a-z][A-Za-z0-9_]*
              | "'" any_except_quote "'"
ident_var   ::= [A-Z_][A-Za-z0-9_]*
qualified   ::= atom ("::" atom)+
number      ::= [-]?[0-9]+("."[0-9]+)?
string      ::= "\"" any_except_quote "\""
comment     ::= "%" ... newline | "/*" ... "*/"
```

字句は現状を踏襲。

### 4.2 トップレベル (Clause)

```ebnf
program ::= clause*

clause ::= use_directive
         | record_decl
         | function_def

use_directive ::= "#use" "(" string ("," "expose" "(" "[" name_list "]" ")")? ")" "."
record_decl   ::= "#record" atom "{" field_list "}" "."
function_def  ::= head ":-" body "."
                | head "."                   // body 空 (= fact)
```

### 4.3 関数のシグネチャと range

```ebnf
head      ::= atom "(" param_list? ")" return_type
param     ::= range_var | type_var | pattern
range_var ::= number cmp ident_var (cmp number)?    // 0<X  or  0<=X<=10
            | ident_var cmp number                    // X<10  or  X>=0
cmp       ::= "<" | "<=" | ">" | ">="
type_var  ::= ident_var ":" type_expr                 // 例: Profile: Shape2D
return_type ::= "->" type_expr                        // 必須
```

例:

```prolog
% range が型 (Number) を兼ねる
m5_bolt(6<=Length<=80) -> Shape3D :-
    Shape3D = ... .

% 範囲が片側だけ
gear(Module>0, 8<=Teeth) -> Shape3D :-
    ...

% range 無し: 型は推論
my_box(Size) -> Shape3D :-
    Size = cube(Size, Size, Size).
```

ルール:
- range annotation は **head の引数位置に inline**。body 内 range は不要 (`==` で
  制約として書くか、シグネチャに上げる)
- range があれば `Number` 型と確定 (range が型注釈を兼ねる)
- range の無い引数は **`X: Type` で型を必ず明示**
- 戻り値型 `-> Type` は **必須**
- `X@25` の default annotation は **構文から削除**

例 (型必須を満たさない / 満たす):

```prolog
% NG: Size の型不明、戻り値型もない
my_box(Size) :- ... .

% OK: range が型を兼ねる + 戻り値型明示
my_box(0<Size<=100) -> Shape3D :- ... .

% OK: 明示的な型注釈
build(Profile: Shape2D, H: Number) -> Shape3D :- ... .
```

### 4.4 record

```ebnf
record_decl ::= "#record" atom "{" field ("," field)* ","? "}" "."
field       ::= atom (":" type_expr)? ("=" default_expr)?
```

```prolog
#record output {
    models:   List(Shape3D) = empty_3d_list,
    bom:      List(BomEntry) = [],
    controls: List(Control) = [],
}.
```

`#record` directive と Prolog の `:-` directive (load 時クエリ実行) を混在させないため、
全てのメタ宣言を `#` プレフィックスに統一する (`#use` と一貫)。

- 型注釈 (`: List(Shape3D)`) は省略可。書けば検査される、書かなければ default 値から推論
- default は型に応じた zero/unit 値が選べる。`empty_3d_list`, `empty_2d`, `0`, `[]` 等
- フィールドアクセスは `record_expr.field_name` (静的 syntax)
- record 構築は 2 通り提供:
  - **positional**: `output(Models, Bom, Controls)` (今と同じ。型順)
  - **named**: `output { models: Ms, bom: Bs, controls: Cs }` (新)
- `make_NAME` / `NAME_FIELD` / `set_FIELD_of_NAME` の **auto-generation は廃止**

### 4.5 式と制約

これは現状を踏襲

- `+ - * /` は算術 (`InfixExpr`)、Shape の合成は関数 (`union(a,b)` 等)
- `=` は構造的単一化 + 線形等式制約 (現状の `Eq` の二重 dispatch を維持)
- `|>` パイプは現状の挙動を維持
- list, qualified atom, body の goal 列も現状通り

## 5. 型システム

### 5.1 基本型

```
Type ::= Number
       | Atom
       | String
       | Bool
       | List(Type)
       | Record(RecordName)
       | Shape2D
       | Shape3D
       | PlacedShape2D
       | Path2D
       | Point2D | Point3D
       | Plane
       | TypeVar(α)             // 推論用
```

- `Shape2D` の中身は今の `Model2D` enum (Sketch / Circle / Union / Diff / Inter /
  Center2D + 新規 `Empty`) を維持
- `Path2D` は今の `Path` を維持 (+ 新規 `Empty`)
- `Shape3D` は manifold-rs の Manifold + lazy な記号木 (+ 新規 `Empty`)
- `PlacedShape2D` は `Shape2D` を `Plane` に貼り付けたもの。extrude 系の入力
- `Bbox` 等の派生型は `std` で `#record` 宣言される (= `Record(Bbox)`)

### 5.2 推論

- **関数間境界 (シグネチャ) は型必須**。引数型と戻り値型はコードに書く
- **関数本体内のみ推論**: body 内の中間変数 (`Shape = cube(...)`) は型注釈不要、
  右辺と束縛から推論
- range annotation (`6<=X<=80`) は `X: Number` の代替として機能
- 推論結果とシグネチャが食い違ったらエラー

利点:
- LSP の hover が関数を見ただけで型を返せる (本体を辿らない)
- ビルトインとユーザー関数で hover/completion の挙動が均質
- 推論器は関数内に閉じるので実装が単純 (相互再帰の不動点計算が不要)

推論失敗 (本体内で型が決まらない) はエラー。**fallback で `Any` には落とさない**
(silent な型崩しを禁止)。

### 5.3 多相 (rank-1 polymorphism)

`List(T)` などの多相型を扱うため **シグネチャに型変数を明示する rank-1 多相** を採用。
フル HM (Algorithm W) ではなく、Rust の generics と同じモデル。

```prolog
% T はシグネチャに初出する大文字始まり identifier → 自動で型変数として量化
length(Xs: List(T)) -> Number :- ... .

% 呼び出し側で T = Shape3D に instantiate される
N :- N = length([cube(1,1,1), sphere(2)]).

% 複数の型変数も同様
zip(Xs: List(A), Ys: List(B)) -> List(Pair(A, B)) :- ... .
```

実装に必要な機構:

| 機構 | 必要 | 備考 |
|------|------|------|
| Unification (Robinson) | ✓ | `List(T)` と `List(Shape3D)` を unify |
| Type variable 生成 | ✓ | シグネチャの T ごとに fresh α |
| Instantiation | ✓ | 関数呼び出し点ごとに新しい α |
| Occurs check | ✓ | 無限型の防止 |
| Generalization | ✗ | シグネチャに明示済み |
| Let polymorphism | ✗ | 本体内中間変数は monomorphic |

generalization と let polymorphism を切ったぶん、HM 標準実装より大幅に軽い。

### 5.4 多相値の context 推論

`[]` や `empty_3d` のような多相値は周辺の context から型が決まる:

```prolog
% [] は List(T) の T が context から確定
main() -> output {
    models: [cube(10,10,10)],   % T = Shape3D (要素から確定)
    bom: [],                     % T = BomEntry (record 宣言から確定)
    controls: [],                % T = Control (record 宣言から確定)
}.
```

context が無い裸の `[]` は推論不能 → エラー。逃げ道は型注釈付きリテラル:

```prolog
empty: List(Shape3D) = []: List(Shape3D).
```

または `std` で alias 定数を提供 (`empty_3d_list` 等)。`Any` への silent fallback は
しない。

### 5.5 型固有の zero/unit 値

`std` で組み込み定数として提供:

| 型 | empty 値 |
|----|---------|
| `Shape3D` | `empty_3d`  (Manifold::Empty) |
| `Shape2D` | `empty_2d`  (Model2D::Empty) |
| `Path2D` | `empty_path` |
| `List(T)` | `[]` (型ごとの empty list、推論で T が決まる) |
| `List(Shape3D)` | `empty_3d_list` (= `[]: List(Shape3D)`) — record default に使うエイリアス |

`Model2D` / `Model3D` enum に `Empty` バリアントを追加し、CSG 演算 (`union` 等) が
empty を identity element として扱うようにする。これで record default が `[]` 一辺倒
にならず、`empty_3d` のような型整合した値を書ける。

## 6. ビルトインの単一登録

新しい `cadhr-lang/src/builtins.rs` に集約:

```rust
pub fn registry() -> BuiltinRegistry {
    BuiltinRegistry::new()
        .add(Builtin {
            name: "cube",
            sig: fn_sig!(x: Number, y: Number, z: Number) -> Shape3D,
            doc: "axis-aligned box centered at origin",
            eval: |args| Shape3D::cube(args[0].as_num()?, args[1].as_num()?, args[2].as_num()?),
        })
        .add(Builtin {
            name: "linear_extrude",
            sig: fn_sig!(profile: Shape2D_placed, height: Number) -> Shape3D,
            doc: "...",
            eval: |args| ...,
        })
        // ...
}
```

- 名前・型・実装・docs が 1 entry
- LSP は `registry()` を読むだけで completion / hover / signature help を提供
- `BUILTIN_FUNCTORS` 配列、`FunctorTag` enum、`FromStr`/`Display`、`Manifold` 構築の
  match arm、LSP 個別ファイルへの編集は全部不要に

`inventory` クレートでの分散登録は使わない。`registry()` 1 関数で完結。

### 6.1 提供するビルトイン (初期セット)

overload は廃止。次元ごとに別名を付ける (`union3d` / `union2d` 等)。CSG 以外も同様
(`diff` / `intersect` / `hull` / `place` 系)。

**Primitives**
- `Shape3D`: `cube`, `sphere`, `cylinder`, `tetrahedron`, `empty_3d`
- `Shape2D`: `circle`, `sketch`, `polygon`, `empty_2d`
- `Path2D`: `path`, `line_to`, `bezier_to`, `empty_path`

**Loader (runtime I/O)**
- `stl(path: String) -> Shape3D` — ファイル読み込みは **runtime**。compile 時には
  path の形式 (String 型) のみ検査。実行時に load 失敗したら `Diagnostic` で表面化

**CSG (3D)**
- `union3d(Shape3D, Shape3D) -> Shape3D`
- `diff3d(Shape3D, Shape3D) -> Shape3D`
- `intersect3d(Shape3D, Shape3D) -> Shape3D`
- `hull3d(Shape3D, Shape3D) -> Shape3D`

**CSG (2D)**
- `union2d(Shape2D, Shape2D) -> Shape2D`
- `diff2d(Shape2D, Shape2D) -> Shape2D`
- `intersect2d(Shape2D, Shape2D) -> Shape2D`
- `hull2d(Shape2D, Shape2D) -> Shape2D`

**Transform (3D)**
- `translate3d(Shape3D, Point3D, Point3D) -> Shape3D` (src 点を dst 点に運ぶ)
- `scale3d(Shape3D, Point3D) -> Shape3D` (Point3D を倍率ベクタとして使う)
- `rotate3d(Shape3D, Point3D) -> Shape3D` (Point3D を回転角ベクタとして使う)

**Place (2D → 3D)**
- `place(Shape2D, Plane) -> PlacedShape2D` (現 `rotateToXY/YZ/XZ` を統合)

**Extrude**
- `linear_extrude(PlacedShape2D, Number) -> Shape3D`
- `complex_extrude(PlacedShape2D, Number, Number, Number, Number) -> Shape3D`
- `revolve(PlacedShape2D, Number) -> Shape3D`
- `sweep_extrude(PlacedShape2D, Path2D) -> Shape3D`

**Geometry helpers**
- `bbox(Shape3D) -> Bbox`, `center_of(Bbox) -> Point3D`
- `p(Number, Number) -> Point2D`, `p(Number, Number, Number) -> Point3D`

廃止:
- `make_NAME`, `NAME_FIELD`, `set_FIELD_of_NAME` (record literal で置換)
- `rotateToXY/YZ/XZ` (`place/2` に統合)
- 同名 overload (`union/2` 等) — 次元ごとに別名へ

## 7. main の規約

```prolog
#record output {
    models:   List(Shape3D) = empty_3d_list,
    bom:      List(BomEntry) = [],
    controls: List(Control) = [],
}.

main(6<=Length<=80, 1<=Holes<=5) -> output { ... } :- ...
```

- `main` は `output` 型を返す関数
- 引数 (range 付き) は GUI のスライダー仕様の **メタデータ源**
- 引数の現在値は `previews.json` に保存される
- interpreter は output 型から `.models` / `.bom` / `.controls` を **named field で**
  取り出す。flat list の走査 + 文字列マッチによる振り分けは廃止

## 8. previews.json の責務

```json
{
  "preview_id": "main_v1",
  "values": { "Length": 25.0, "Holes": 3 },
  "control_overrides": { "cp_0": [0.0, 0.0, 5.0] }
}
```

- `values`: main の各引数の **現在値のみ**。range や型は db を読めば分かる
- `control_overrides`: 既存仕様通り (preview window 上で drag した制御点)
- スライダーの range・並び順・default 値は **コード側で決まる** ので json には入れない

## 9. モジュール

- `#use("rel/path")` は `rel/path/db.cadhr` を読む (現状通り)
- `expose([name, ...])` には **型名 (record)** と **関数名** の両方を書ける
- ビルトインはモジュール prefix を受けない (現状通り)
- record 型の再宣言はエラー (現状の「field 一致なら idempotent」を廃止 — 静的型検査
  との相性が悪い)

### 9.1 std が提供するもの

`std` は CARGO_MANIFEST_DIR 依存をやめ `include_str!` で bundle する。提供物:

- **record 宣言**: `output`, `BomEntry`, `Control`, `Bbox`
- **多相 list helpers** (必要なら): `length`, `map`, `filter`, `zip` 等
- `empty_3d_list` 等の名前付き alias 定数 (record の default に書きやすくするため)

ビルトインの `empty_3d` / `empty_2d` / `empty_path` 本体は std ではなく registry 側
(`builtins.rs`) に居る。std は型注釈やエイリアスを足すだけ。

## 10. エラー・診断

- `Diagnostic { severity, span, code, message, related: [...] }` を中間表現に
- 型エラーは「期待した型」「実際の型」「式の span」を含める
- LSP は `Diagnostic` をそのまま `lsp_types::Diagnostic` に変換

## 11. インタープリタとのインターフェース

```rust
pub struct CompiledProgram { /* AST + type info + resolved modules */ }

pub fn compile(
    main_src: &str,
    include_paths: &[Path],
) -> Result<CompiledProgram, Vec<Diagnostic>>;

pub struct MainSignature {
    pub args: Vec<MainArg>,
    pub return_type: Type,
}
pub struct MainArg {
    pub name: String,
    pub ty: Type,
    pub range: Option<Range>,
}

pub fn main_signature(program: &CompiledProgram) -> &MainSignature;

pub struct Inputs {
    pub values: HashMap<String, Rational>,
    pub control_overrides: HashMap<String, Point3D>,
}

pub fn run_main(
    program: &CompiledProgram,
    inputs: &Inputs,
) -> Result<MainOutput, Diagnostic>;

pub struct MainOutput {
    pub models: Vec<Shape3D>,
    pub bom: Vec<BomEntry>,
    pub controls: Vec<Control>,
}
```

- GUI は `compile` → `main_signature()` から slider 仕様を構築
- `previews.json` の値は `Inputs.values` に流し込む
- `run_main` の返り値は named field でアクセス。flat list の振り分けは消滅

## 12. 移行戦略

スコープを **「型・型推論・ビルトイン集約・record ネイティブ化」** に絞る。Prolog 機構
(unify, `=`, backtracking, `|>`, SrcSpan) は現状を維持。これで diff は半分以下になる
はず。

段階:

1. **AST に型注釈フィールドを追加** (`Option<Type>`)。既存の `Term` を拡張するか、新 AST に
2. **`Type` enum と推論器を実装**。最初は warn-only でも動く
3. **`empty_3d` / `empty_2d` / `Model2D::Empty` / `Model3D::Empty` を追加**
4. **`BuiltinRegistry` を新設**, 既存の 6 箇所を 1 entry/関数 に集約。`inventory` 廃止
5. **record の auto-generate を停止**, 代わりにネイティブ field access (`r.field`) と
   record literal (`R { f: v }`) を AST/評価器に追加
6. **`X@25` default annotation を構文から削除**, previews.json に move
7. **LSP を registry ベースに書き換え**
8. **GUI を `MainSignature` 経由に書き換え**, `unpack_main_output` の flat 走査を named
   field access に置換

各段階で `examples/*.cadhr` の動作確認。テストケース変更はユーザー確認の上。

## 13. 決定事項

- **`empty_3d` の正式名**: `empty_3d` / `nothing_3d` / `void_3d` のどれが好みか => `empty_3d`
- **named record syntax の括弧**: `output { ... }` (Rust 風) vs `output(. ...)` (Prolog風) => レコードでのみ `{}` を使うというのはわかりやすいため `{}`
- **型注釈の syntax**: `X: Shape3D` (Rust/TS 風) vs `Shape3D :: X` (Haskell 風) vs `Shape3D X` (位置) => `X: Shape3D`
- **stl の I/O**: `stl(path)` は compile 時に load するか、runtime か => runtime
- **Shape2D/3D の同名 union**: `union/2` を overload するか、それぞれ別名にするか => `union2d`/`union3d` のように別名。類似のものもそれぞれ同様 (`diff` / `intersect` / `hull` / `translate` / `scale` / `rotate`)
