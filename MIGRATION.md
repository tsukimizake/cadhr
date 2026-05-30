# cadhr-proj migration status (Phase 10)

旧 cadhr-lang (Prolog) → 新 cadhr-lang (Elm-like) への移行可否。
代表プロジェクトはテストとして `cadhr-lang/tests/migration_*.cadhr` に置く。

## 移行済み (テストで動作確認済み)

| プロジェクト | 仕掛け |
|------------|--------|
| `counter_cable_clip` | cube + diff3d のみ。素直な変換 |
| `desk_foot_cover` | polygon + extrude_xy。slider あり |
| `bed-cable-clip` | polygon × 2 を別々に extrude → diff3d (旧版は 2D CSG だった) |
| `bed_drink_holder` | cube/cylinder/rotate3d/translate3d/diff3d (2D 不要) |
| `bolts` (library pattern) | `module Bolts exposing (..)` + 別 `.cadhr` からの `import` |
| `loki_home_key` | 3 平面 (XY/YZ/XZ) sketch を `extrude_xy/yz/xz` で押し出し→ intersect3d。さらに `import Bolts` で `Bolts.m3_hole` を呼び diff3d でボルト穴。record (`{ sketch = .., p0 = .. }`) を関数戻り値として扱い `.sketch` でフィールドアクセス |

## ブロック (要追加機能)

| プロジェクト | 必要な機能 |
|------------|-----------|
| `wabouchou` | `bezier_to` (制御点付き Bezier 辺) |
| `loki_home_key` | record `.field` の library 越し読み出し、複数平面別 sketch を `(R1 + R2 + R3) - HOLE` で組む 3D CSG (これは可能だが書き直しが多い) |
| `extrusion_2020` | 2D CSG (`R1 - R2 - R3 + R5 + D1 + D2`) のままだと難しい。3D CSG に書き換え可 |
| `battery_case` | 2D CSG + `center2d` + `control2d`/`control3d` (slider 経由の動的点指定) |
| `ive_reararck` | TBD (未調査) |
| `ive_rear_doghouse_mount` | TBD (未調査) |

## 既知の言語機能ギャップ

旧 cadhr (Prolog) で提供されていたが、新 cadhr-lang にまだ無い builtin / 構文:

- **2D CSG**: `union2d` / `diff2d` / `intersect2d`。今は extrude 後に 3D で CSG する回避策が必要
- **Bezier 曲線**: `bezier_to(ctrl, end)` 相当。polygon は直線辺のみ
- **`revolve`**: 2D profile を回転軸まわりに回して 3D 化
- **`complex_extrude`**: ねじり (twist) + 縮尺 (scaleXY) 付き extrude
- **`sweep_extrude`**: 任意 path に沿った押し出し
- **`center2d` / `center3d`**: BBox 中心を変数に束縛するヘルパ
- **`control2d` / `control3d`**: GUI ドラッグ用 control point (Phase 4 のスライダーは
  値だが点ではない)
- **`stl`**: STL ファイル読み込み (signature だけ登録、評価は未実装)
- **`hexhead` 系の組み立て便利関数**: ライブラリ側でユーザ定義は可能だが、ある程度は
  builtin として欲しいかも

## 移行時の典型置換パターン

| 旧 (Prolog) | 新 (Elm-like) |
|------------|---------------|
| `a + b` (3D union) | `union3d a b` |
| `a - b` (3D diff) | `diff3d a b` |
| `a * b` (3D intersect) | `intersect3d a b` |
| `cube(X, Y, Z)` | `cube X Y Z` |
| `translate(p3(0,0,0), p3(X,Y,Z), S)` | `S \|> translate3d (p3 0.0 0.0 0.0) (p3 X Y Z)` |
| `sketch(p, [line_to(...)...])` | `polygon [p, ...]` (現状は直線のみ) |
| `S \|> rotateToXY \|> linear_extrude(H)` | `extrude_xy S H` |
| `#use("std")` | `import Std exposing (..)` |
| `:- record output { ... }` | `type alias Output = { ... }` |
| `slider X@A<X<B` | `slider x = A.. B` |
| `output { models = [...] }` | `{ models = [...], bom = [], controls = [] }` |
| 逆方向制約 `Z1*T1 + Z2*T2 = 0` | forward 関数 `next_theta g1 g2 = ...` |
