# cadhr-lang CLI

## 3MF出力

```nu
cargo run -p cadhr-lang -- export ./project -o ./model.3mf
cargo run -p cadhr-lang -- export ./project/db.cadhr --binding part -o ./part.3mf
```

- 入力は `.cadhr` ファイル、または直下に `db.cadhr` があるプロジェクトディレクトリ。
- `--output` / `-o` は必須。既存の出力ファイルは上書きする。
- `--binding` / `-b` の既定値は `main`。引数付きbindingは実行検査と同じ既定値（sliderの中央値、未指定なら `0.0`）で評価する。
- `Shape3D`、`List Shape3D`、`models` フィールドを持つレコードを出力できる。複数モデルは座標を保持した別々のオブジェクトとして1つの3MFに格納し、単位はmm。
- 空のモデルは除外し、出力可能なモデルがなければエラー。BOMやコントロールポイントは出力しない。
- コンパイル・評価・3MF変換に失敗した場合、出力ファイルは作成・変更しない。

## ライブラリAPI

GUIとCLIは `cadhr_lang::export` の3MF変換を共有する。

- `models_to_threemf` は `Model3D` のリストと検索パスを受け取り、メッシュ生成から変換まで行う。
- `meshes_to_threemf` は `Mesh`（`f64` の頂点座標と `u64` の三角形インデックス列）を受け取る。GUIは表示済みメッシュをこの形式に変換して渡す。
- どちらも `Result<Vec<u8>, String>` を返す。保存先の選択とファイル書き込みは呼び出し側で行う。

## 実行検査

```nu
cargo run -p cadhr-lang -- check ./project
cargo run -p cadhr-lang -- check ./project --binding part
```
