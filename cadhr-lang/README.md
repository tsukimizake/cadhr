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

## 実行検査

```nu
cargo run -p cadhr-lang -- check ./project
cargo run -p cadhr-lang -- check ./project --binding part
```
