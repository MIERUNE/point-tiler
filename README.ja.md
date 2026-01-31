# Point Tiler

点群データ（LAS/LAZ/CSV）を 3D Tiles v1.1 に変換するツールです。

[English](./README.md)|**日本語**

![alt text](images/README_image.png)

## Point Tiler？

Point Tiler は、既存のオープンソース点群タイラーと比較して、以下の点で優れています。

- **3D Tiles 1.1 のネイティブ出力** — GLB（glTF Binary）ファイルを直接出力し、3D Tiles 1.1 に完全対応しています。多くの代替ツールはレガシーな `.pnts` 形式（3D Tiles 1.0）を使用しているか、1.1 サポートは実験的な段階にとどまっています。
- **LAZ 対応** — 圧縮された LAZ ファイルを並列デコードで読み込みます。実際のワークフローでは LAZ が主流のフォーマットであるため、これは不可欠な機能です。
- **外部ソートによる大規模データ対応** — 設定されたメモリ上限に応じて、インメモリ処理と外部ソート処理を自動的に切り替えます。RAM に収まるようにファイルを手動で分割する必要がありません。
- **高速な変換処理** — Rust で実装され、Rayon による並列処理を活用し、大規模データセットに対して高いスループットを実現します。

## 機能

- LAS/LAZ/CSV 点群データを 3D Tiles v1.1 形式に変換
- PROJ ライブラリ内蔵で幅広い座標系をサポート
- Rayon による高性能な並列処理
- 大規模データ向けストリーミング処理
- ボクセルベースの点群間引きによる効率的な LOD 生成
- 出力タイルの GZIP 圧縮サポート
- より小さな GLB ファイルのための量子化サポート

## プロジェクト構成

```
point-tiler/
├── app/                       # メイン CLI アプリケーション
├── pcd-core/                  # 点群データのコアデータ構造
├── pcd-parser/                # LAS/LAZ/CSV ファイルパーサー
├── pcd-exporter/              # タイリングと 3D Tiles エクスポート
├── coordinate-transformer/    # 座標変換（PROJ ベース）
└── cesiumtiles-gltf/          # 3D Tiles 出力用 glTF/GLB I/O
```

## インストール

以下のコマンドを実行してください。

```sh
curl -sSf https://raw.githubusercontent.com/MIERUNE/point-tiler/main/install.sh | bash
```

現在、Windows では利用できません。
ただし、Rust がインストールされていて Cargo が使用可能であれば、`cargo` コマンドで実行できます。

### 開発者向け

Rust のインストールが必要です。以下のページから簡単にインストールできます。

[Getting started](https://www.rust-lang.org/learn/get-started)

Rust をインストールした後、このリポジトリをダウンロードしてください。

## 使用方法

### オプション

| オプション        | 説明                                                                                 |
| ----------------- | ------------------------------------------------------------------------------------ |
| `--input`, `-i`   | 入力ファイルパス。`.las`、`.laz`、`.csv`、`.txt` に対応。複数ファイル指定可能。      |
| `--output`, `-o`  | 出力フォルダパス。`tileset.json` と GLB ファイルを出力。                             |
| `--input-epsg`    | 入力座標系の EPSG コード。PROJ を介して任意の EPSG コードに対応。                    |
| `--output-epsg`   | 出力座標系の EPSG コード。Cesium 向けには通常 EPSG:4979（WGS84 地理座標 3D）を使用。 |
| `--min`           | 最小ズームレベル（デフォルト: 15）                                                   |
| `--max`           | 最大ズームレベル（デフォルト: 18）                                                   |
| `--max-memory-mb` | ワークフロー選択のためのメモリ上限（MB 単位、デフォルト: 4096）                      |
| `--threads`       | 並列処理のスレッド数（デフォルト: CPU コア数）                                       |
| `--quantize`      | より小さな GLB ファイルのための量子化を有効化                                        |
| `--gzip-compress` | 出力タイルの GZIP 圧縮を有効化                                                       |

### 使用例

```sh
point_tiler --input app/examples/data/sample.las \
    --output app/examples/data/output \
    --input-epsg 6677 \
    --output-epsg 4979 \
    --min 15 \
    --max 18 \
    --max-memory-mb 8192 \
    --threads 8 \
    --quantize \
    --gzip-compress
```

### ベンチマーク

| 実行環境 | |
| --- | --- |
| マシン | Apple M1 Max / 64 GB RAM |
| OS | macOS (Darwin 23.4.0) |

**実行コマンド:**

```sh
point_tiler --input /path/to/data/*.las \
    --output /path/to/output \
    --input-epsg 6677 \
    --output-epsg 4979 \
    --min 15 \
    --max 18 \
    --max-memory-mb 8192 \
    --threads 8 \
    --quantize \
    --gzip-compress
```

| 入力データ | |
| --- | --- |
| ファイル数 | 41（LAS 形式） |
| 合計サイズ | 約 8.0 GB |

| 結果 | |
| --- | --- |
| 合計時間 | **4分59秒**（299.1秒） |
| 出力タイル数 | 278 タイル |

### 座標系

このツールは座標変換に PROJ ライブラリを使用しており、任意の EPSG コードに対応しています。

**一般的な入力座標系：**
- 日本の平面直角座標系（EPSG:6669-6687）
- UTM ゾーン（例: UTM54N の場合 EPSG:32654）
- その他の投影座標系または地理座標系

**推奨される出力座標系：**
- EPSG:4979（WGS84 地理座標 3D）- Cesium の標準
- EPSG:6697（JGD2011 地理座標 3D）- 日本固有のアプリケーション向け

### ズームレベル

このツールは「ズームレベル」というユニークな概念を持っています。これはラスタータイルや Google Photorealistic 3D Tiles などの 2D タイルとほぼ同じサイズの平面領域を表します。
ジオメトリックエラーはズームレベルに応じて変化し、例えばズームレベル 15 では約 64.0、ズームレベル 18 では約 8.0、ズームレベル 21 では約 1.0 になります。
出力タイルは `ジオメトリックエラー × 0.1` のボクセルグリッドに 1 点が格納されるように間引かれます。

### CSV/TXT 形式

.csv と .txt のファイル拡張子に対応しています。

CSV 形式では、XYZRGB 以外のカラムは無視されます。
また、カラム名は "x"、"y"、"z" および "r"、"g"、"b" または "red"、"green"、"blue" である必要があります。
（大文字・小文字は区別しません。）

例えば、以下のようなデータが有効です。

```csv
"X","Y","Z","Intensity","ReturnNumber","NumberOfReturns","ScanDirectionFlag","EdgeOfFlightLine","Classification","Synthetic","KeyPoint","Withheld","Overlap","ScanAngleRank","UserData","PointSourceId","GpsTime","Red","Green","Blue"
-5599.971,-35106.097,3.602,680.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.207,19532.000,20046.000,20560.000
-5599.976,-35111.458,3.440,715.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.287,20560.000,21074.000,21588.000
-5599.978,-35115.695,3.543,311.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.347,19018.000,19275.000,20303.000
-5599.992,-35119.757,3.603,722.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.407,18504.000,20046.000,20817.000
-5599.992,-35129.327,3.431,505.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.547,18504.000,19789.000,21074.000
```

## ロードマップ

- [ ] 入力ファイルからの CRS 自動検出
- [ ] 点属性のエクスポート（classification、intensity など）
- [ ] Windows サポート
- [ ] 外部連携のためのライブラリ API 公開
- [ ] PLY 形式の入力対応

## ライセンス

[MIT License](./LICENSE)

## 著者

- Satoru Nishio (@nokonoko1203)
- 全コントリビューター

## サードパーティソースコードの使用

このプロジェクトは、以下のサードパーティプロジェクトのソースコードを組み込み、改変しています。

- [PLATEAU GIS Converter](https://github.com/MIERUNE/plateau-gis-converter)
  - リポジトリ: [PLATEAU GIS Converter](https://github.com/MIERUNE/plateau-gis-converter)
  - 元のライセンス: [MIT License](https://github.com/MIERUNE/plateau-gis-converter/blob/main/LICENSE)

### 行った変更

[PLATEAU GIS Converter](https://github.com/MIERUNE/plateau-gis-converter) のソースコードに対して以下の変更を行いました。

- [nusamai-gltf](https://github.com/MIERUNE/plateau-gis-converter/tree/main/nusamai-gltf) を [cesiumtiles-gltf](./cesiumtiles-gltf/) として組み込みました。

変更されたコードは元の著作権を保持しており、MIT License に従って使用されています。詳細は LICENSE ファイルをご参照ください。
