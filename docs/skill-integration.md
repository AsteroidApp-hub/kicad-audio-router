# jlc-hardware-project の Skill での使い方

[jlc-hardware-project](https://github.com/AsteroidApp-hub/jlc-hardware-project) の Skill は、基板の配線に kar を使います。
この文書は Skill のどこで kar を呼び、何を確かめているかをまとめたものです。

## 配線の流れ

- 雛形の `tools/gen_pcb.py` は配置、シルク、内層の GND ベタまでを作ります。内層は層の種類を「電源」にするので、kar はそこに配線しません。
- そのあと `kar auto` を呼び、配線、GND パッドごとのビア、KiCad の DRC、品質レポートをまとめて行います。
  途中のファイルとレポートは `kicad/.route/kar_out/` に出ます。
- プロジェクト直下に `kar_rules.json` があれば、ネットの分類の上書きとして渡します。
- 未配線か、配線による DRC の問題が残ると、`gen_pcb.py` はエラーで止まります。配置で決まる指摘 (許したコートヤードの重なりなど) では止まりません。
- kar が入っていないときは、入れ方を出して止まります。`ROUTER=builtin` を付けると、Skill に元からある簡易の格子ルーターで配線します。

## 資料

- `tools/design_notes.py` は、品質レポートの要点を設計の根拠の資料に「基板の配線」の節として載せます。
  レポートの全文と配線図 (`配線図.svg`) も `設計根拠/` に写すので、製作者に渡す zip に入ります。
- Skill の「完了の目安」には次を入れています。
  - アナログの線がデジタルの銅箔の 1mm 以内を走る長さが 0 に近いこと
  - 差動ペアが並んでいること (ピンの位置で決まる部分を除く)
  - GND のベタがあるネットの GND パッドが、すべてビアから 3mm 以内にあること
  - 電源の線が細くなっていないこと

## 入れ方

Skill のインストールスクリプト (`setup/install_mac.sh`、`setup/install_windows.ps1`) が、kar を KiCad 同梱の Python に入れます。
`setup/check.py` は、kar がその Python から読み込めるかを確かめます。手で入れるときは次のとおりです。

```sh
$KPY -m pip install git+https://github.com/AsteroidApp-hub/kicad-audio-router
```

KiCad を入れ直したときや更新したときは、kar も入れ直します。

## 雛形での比較

Skill の見本 (2ch P48 ファンタム電源) を、元の簡易ルーターと kar で配線して比べた結果です (KiCad 10、macOS)。

| | 簡易ルーター | kar |
|---|---|---|
| 品質レポートの点数 | 82.2 | 83.6 |
| アナログの線の長さ | 404 mm | 350 mm |
| アナログのビア | 2 | 0 |
| ビアから 3mm より遠い GND パッド | 3 (すべて CHASSIS) | 2 (すべて CHASSIS) |
| 未配線、配線による DRC の問題 | 0 | 0 |

CHASSIS のパッドが残るのは、CHASSIS にはベタが無く、ビアを打っても落とす先が無いためです。
