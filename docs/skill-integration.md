# jlc-hardware-project の Skill への組み込み (計画)

kicad-audio-router を公開したあとで、jlc-hardware-project の Skill に組み込むための計画です。公開するまでは Skill を変えません。

## 今の Skill の配線

- 雛形の `assets/template/tools/gen_pcb.py` が配置と配線を 1 本のスクリプトで行う。配線は簡単な格子の A* で、F.Cu と B.Cu を使い、内層 2 枚は GND のベタにする。
- `references/kicad-generation.md` の「基板」の節に、その手順が書いてある。手順は、`--check` でコートヤードの重なりを確かめてから配線すること、`NARROW` と `ORDER_FIRST` を設定すること、SMD の GND パッドをビアで内層へ落とすことだ。
- 配線の品質は「直角が多いが実用上は問題ない」とだけ書いてあり、数値で確かめる手順は無い。

## 変えること

1. `gen_pcb.py` に配線をしない `--place-only` を足し、配置までにする。配線は `kar auto` に任せる。
2. `references/kicad-generation.md` の「基板」の手順を、次の順に書き直す。
   1. `gen_pcb.py --place-only` で配置し、`--check` でコートヤードの重なりを確かめる
   2. `kar classify` でネットの分類を確かめ、合わないものはプロジェクトの `kar_rules.json` で直す
   3. `kar auto` で配線、GND 処理、DRC、品質レポートをまとめて行う
   4. `kar plot` の図と KiCad の 3D 画像を目で見る
3. 品質レポート (`*_quality.md`) を設計の根拠と製作者向け資料に載せる。`design_notes.py` と `handover.py` から読み込む。
4. SKILL.md の「完了の目安」の基板の項に、次を足す。
   - `kar auto` の DRC で、未接続 0、kar が原因の問題 0
   - 品質レポートで、アナログとデジタルの 1mm 以内の並走が 0 に近いこと、差動ペアが並んでいること、GND パッドがビアから 3mm 以内にあること
5. 今の A* は、kar が使えない環境のための予備として残す (`ROUTER=builtin`)。

## 入れ方

公開後は kar を KiCad に付いている Python に入れる。

```sh
$KPY -m pip install git+https://github.com/<公開先>/kicad-audio-router
```

pip が使えない環境では、リポジトリを置いて `PYTHONPATH` で指す。Skill の `setup/check.py` に、kar が読み込めるかの確認を足す。

## 先に確かめること

- Skill の見本 (P48 ファンタム電源) を `kar auto` で配線し、DRC と品質レポートを今の A* の結果と比べる。
- KiCad 9 でも動くか (今の確認は KiCad 10 だけ)。Windows で動くか。
