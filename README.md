# NHK2024 2次ビデオ用リポジトリ
サークル内向けです。

## 必要なものをインストールする方法
```bash
git clone https://github.com/chibarobotstuidonhk/nhk24_ws.git -b 2nd_ws
. bash/workspace_install.sh
```
ビルドは自分でやってください。`. bash/build_src.bash`でもできます。

.vscodeも上がっちゃったんで、有効活用してください。
- Ctrl+Shift+Bでsrc下のビルドができます。
- Ctrl+Shift+Pでコマンドパレットを開き、Tasks: Run Taskを選択すると、
  - build/, log/, install/の削除
  - 今開いているファイルのコンパイル
ができます。

千葉大の学内ネットワークだと色々動きません。テザリングするとかしてください。

色々なツールが必要かもです。適宜入れてください。
