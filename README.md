# robosyshw2
ロボットシステム学課題２


[![test](https://github.com/tsukada8810/robosyshw2/actions/workflows/test.yml/badge.svg)](https://github.com/tsukada8810/robosyshw2/actions/workflows/test.yml)

画面上のマウス座標を取得するノードを提供するROS 2パッケージ

## 実行環境
- Ubuntu 24.04 LTS
- ROS 2 Jazzy

## ノードとトピックの構成
#### position (送信ノード)
- PC画面全体に対するマウスカーソルの座標を取得し、トピック`/mouse_pos`に対して`geometry_msgs/msg/Point`型でパブリッシュする
#### distance (受信ノード)
- トピックをサブスクライブする
- 送信された座標をログに出力する

## 実行方法
- ノード１：座標送信ノード
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run robosyshw2 position
```
- ノード２：受信・デバック用ノード
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run robosyshw2 distance
```

## 実行結果(例)
- ノード２で起動したあとマウスを動かすと、ノード１に以下のようなログが出力される
```Plaintext
[INFO] [1735626883.371470528] [distance_subscriber]: Position: X=100, Y=200
[INFO] [1735626884.047770986] [distance_subscriber]: Position: X=150, Y=100
...
```
## 動作確認環境について
### Ubuntu
- 実行後画面全体からマウスの座標を取得できる

### Windows (WSL2)
- WSLなどの仮想環境などでは座標を取得できない可能性がある
- その場合には３つ目の端末で以下のコマンドで仮想ウィンドウを起動し、そのウィンドウ上でマウスを動かすことで座標を取得することができる
- ノード３：仮想ウィンドウ用ノード
```bash
$ rqt
```

## トラブルシューティング
### ビルドや実行がうまくいかない場合
Anaconda等の仮想環境が有効になっているとビルドや実行が失敗する場合がある  
そのときには、以下のコマンドで仮想環境を無効化してから再度ビルドや実行
```bash
$ conda deactivate
```

## 参考文献
- https://github.com/ryuichiueda/slides_marp/tree/master/robosys2025

## ライセンス
- このソフトウェアパッケージは、3条項BSDライセンスの下、再配布及び使用が許可されます。
- © 2025 Hayato Tsukada
