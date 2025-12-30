# robosyshw2
ロボットシステム学課題２


[![test](https://github.com/tsukada8810/robosyshw2/actions/workflows/test.yml/badge.svg)](https://github.com/tsukada8810/robosyshw2/actions/workflows/test.yml)

ROS2を用いて画面上のマウスの座標を出力するパッケージ

## 実行環境
- Ubuntu 24.04 LTS
- ROS 2 Jazzy

## ノードとトピックの構成
#### position (送信ノード)
- マウスの画面上の座標を取得し、トピック'/mouse_pos'に対して'geometry_msgs/msg/Point'型でパブリッシュする
- 実行時に左上を(0, 0)とする白いウィンドウが表示され、その中でマウスの位置を取得する
#### distance (受信ノード)
- トピックをサブスクライブする
- 送信された座標をログに出力する

## 実行方法
- ノード１：座標送信ノード
```bash
$ ros2 run robosyshw2 position
```
- ノード２：受信ノード
```bash
$ ros2 run robosyshw2 distance
```

## 実行結果(例)
- ノード２で起動した白いウィンドウ上でマウスを動かすと、ノード１に以下のようなログが出力される
```Plaintext
[INFO] [1735626883.371470528] [distance_subscriber]: Position: X=100, Y=200
[INFO] [1735626884.047770986] [distance_subscriber]: Position: X=150, Y=100
...
```

## 参考文献
- https://github.com/ryuichiueda/slides_marp/tree/master/robosys2025

## ライセンス
- このソフトウェアパッケージは、3条項BSDライセンスの下、再配布及び使用が許可されます。
- 🄫 2025 Hayato Tsukada
