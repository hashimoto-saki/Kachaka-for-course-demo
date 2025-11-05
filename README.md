コースデモ2025　実行手順

参考：https://demura.net/robot/athome/23407.html

kachakaのwifiの設定

必ずポケットwifiと接続する

携帯も同じwifiに繋げる

IPアドレスを確認

地図を手動で作成（やらなくてOK）

関係する範囲だけOK

使用する地点を登録

スライド参照

kachakaとの通信

ターミナル①起動

cd ~/Kachaka_hashimoto_2024/kachaka-api/tools/ros2_bridge

./start_bridge.sh IPアドレス

例：　./start_bridge.sh 10.40.20.164

ros2_bridge_1が起動できていることを確認

地点のID確認（やらなくてOK）

cd ~/Kachaka_hashimoto_2024/ros2_ws/

cd ~/ros2_ws

source install/setup.bash

ros2 topic echo /kachaka/layout/locations/list

プロンプト内のIDとの参照を確認（やらなくてOK）

~/Kachaka_for_python/ros2_ws/src/kachaka_python/course_demo_2025/task_planning.txt

コードの実行

ターミナル②起動

cd ~/Kachaka_for_python/ros2_ws/src/kachaka_python/course_demo_2025

python3 course_demo.py

実行の流れ

５秒待つ

kachakaの発話後、PCのマイクに指示を出す

ターミナルで発話結果を確認

ロボットが行動
