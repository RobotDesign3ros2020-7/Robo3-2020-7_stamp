[English](README.en.md) | [日本語](README.md)

### インストール方法
```sh
cd ~/catkin_ws/src/
git clone 
cd ..
catkin_make
rosdep install -r -y --from-path --ignore-src crane_x7_ros
```

### 実機を使う場合

実機で動作を確認する場合、
制御信号ケーブルを接続した状態で次のコマンドを実行します。

```sh
sudo chmod 777 /dev/ttyUSB0
roslaunch crane_x7_control crane_x7_control.launch
```

### Gazeboを使う場合

次のコマンドで起動します。実機との接続やcrane_x7_bringupの実行は必要ありません。

```sh
roslaunch crane_x7_gazebo crane_x7_with_table.launch
```

### 実行方法
以下のコマンドで実行します。
```sh
rosrun crane_x7_example rats.py 
```
