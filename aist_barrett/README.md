aist_barrett
==================================================

## 概要
本パッケージは，[Barrett Technology社](https://barrett.com/)の[BarrettHand](https://barrett.com/barretthand)を制御するROS2ドライバ・コントローラを提供します．

## Barrett Hand
本パッケージの制御対象である[BarrettHand](https://barrett.com/barretthand)の特徴は，以下のとおりです．
 - ハンドのコントローラボックスは，CAN-USBコンバータ[PCAN-USB](https://www.peak-system.com/products/hardware/external-pc-interfaces/pcan-usb/)を介してPCに接続される．`PCAN-USB`は，`Ubuntu`に含まれるsocket CANドライバで駆動できる．
 - 3本の指(`left`, `right`, `middle`)は，それぞれ駆動軸と受動軸を1つずつ持ち，受動軸は駆動軸に連動して回転する．
 - `left`と`right`の指は，掌に平行な水平面内で互いに逆方向に連動して回転する．これを`spread`軸と呼ぶ．
 - 以上より，独立に駆動できる軸数（自由度）は4である．
 - 各駆動軸は，トルクセンサを備える（計4つ）．
 - 各指および掌は，触覚センサアレイを備える（計4つ）．

ドライバは，位置/速度/トルクのいずれかのコマンドで4つの駆動軸を独立に制御します．また，各軸の現在位置とトルクを`/joint_states`トピックとして出力します．さらに，触覚センサアレイの観測値もトピックとして出力します．

## barrett_hand_controllerノード
### ノードの機能
`barrett_hand_controller`ノードは，ROSからアクションまたはトピックを介して与えられるハンド制御命令を受信して`BarrettHand`ハードウェアの駆動命令に変換するコントローラ機能と，変換された命令をハードウェアに送信するドライバ機能の両方の役割を果たします．

### ROSアクション
- **~/gripper_cmd**（[control_msgs/msg/GripperCommand](https://docs.ros2.org/latest/api/control_msgs/action/GripperCommand.html)型：

### 入力トピック
- **~/position_command**（[std_msgs/Float64MultiArray](https://docs.ros2.org/latest/api/std_msgs/msg/Float64MultiArray.html)型）：

### 出力トピック
- **/joint_states**（[sensor_msgs/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)型）：ハンドの各駆動軸の回転角とトルクをpublish
- **~/tactile_states** ([aist_barrett_msgs/TactileStateArray](../aist_barrett_msgs/msg/TactileStateArray.msg)型)：ハンドの各指および掌に貼付された触覚センサアレイの観測値をpublish

### ノードパラメータ
- **device_name** (type: string): ハンドの名前 (default: `bhand`)

## ノードの起動
### 注意
本ノードは[rclcppのコンポーネント](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-a-Composable-Node.html)として実装されており，コンポーネントコンテナにロードして使用します．このとき，[launchファイル](./launch/launch.py#L60)にあるように，必ずマルチスレッド対応のコンテナ(`component_container_mt`)にロードしてください．複数のコールバックグループを使用し，マルチスレッドで実行することを前提にしていますので，シングルスレッドコンテナ(`component_container`)にロードするとデッドロックに陥ってハングします．

### 設定ファイルの準備


### 起動方法
次のコマンドを投入して起動します．
```bash
ros2 launch aist_barrett launch.py [device_name:=<hand_name>] [param_file:=<param_file>] [container:=<container_name>] [external_container:=true]
```
- **device_name**: ハンドに与える名前 (default: `bhand`)
- **param_file**: ノードパラメータ設定ファイルへのパス (default: [default.yaml](./config/default.yaml))
- **container**: ノードのロード先となるコンポーネントコンテナの名前 (default: `dynamixel_workbench_container`)
- **external_container**: `true`ならば，`container`に指定した名前で別途起動していた既存のコンテナにロード．`false`ならば，`container`に指定した名前で新たにコンテナを起動し，それにロード (default: `false`)
