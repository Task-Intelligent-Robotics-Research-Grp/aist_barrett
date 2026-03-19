aist_barrett and aist_barrett_msgs
==================================================

## 概要
本ソフトウェアは，以下の２つのROS2パッケージを含みます．
 - **[aist_barrett](./aist_barrett/)**: [Barrett Technology社](https://barrett.com/)の[BarrettHand](https://barrett.com/barretthand)を制御するROS2ドライバ・コントローラ
 - **[aist_barrett_msgs](./aist_barrett_msgs/)**: `aist_barrett`で使用するROS2 message/service/actionの定義
 
  ## インストール
 本ソフトウェアは，[ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) distributionで動作確認しています．
 
 まず最初に，`Barrett Technology`社が提供する[libbarrett](https://git.barrett.com/software/libbarrett)の[修正版](https://github.com/Task-Intelligent-Robotics-Research-Grp/libbarrett)をインストールします．詳細はライブラリ付属の[README](https://github.com/Task-Intelligent-Robotics-Research-Grp/libbarrett/blob/devel-aist/README.md)を参照してください．
 
 次に，`github`から本ソフトウェアを入手し，ワークスペースに展開します．
 ```bash
 cd ~/ros2_ws/src
 git clone https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_barrett
 ```

 さらに，`rosdep`によって依存パッケージをインストールします．
 ```bash
 cd aist_barrett
 rosdep update
 rosdep install -iy --from-paths ./
 ```
 最後に，ワークスペース全体をコンパイルしてください．
 ```bash
 cd ~/ros2_ws
 colcon build
 ```
 以上により，本ソフトウェアに含まれる[aist_barrett](./aist_barrett/)と[aist_barrett_msgs](./aist_barrett_msgs/)がインストールされます．使い方については，それぞれの`README`を参照してください．
 