![GitHub Release](https://img.shields.io/github/v/release/Task-Intelligent-Robotics-Research-Grp/aist_barrett)
![GitHub License](https://img.shields.io/github/license/Task-Intelligent-Robotics-Research-Grp/aist_barrett)

| ROS 2 Distribution | Jazzy                                                                                                                                                                    |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Build Status       | [![jazzy-build](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_barrett/actions/workflows/jazzy-build.yaml/badge.svg)](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_barrett/actions/workflows/jazzy-build.yaml) |

aist_barrett
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
 以上により，本ソフトウェアに含まれる`aist_barrett`と`aist_barrett_msgs`がインストールされます．使い方については，それぞれ以下のリンクを参照してください．
- [aist_barrett](https://task-intelligent-robotics-research-grp.github.io/aist_barrett/md_aist__barrett_2README.html): Usage of controllers and drivers for `Barrett` grippers. API document of the controller clients is also included.
- [aist_barrett_msgs](https://task-intelligent-robotics-research-grp.github.io/aist_barrett/aist_barrett_msgs/index.html): Definitions of ROS message/service/action used by controllers and drivers included in `aist_barrett`.
 