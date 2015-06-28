`玉川大学認知発達ロボティクス研究室 <http://www.tamagawa.ac.jp/engineering/ims/t_h_okada.html>`_ (RoboCup@Home 参加チーム) で開発されている移動マニピュレータ "SPUR" の ROS パッケージです．

メインのドキュメントは `こちら <./README.rst>`_ を参照下さい．本日本語ドキュメントはインストール方法のみ記載します．

.. contents:: Table of Contents
   :depth: 3

Install
===========

インストール要件
--------------------------------

下記がインストール済であると仮定します．

* `Ubuntu 14.04 Trusty Tahr <https://wiki.ubuntu.com/TrustyTahr/ReleaseNotes>`_ 64bit
* `ROS Indigo Igloo <http://wiki.ros.org/indigo>`_

apt でのインストール (推奨)
--------------------------------

*__(May 9, 2015) apt でのインストールは推奨に違いないのですが，今日現在, ソースインストールを行って下さい (omni 用 joystick ノードがまだ作業中のため．このチケット (https://github.com/tork-a/spur/issues/16) がクローズされたら，ソースインストールは不要になります)．__*

以下，簡便のために `ROS Indigo` と `SPUR` ROS パッケージ両方をインストールするコマンドを記載します．
`ROS` のインストール詳細は, `wiki <http://wiki.ros.org/indigo/Installation/Ubuntu>`_ をご覧下さい．

.. code-block::

 Ubuntu$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
 Ubuntu$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
 Ubuntu$ sudo apt-get update && sudo apt-get install ros-indigo-desktop-full ros-indigo-spur
 Ubuntu$ sudo rosdep init && rosdep update
 
 Ubuntu$ echo "### For ROS setting"
 Ubuntu$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
 Ubuntu$ source ~/.bashrc

ソースからインストール
------------------------

ソースインストールは開発目的でのみ推奨されます．
`~/catkin_ws/` ディレクトリにインストールすることを想定しています．

1. catkin workspace を設定し `SPUR` ROS パッケージをダウンロード.

.. code-block::

 $ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && catkin_init_workspace
 $ git clone https://github.com/tork-a/spur.git                             

1-1. joy stick を設定 (暫定).

全方位移動ロボット用の ROS の joystick ドライバはまだ開発が進んでいる最中なので，ソースでインストールします (April, 2015．`関連するチケット <https://github.com/ros-teleop/teleop_twist_joy/pull/6>`_).

.. code-block::

 $ git clone https://github.com/130s/teleop_twist_joy.git && cd teleop_twist_joy && git checkout add/omnidir

2. 依存するライブラリのインストール．コンパイル．

.. code-block::

 $ cd ~/catkin_ws                                                           
 $ rosdep install --from-paths src --ignore-src -r -y                       
 $ catkin_make install && source install/setup.bash                         

Usage
===========

タスクを作った都度，必ず実機で動作させる前にシミュレーションで確認するようにして下さい．

ロボットを起動 (simulation & 実機)
------------------------------------------------

.. code-block::

 $ roslaunch spur_gazebo spur_world.launch    # Simulation
 $ roslaunch spur_bringup minimal.launch      # 実機

テレオペレーション
------------------------------------------------

テレオペレーションjoystick (Elecom PS3 使用)
###############################################################

以下は `PS3-Elecom <http://www.amazon.co.jp/ELECOM-USB%E3%82%B2%E3%83%BC%E3%83%A0%E3%83%91%E3%83%83%E3%83%89-12%E3%83%9C%E3%82%BF%E3%83%B3-%E3%83%96%E3%83%A9%E3%83%83%E3%82%AF-JC-GMU3312SPBK/dp/B003UIRIHC>`_ で確認済みの手順です．

1. joystick をペアリング．

* `For bluetooth-based joystick <http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle>`_

2. joystick ROS ノード起動．

.. code-block::

 $ roslaunch spur_bringup joy_teleop.launch
 $ roslaunch spur_bringup joy_teleop.launch joy_port:=/dev/input/js1  # If joy is found on a different port

3. PS3-Elecom を使うためには, "Mode" ボタンを二回押して，アナログ入力を有効化する．動作を行う際はボタン 9 を押し下げ続ける．

キーボードを使ったテレオペレーション
###########################################

.. code-block::

 $ roslaunch spur_bringup kb_teleop.launch

コミュニティ
============

* `質問フォーラム (英語, お薦め) <http://answers.ros.org/>`_
* `質問フォーラム (日本語) <https://groups.google.com/forum/#!forum/ros-japan-users>`_
* `問題報告 / 機能強化リクエスト <https://github.com/tork-a/spur/issues>`_
