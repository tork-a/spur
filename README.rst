ROS package suite for `SPUR`, a mobile omni-directional base robot with extensible arm made at `Okada Lab, Tamagawa University (玉川大学) <http://www.tamagawa.ac.jp/engineering/ims/t_h_okada.html>`_ (a RoboCup@Home contender).

See the video on http://wiki.ros.org/spur.

日本語での `簡易インストール方法はこちら <./README_ja.rst>`_

.. contents:: Table of Contents
   :depth: 3

DEB build status
================

"apt-get"table binary (`DEB`) files are generated at `ROS build farm <http://wiki.ros.org/buildfarm/Gen1Buildfarm>`_ maintained by `OSRF <http://osrfoundation.org/>`_). The following is the status of the build jobs.

+--------------+-------------------------+----------------------------+-----------------------------+-----------------------+-------------------------------+-------------------------+
| ROS Distro   | Source deb              | Development Branch (travis)| Development branch (ros.org)| Release Branch        | binarydeb Precise AMD64       | Documentation (ros.org) |
+==============+=========================+============================+=============================+=======================+===============================+=========================+
| Indigo       | |buildstatus_sourcedeb| | |buildstatus_devel_travis| | |buildstatus_devel_ros.org| | |buildstatus_release| | |buildstatus_binarydeb_amd64| | |buildstatus_doc|       |
+--------------+-------------------------+----------------------------+-----------------------------+-----------------------+-------------------------------+-------------------------+

`Devel Test Status <http://wiki.ros.org/regression_tests#Development_Tests>`_
-------------------------------------------------------------------------------------

|job_devel-indigo-spur|

Install
===========

Installation Requirement
--------------------------------

Following is assumed to be already installed:

* `Ubuntu 14.04 Trusty Tahr <https://wiki.ubuntu.com/TrustyTahr/ReleaseNotes>`_ 64bit
* `ROS Indigo Igloo <http://wiki.ros.org/indigo>`_

Install via apt (RECOMMENDED)
--------------------------------

*__(May 9, 2015) This is indeed recommended, however, due to some ongoing work, installing from source is required. Once https://github.com/tork-a/spur/issues/16 gets resolved this limitation will be gone.__*

The following set of commands will install both `ROS Indigo` and `spur` ROS package together for your convenience.
For the complete instruction for installing `ROS`, see `its wiki <http://wiki.ros.org/indigo/Installation/Ubuntu>`_.

.. code-block::

 Ubuntu$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
 Ubuntu$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
 Ubuntu$ sudo apt-get update && sudo apt-get install ros-indigo-desktop-full ros-indigo-spur
 Ubuntu$ sudo rosdep init && rosdep update
 
 Ubuntu$ echo "### For ROS setting"
 Ubuntu$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
 Ubuntu$ source ~/.bashrc

Install via source
------------------------

Installing via source is only recommended for development purpose.
The directory `~/catkin_ws/` will be used as a source directory for this instruction.

1. Set up catkin workspace. Download `SPUR` ROS package.

.. code-block::

 $ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && catkin_init_workspace
 $ git clone https://github.com/tork-a/spur.git                             

1-1. Install joy stick (temporarilly required for joy stick users).

Joy stick operation for omni-directional robots in ROS is still under development. As of now (April, 2015), install the joy stick ROS driver from source. 
This step will become not required in the future (`related <https://github.com/ros-teleop/teleop_twist_joy/pull/6>`_).

.. code-block::

 $ git clone https://github.com/130s/teleop_twist_joy.git && cd teleop_twist_joy && git checkout add/omnidir

2. Install depended libraries. Build sources.

.. code-block::

 $ cd ~/catkin_ws                                                           
 $ rosdep install --from-paths src --ignore-src -r -y                       
 $ catkin_make install && source install/setup.bash                         

Usage
===========

As always the case with all kinds of robots, you should first test on the simulator then run the real robot.

Run the robot (simulation & real)
------------------------------------------------

.. code-block::

 $ roslaunch spur_gazebo spur_world.launch           # Simulation
 $ roslaunch spur_controller spur_controller.launch  # Real robot

Visualize laser range on Gazebo
########################################

.. code-block::

 $ roslaunch spur_gazebo spur_world.launch visualize_laser:=true

.. image:: https://cloud.githubusercontent.com/assets/1840401/8394557/2ff52470-1cf2-11e5-8dcc-6a2565463694.png
   :width: 500 px
   :alt: Laser range visualized on RViz and Gazebo
   :align: left

Teleoperation
------------------------------------------------

Teleop with joystick (with Elecom PS3)
########################################

The following note is confirmed with `PS3-Elecom <http://www.amazon.co.jp/ELECOM-USB%E3%82%B2%E3%83%BC%E3%83%A0%E3%83%91%E3%83%83%E3%83%89-12%E3%83%9C%E3%82%BF%E3%83%B3-%E3%83%96%E3%83%A9%E3%83%83%E3%82%AF-JC-GMU3312SPBK/dp/B003UIRIHC>`_ (sorry only Japanese web sites are found).

1. Make sure your joystick device is paired with your computer.

* `For bluetooth-based joystick <http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle>`_

2. Then run:

.. code-block::

 $ roslaunch spur_controller joy_teleop.launch
 $ roslaunch spur_controller joy_teleop.launch joy_port:=/dev/input/js1  # If joy is found on a different port

3. To use PS3-Elecom, press "Mode" button twice to enable analog input. Also you may need to keep pressing the button 9 during operation.

Teleop with keyboard
########################

.. code-block::

 $ roslaunch spur_controller kb_teleop.launch

Configuration
------------------------------

Change idle stop time
########################

This robot's base stops when no velocity message (linear and angular represented by `geometry_msgs/Twist <http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html>`_) is received for a certain period of time (default: `3.0 seconds <https://github.com/tork-a/spur/blob/0a53cfad271425ae1b7365e6e5ef8362bd5ac3c3/spur_controller/launch/base_controller.launch#L2>`_). You can change that from commandline option when you boot the robot's controller.

.. code-block::

 $ roslaunch spur_gazebo spur_world.launch sec_idle:=1.0           # Simulation
 $ roslaunch spur_controller spur_controller.launch sec_idle:=1.0  # Real robot

Change base velocity
########################

`spur_controller_configuration.yaml <https://github.com/tork-a/spur/blob/master/spur_controller/config/spur_controller_configuration.yaml>`_ defines each motor's configuration. You should not, however, directly modify this file (you can, but not recommended). Instead, follow:

1. Modify `spur_controller_configuration_gen.sh <https://github.com/tork-a/spur/blob/master/spur_controller/config/spur_controller_configuration_gen.sh>`_ as you like.
2. Then run it. This yields the aforementioned `spur_controller_configuration.yaml`.

.. code-block::

  $ ./spur_controller_configuration_gen.sh

Community
==========

* `Forum (in English, encouraged) <http://answers.ros.org/>`_
* `Forum (in Japanese) <https://groups.google.com/forum/#!forum/ros-japan-users>`_
* `Report issues / submit request <https://github.com/tork-a/spur/issues>`_

.. |buildstatus_sourcedeb| image:: http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-spur_sourcedeb
.. |buildstatus_devel_travis| image:: https://travis-ci.org/tork-a/spur.png?branch=master
.. |buildstatus_devel_ros.org| image:: http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-spur
.. |buildstatus_release| image:: https://travis-ci.org/tork-a/spur.png?branch=master
.. |buildstatus_binarydeb_amd64| image:: http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-spur_binarydeb_trusty_amd64
.. |buildstatus_doc| image:: http://jenkins.ros.org/buildStatus/icon?job=doc-indigo-spur
.. |job_devel-indigo-spur| image:: http://jenkins.ros.org/job/devel-indigo-spur/test/trend?job
