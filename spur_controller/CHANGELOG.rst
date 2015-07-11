^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spur_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.4 (2015-07-11)
------------------
* set use_base_odom to true as default
* support use_base_odom args for base_contraller, on gazebo it is dsiabled by default and on realroobt, it is enabled
* add publish_odom parm to publish odom from base_controller
* Contributors: Isaac IY Saito

0.2.3 (2015-07-02)
------------------
* [feat] Add simulation capability of dynamixel's joint_state_publisher (temporary addition until https://github.com/arebgun/dynamixel_motor/pull/27 and https://github.com/arebgun/dynamixel_motor/pull/28 get merged)
* [sys] Fix files wrong location
* [sys] workaround for travis test (pass visualize_laser as arg. Only hydro can be tested on travis)
* Contributors: Isaac IY Saito, TORK 534o

0.2.2 (2015-06-30)
------------------
* [Sys] Port general stuff to spur_bringup package
* Contributors: Isaac IY Saito

0.2.1 (2015-05-13)
------------------
* (Fix) Add more dependency
* Contributors: Isaac IY Saito

0.2.0 (2015-05-09)
------------------
* (Fix) Adjust spur controller to the real robot config
* (Fix) Specific dependency
* (Feature) Allow configuring idle time for Twist command.
* (Doc) Copyright to the project owner. Update package description.
* (Doc) PEP8
* Contributors: Isaac IY Saito

0.1.3 (2015-05-07)
------------------
* joy launch improvement
* Contributors: Isaac IY Saito

0.1.2 (2015-05-07)
------------------
* (Fix) Correct sides of wheels (replacing L and R).
* (Feat.) Separate launches to allow dynamixel to start by itself.
* Contributors: Isaac IY Saito

0.1.1 (2015-04-29)
------------------
* (Feature) Add a launch for joy, kb teleop
* Contributors: Isaac IY Saito

0.1.0 (2015-03-30)
------------------
