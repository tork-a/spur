^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spur
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [feat] Add simulation capability of dynamixel's joint_state_publisher (temporary addition until https://github.com/arebgun/dynamixel_motor/pull/27 and https://github.com/arebgun/dynamixel_motor/pull/28 get merged)
* [sys] add test
* [sys] Fix files wrong location
* [sys] workaround for travis test (pass visualize_laser as arg. Only hydro can be tested on travis)
* [sys] Fix CMake build rule `issue40 <https://github.com/tork-a/spur/pull/40>`_
* [sys] urg_node only runs with simulation
* Contributors: TORK 534o

0.2.2 (2015-06-30)
------------------
* [Feat] Enable Hokuyo
* [Sys] Adjust to bringup pkg. Add 2dnav pkg up to gmapping feature
* [Sys] Port general stuff to spur_bringup package
* Contributors: Isaac IY Saito

0.2.1 (2015-05-13)
------------------
* (Fix) Add more dependency
* Contributors: Isaac IY Saito

0.2.0 (2015-05-09)
------------------
* (Fix) Complete adjusting model to real robot
* (Fix) Specific dependency
* (Feature) Allow configuring idle time for Twist command.
* (Doc) Copyright to the project owner. Update package description.
* Contributors: Isaac IY Saito

0.1.3 (2015-05-07)
------------------
* joy launch improvement
* Contributors: Isaac IY Saito

0.1.2 (2015-05-07)
------------------
* (Fix) Correct sides of wheels (replacing L and R).
* (Feat.) Separate launches to allow dynamixel to start by itself.
* (Maintenance) More correct dependency.
* Contributors: Isaac IY Saito

0.1.1 (2015-04-29)
------------------
* Enable Odometry RViz plugin.
* (Feature) Add a launch for joy, kb teleop
* Contributors: Isaac IY Saito

0.1.0 (2015-03-30)
------------------
