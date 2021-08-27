^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qb_device_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.1 (2021-08-27)
------------------

2.2.0 (2021-08-27)
------------------
* Fixed reconnection on killing nodes.
* Refactor control structures initialization
* Add Gazebo SoftHand and SoftHand 2 Motors plugin skeletons
* Refactor Gazebo plugin to prepare for SoftHand simulation
* Add hardware interface for SoftHand 2 Motors
* Set a better name for fake joint simulation variables
* Added the control mode switch service. Added a parameter to choose to use .yaml limits or firmware ones.
* Add Gazebo support for qbmove devices
* Fix hardcoded params in launch files since ros_comm#1354 has been backported to kinetic
* Fix 'pass_all_args' workaround since ros_comm#889 has been backported to kinetic
* Added compatibility to qb SoftClaw in .launch files
* Update arg description
* Added a parameter to use the device with or without other robots. Added some motor limits when the qbmoves are used in kinematic configs.

2.1.1 (2019-10-07)
------------------

2.1.0 (2019-05-28)
------------------
* Improve inheritance for other devices
* Update documentation
* Add a simulator mode to debug joint trajectories
* Add specific joint limits for the delta
* Add interactive markers startup activation
* Fix set/reset use_waypoints at each launch

2.0.3 (2018-08-09)
------------------

2.0.2 (2018-08-07)
------------------

2.0.1 (2018-06-01)
------------------

2.0.0 (2018-05-30)
------------------
* Refactor launch files
* Fix minors
* Add a blocking setCommands method
* Fix until ros_comm#889 is fixed
* Refactor launch files and fix controllers spawner

1.2.2 (2017-11-30)
------------------
* Fix launch file

1.1.0 (2017-11-24)
------------------
* Fix control nodes type

1.0.8 (2017-06-27)
------------------

1.0.7 (2017-06-26)
------------------

1.0.6 (2017-06-23)
------------------
* Update cmake version to match Kinetic standards

1.0.5 (2017-06-22)
------------------

1.0.4 (2017-06-21)
------------------

1.0.3 (2017-06-21)
------------------
* fix cmake settings to solve isolated builds

1.0.2 (2017-06-20)
------------------

1.0.1 (2017-06-19)
------------------
* first public release for Kinetic
