^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qb_device_hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.1 (2021-08-27)
------------------
* Fix qbhand movement at startup
* Fix qb SoftHands startup

2.2.0 (2021-08-27)
------------------
* Replaced error stream with throttle
* Add controller startup sync to avoid hardcoded wait
* Refactor control structures initialization
* Pass command_old value in PositionJointSaturationHandle constructor
* Set out-of-range warning with _TRHOTTLE mode
* Add methods to retrieve actual device references
* Set a better name for fake joint simulation variables
* Added the control mode switch service. Added a parameter to choose to use .yaml limits or firmware ones.
* Update license

2.1.1 (2019-10-07)
------------------
* Fix initial command and velocity limit for qbmoves

2.1.0 (2019-05-28)
------------------
* Add comments for future changes
* Add a simulator mode to debug joint trajectories
* Fix minor style issues
* Fix position limits initialization
* Fix minor style issues

2.0.3 (2018-08-09)
------------------
* Update license agreement copyright

2.0.2 (2018-08-07)
------------------

2.0.1 (2018-06-01)
------------------

2.0.0 (2018-05-30)
------------------
* Add device configuration to the state topic info
* Fix recovery on service server fault
* Refactor node registration
* Fix accidental repetitions with namespace prefix
* Refactor initialization by extending the base init
* Add method to get currents and positions together
* Fix repetitions reliablity check
* Add a blocking setCommands method
* Refactor launch files and fix controllers spawner
* Fix minors
* Let the user decide whether to read/write or not
* Refactor node registration
* Add repetitions while reading from serial

1.2.2 (2017-11-30)
------------------
* Fix possible fault on registration
* Refactor service clients management

1.1.0 (2017-11-24)
------------------
* Add notes for the future

1.0.8 (2017-06-27)
------------------
* Fix C++11 support for cmake version less than 3.1

1.0.7 (2017-06-26)
------------------
* Fix minor build problems

1.0.6 (2017-06-23)
------------------
* Update cmake version to match Kinetic standards

1.0.5 (2017-06-22)
------------------
* remove gmock unsupported dependencies

1.0.4 (2017-06-21)
------------------
* fix cmake settings to solve isolated builds (install also libraries)

1.0.3 (2017-06-21)
------------------
* fix cmake settings to solve isolated builds

1.0.2 (2017-06-20)
------------------

1.0.1 (2017-06-19)
------------------
* first public release for Kinetic
