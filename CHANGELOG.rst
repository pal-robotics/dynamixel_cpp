^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2016-03-17)
------------------
* Seems that after an overheat we DO need to set the torque again
* Contributors: Sam Pfeiffer

0.0.5 (2016-03-16)
------------------
* Merge branch 'add_torque' into 'cobalt-devel'
  Add inverting a motor commands and torque limit setup
  Also be less verbose with errors (that happen all the time).
  Also fix bug that the head would fall to a 0.0 position when there was a bad read on the current position.
  Removed continuous timer that tried to setup compliance slope as actually the reading of the value was sometimes wrong, not really the value.
  Auto indented code to PAL profile too.
  See merge request !1
* Removed too much verbose output
* Seemingly working version but dynamixel refuses to write torque limit value
* Contributors: Sam Pfeiffer

0.0.4 (2015-07-14)
------------------
* Remove unused dynamic reconfigure header and artifacts
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.3 (2015-07-14)
------------------
* Feedback has joint names
* Node is now configurable using a yaml file
* Remove dynamic_reconfigure
* Lower compliance slope by 1 level, faster response but a bit more responsive to errors
* Contributors: Bence Magyar

0.0.2 (2015-04-14)
------------------
* Add compliance slope (smoothness) to device lib and node
* Init sets slope, added init guards to other functions
* Contributors: Bence Magyar

0.0.1 (2015-01-20)
------------------
* First beta version of library and node.
* Contributors: Bence Magyar
