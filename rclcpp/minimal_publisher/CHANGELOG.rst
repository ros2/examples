^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package examples_rclcpp_minimal_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2019-05-29)
------------------

0.7.2 (2019-05-20)
------------------

0.7.1 (2019-05-08)
------------------
* Avoid deprecated API's by providing history settings (`#240 <https://github.com/ros2/examples/issues/240>`_)
* avoid deprecated publish signature (`#239 <https://github.com/ros2/examples/issues/239>`_)
* Contributors: William Woodall

0.7.0 (2019-04-14)
------------------

0.6.2 (2019-02-08)
------------------

0.6.0 (2018-11-20)
------------------
* Added semicolons to all RCLCPP and RCUTILS macros. (`#214 <https://github.com/ros2/examples/issues/214>`_)
* Contributors: Chris Lalancette

0.5.1 (2018-06-27)
------------------
* make Mikael Arguedas the maintainer (`#212 <https://github.com/ros2/examples/issues/212>`_)
* Contributors: Mikael Arguedas

0.5.0 (2018-06-26)
------------------
* Add #include <chrono> if using std::chrono_literals `#198 <https://github.com/ros2/examples/issues/198>`_
* Contributors: Mikael Arguedas, Yutaka Kondo

0.4.0 (2017-12-08)
------------------
* Remove node:: namespace (`#192 <https://github.com/ros2/examples/issues/192>`_)
  connects to `ros2/rclcpp#416 <https://github.com/ros2/rclcpp/issues/416>`_
* Use logging (`#190 <https://github.com/ros2/examples/issues/190>`_)
* Switch to using rate (`#188 <https://github.com/ros2/examples/issues/188>`_)
* 0.0.3
* call shutdown before exiting (`#179 <https://github.com/ros2/examples/issues/179>`_)
* 0.0.2
* rename executables with shorter names (`#177 <https://github.com/ros2/examples/issues/177>`_)
* install executables in package specific path `#173 <https://github.com/ros2/examples/issues/173>`_
* use CMAKE_X_STANDARD and check compiler rather than platform
* add pedantic flag
* Cpp14 (`#147 <https://github.com/ros2/examples/issues/147>`_)
  move to C++14 and use standard duration literals
* Minimal service and client (`#138 <https://github.com/ros2/examples/issues/138>`_)
* Add examples\_ prefix to package names to avoid future collisions. `#137 <https://github.com/ros2/examples/issues/137>`_
* change talker/listener to minimal_publisher/minimal_subscriber
* Contributors: Dirk Thomas, Mikael Arguedas, Morgan Quigley, dhood
