^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package examples_rclcpp_minimal_action_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.1 (2024-07-10)
-------------------

0.18.0 (2023-04-11)
-------------------

0.17.1 (2023-03-01)
-------------------

0.17.0 (2023-02-14)
-------------------
* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`_)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.16.2 (2022-11-02)
-------------------

0.16.1 (2022-09-13)
-------------------

0.16.0 (2022-04-29)
-------------------

0.15.0 (2022-03-01)
-------------------

0.14.0 (2022-01-14)
-------------------
* Updated maintainers (`#329 <https://github.com/ros2/examples/issues/329>`_)
* Contributors: Aditya Pande

0.13.0 (2021-10-18)
-------------------

0.12.0 (2021-08-05)
-------------------

0.11.2 (2021-04-26)
-------------------

0.11.1 (2021-04-12)
-------------------

0.11.0 (2021-04-06)
-------------------

0.10.3 (2021-03-18)
-------------------

0.10.2 (2021-01-25)
-------------------

0.10.1 (2020-12-10)
-------------------
* Update maintainers (`#292 <https://github.com/ros2/examples/issues/292>`_)
* Contributors: Shane Loretz

0.10.0 (2020-09-21)
-------------------
* Update goal response callback signature (`#291 <https://github.com/ros2/examples/issues/291>`_)
* Make sure to include what you use in all examples. (`#284 <https://github.com/ros2/examples/issues/284>`_)
* Added common linters (`#265 <https://github.com/ros2/examples/issues/265>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Jacob Perron

0.9.2 (2020-06-01)
------------------

0.9.1 (2020-05-26)
------------------
* Fixed action_client sequence type (`#268 <https://github.com/ros2/examples/issues/268>`_)
* Contributors: Alejandro Hernández Cordero

0.9.0 (2020-04-30)
------------------
* avoid new deprecations (`#267 <https://github.com/ros2/examples/issues/267>`_)
* Restructure rclcpp folders (`#264 <https://github.com/ros2/examples/issues/264>`_)
* Contributors: Marya Belanger, William Woodall

0.8.2 (2019-11-19)
------------------

0.8.1 (2019-10-24)
------------------

0.7.3 (2019-05-29)
------------------
* Use action client get result method (`#245 <https://github.com/ros2/examples/issues/245>`_)
  Since a result callback is not provided when sending the goal, the goal handle is not "result aware"
  and calling the action client method will make it so.
  The behaviour was changed in https://github.com/ros2/rclcpp/pull/701.
* Contributors: Jacob Perron

0.7.2 (2019-05-20)
------------------

0.7.1 (2019-05-08)
------------------
* Avoid deprecated API's by providing history settings (`#240 <https://github.com/ros2/examples/issues/240>`_)
* Add rclcpp action examples using member functions
* Use options struct when action client sends a goal
* Contributors: Jacob Perron, William Woodall

0.7.0 (2019-04-14)
------------------
* Updated to use separated action types. (`#227 <https://github.com/ros2/examples/issues/227>`_)
* Contributors: Dirk Thomas

0.6.2 (2019-02-08)
------------------

0.6.1 (2018-12-07)
------------------
* Rclcpp action examples (`#220 <https://github.com/ros2/examples/issues/220>`_)
  * Add minimal_action_server package
  Contains a non-composable implementation with global variables.
  * Add minimal_action_client package
  Contains a non-composable implementation.
  * Add action client example with feedback
  * async python action client example
  * goal -> future
  * fibb -> fib"
  * Syncronous action client example
  * No break statement
  * Update client examples to use separate rcl_action package
  * Add ClientGoalHandle to action client examplesj
  * Add action client with cancel example
  * python non-composable server example
  * [wip] Update action server cpp example
  * remove unnecessary event
  * create_action_server -> ActionServer
  * missing paren
  * Add example of multiple goals in parallel
  * No need for lock
  * Reentrant callback group for execute
  * create_action_client -> ActionClient
  * Fix copyright date
  * )
  * -)
  * Refactor action server cpp example
  * Fix action server cpp example
  Seed the fibonacci sequence and remove const.
  * Fix action server cpp example
  Forgot to increment in Fibonacci sequence loop.
  * Syntax fixes
  * node -> self
  * handle cb returns accept or reject
  * Update action client cpp example
  Return goal handle (containing future) when sending a goal.
  * Preempt goals
  * whitespace removal
  * execute returns result
  * Add missing resources
  * Syntax error
  * Add rcl_action dependency
  * Update maintainer
  * Use goal message getter and alias ResultResponse type
  * Make minimal_action_server work with rclcpp_action
  * Client and server communicate
  * handle_execute -> handle_accepted
  * Check if goal was rejected by server
  * Update example to check result
  * action client cancel example C++ works
  * misc changes to compile
  * misc client api changes
  * Remove python examples
  * Wait for action server
* Contributors: Shane Loretz

0.6.0 (2018-11-20)
------------------

0.5.1 (2018-06-27)
------------------

0.5.0 (2018-06-26)
------------------

0.4.0 (2017-12-08)
------------------
