^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package examples_rclcpp_minimal_action_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.3 (2019-03-09)
------------------

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
