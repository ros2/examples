^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package examples_rclpy_minimal_action_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.3 (2019-03-09)
------------------
* rclpy action examples (Backport of # 222) (`#229 <https://github.com/ros2/examples/issues/229>`_)
  * Copy rclpy action examples from `#216 <https://github.com/ros2/examples/issues/216>`_
  * Bump version of examples_rclpy_action to match other packages
  * Consolidate composable action client example
  * Add action client example that is not composable
  * Wait for action server
  * Restructure into separate packages for action client and action server examples
  * Update API in action server examples
  * Rename action server examples
  * Add action server example that is not composable
  * Update setup.py
  * Fix syntax
  * Update action client examples to use goal handle API for the result
  * Improve action client output and result handling
  * Update action server examples
  * Move action examples into Python packages
  * Add action client cancel example
  * Address review comments
  * Update author
  * Update copyright year
  * Shutdown client example after result received
  * GoalResponse.ACCEPT_AND_EXECUTE -> GoalResponse.ACCEPT
  * Fix client_cancel example
  * Remove race from server_single_goal example
  * Add action server example that defers the execution of an accepted goal
  * Reduce the timer period for the client cancel example
  * Support canceling goals with non-composable action server example
  * Add action server example that queues goals
  * Add author tag to package.xml
* Contributors: Jacob Perron

Forthcoming
-----------
* rclpy action examples (Backport of # 222) (`#229 <https://github.com/ros2/examples/issues/229>`_)
  * Copy rclpy action examples from `#216 <https://github.com/ros2/examples/issues/216>`_
  * Bump version of examples_rclpy_action to match other packages
  * Consolidate composable action client example
  * Add action client example that is not composable
  * Wait for action server
  * Restructure into separate packages for action client and action server examples
  This package structure is consistent with examples for services and topics.
  * Update API in action server examples
  * Rename action server examples
  Now the 'simplest' example is 'server.py'.
  * Add action server example that is not composable
  * Update setup.py
  * Fix syntax
  * Update action client examples to use goal handle API for the result
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Improve action client output and result handling
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Update action server examples
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Move action examples into Python packages
  This avoid top-level module names from clashing when installed.
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Add action client cancel example
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Address review comments
  * Update author
  * Update copyright year
  * Shutdown client example after result received
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * GoalResponse.ACCEPT_AND_EXECUTE -> GoalResponse.ACCEPT
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Fix client_cancel example
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Remove race from server_single_goal example
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Add action server example that defers the execution of an accepted goal
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Reduce the timer period for the client cancel example
  This makes it easy to experiment with the scenario where a deferred goal is canceled prior to execution.
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Support canceling goals with non-composable action server example
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Add action server example that queues goals
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Address review
  - Fix comment
  - Add author tag to package.xml
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
* Contributors: Jacob Perron

0.6.2 (2019-02-08)
------------------

0.6.1 (2018-12-07)
------------------

0.6.0 (2018-11-20)
------------------

0.5.1 (2018-06-27)
------------------

0.5.0 (2018-06-26)
------------------

0.4.0 (2017-12-08)
------------------
