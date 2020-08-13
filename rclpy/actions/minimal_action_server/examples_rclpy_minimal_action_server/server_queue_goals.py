# Copyright 2018-2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import collections
import threading
import time

from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server')
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def handle_accepted_callback(self, goal_handle):
        """Start or defer execution of an already accepted goal."""
        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.get_logger().info('Goal put in the queue')
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self._current_goal.execute()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute a goal."""
        try:
            self.get_logger().info('Executing goal...')

            # Append the seeds for the Fibonacci sequence
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.sequence = [0, 1]

            # Start executing the action
            for i in range(1, goal_handle.request.order):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return Fibonacci.Result()

                # Update Fibonacci sequence
                feedback_msg.sequence.append(
                    feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

                self.get_logger().info(
                    'Publishing feedback: {0}'.format(feedback_msg.sequence))

                # Publish the feedback
                goal_handle.publish_feedback(feedback_msg)

                # Sleep for demonstration purposes
                time.sleep(1)

            goal_handle.succeed()

            # Populate result message
            result = Fibonacci.Result()
            result.sequence = feedback_msg.sequence

            self.get_logger().info(
                'Returning result: {0}'.format(result.sequence))

            return result
        finally:
            with self._goal_queue_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self._current_goal = self._goal_queue.popleft()
                    self.get_logger().info('Next goal pulled from the queue')
                    self._current_goal.execute()
                except IndexError:
                    # No goal in the queue.
                    self._current_goal = None


def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = MinimalActionServer()

    executor = MultiThreadedExecutor()

    rclpy.spin(minimal_action_server, executor=executor)

    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
