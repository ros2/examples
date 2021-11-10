# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import sys

from examples_rclpy_executors.listener import Listener
from examples_rclpy_executors.talker import Talker
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    try:
        talker = Talker()
        listener = Listener()

        # Runs all callbacks in the main thread
        executor = SingleThreadedExecutor()
        # Add imported nodes to this executor
        executor.add_node(talker)
        executor.add_node(listener)

        try:
            # Execute callbacks for both nodes as they become ready
            executor.spin()
        finally:
            executor.shutdown()
            listener.destroy_node()
            talker.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
