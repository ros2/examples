#!/usr/bin/env python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from copy import deepcopy
from threading import Lock, Thread
import time

import rclpy

import matplotlib.pyplot as plt

from std_msgs.msg import Int64


# This script starts a node with a subscription to a relatively high-frequency topic.
# The data received from the subscription callbacks is plotted in the main thread,
# while the callbacks themselves are processed in a second thread.
# Plot updates are scheduled at a regular interval, independent of the rate at which data is
# received. Slow plotting calls will not block the reception of frequent data messages.

time_between_plot_updates = 1  # time in seconds between plot updates


# Class for managing the storage and display of data received
class DataPlotter:
    def __init__(self):
        plt.ion()
        self.fig = plt.figure()
        self.received_data = []
        # As the received data can be access by both the main plotting thread and the thread
        # processing data callbacks, its access must be protected by a lock
        self.received_data_lock = Lock()

    # Called when new data is received
    def data_callback(self, msg):
        print('Data received: {0}'.format(msg.data))

        # Store received data
        self.received_data_lock.acquire()
        self.received_data.append(msg.data)
        self.received_data_lock.release()

    # Called when it's time to update the plot
    def update_plot(self):
        print('Updating plot')

        # Make a copy of the data that will be needed, so that other threads aren't blocked
        self.received_data_lock.acquire()
        y_data = deepcopy(self.received_data)
        self.received_data_lock.release()

        # Update the plot
        x_data = range(0, len(y_data))
        plt.plot(x_data, y_data, '-r')
        self.fig.canvas.draw()

        print('Finished updating plot')


class RCLPYThread(Thread):
    def __init__(self):
        rclpy.init()
        Thread.__init__(self)

    def run(self):
        self.node = rclpy.create_node('data_listener')
        self.sub = self.node.create_subscription(Int64, 'data', data_plotter.data_callback)

        while rclpy.ok():
            # Wait for messages with a timeout, otherwise this thread will block other threads
            # until a message is received
            timeout = 0.05  # seconds
            rclpy.spin_once(self.node, timeout)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()

data_plotter = DataPlotter()


def main():
    try:
        # Start a thread that will process the subscription
        thread = RCLPYThread()
        thread.start()

        # Continually update the display in the main thread
        last_plot_time = time.time()
        while(1):
            now = time.time()
            elapsed_time = now - last_plot_time
            if elapsed_time > time_between_plot_updates:
                last_plot_time = now
                data_plotter.update_plot()

    except KeyboardInterrupt:
        thread.stop()
        # Block the main thread until the other thread terminates
        thread.join()


if __name__ == '__main__':
    main()
