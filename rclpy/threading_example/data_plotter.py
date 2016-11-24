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
        self.time_last_data_received = time.time()

    # Called when new data is received
    def data_callback(self, msg):
        print('Data received: {0}'.format(msg.data))
        now = time.time()
        time_between_data_callbacks = now - self.time_last_data_received
        self.time_last_data_received = now
        print('Time between data callbacks: %.3f seconds' % time_between_data_callbacks)

        # Store received data
        self.received_data_lock.acquire()
        self.received_data.append(msg.data)
        self.received_data_lock.release()

    # Called when it's time to update the plot
    def update_plot(self):
        print('Updating plot')
        start_time = time.time()

        # Make a copy of the data that will be needed, so that other threads aren't blocked
        self.received_data_lock.acquire()
        y_data = deepcopy(self.received_data)
        self.received_data_lock.release()

        # Update the plot
        x_data = range(0, len(y_data))
        plt.plot(x_data, y_data, '-r')
        self.fig.canvas.draw()

        elapsed_time = time.time() - start_time
        print('Time taken to update plot: %.3f seconds' % elapsed_time)


class RCLPYThread(Thread):
    def __init__(self):
        rclpy.init()
        Thread.__init__(self)

    def run(self):
        self.node = rclpy.create_node('data_listener')
        sub = self.node.create_subscription(Int64, 'data', data_plotter.data_callback)
        while rclpy.ok():
            # Wait for messages with a timeout, otherwise this thread will block other threads
            # until a message is received
            rclpy.spin_once(self.node, 0.05)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()
        return


data_plotter = DataPlotter()
def main():
    try:
        # Start a thread that will process the subscription in the background
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
        # Stop the main thread and join the background thread
        thread.stop()
        thread.join()


if __name__ == '__main__':
    main()
