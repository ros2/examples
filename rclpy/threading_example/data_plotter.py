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


time_between_plot_updates = 1  # in seconds

class MyApplication:
    def __init__(self):
        self.data_plotter = DataPlotter()
        self.received_data = []
        # As the received data can be access by both the main plotting thread and the thread
        # processing data callbacks, its access must be protected by a lock
        self.received_data_lock = Lock()

    def data_callback(self, msg):
        print('Data received: {0}'.format(msg.data))
        self.received_data_lock.acquire()
        self.received_data.append(msg.data)
        self.received_data_lock.release()

    def update_plot(self):
        print('Updating plot')
        self.received_data_lock.acquire()
        data = deepcopy(self.received_data)
        self.received_data_lock.release()
        self.data_plotter.update_plot(data)


class DataPlotter:
    def __init__(self):
        self.make_plot()

    def make_plot(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.line, = self.ax.plot([], [], '-r')

    def update_plot(self, data):
        y_data = data
        x_data = range(0, len(y_data))
        self.line, = self.ax.plot(x_data, y_data, '-r')

        try:
            self.fig.canvas.draw()
        except:
            print('except interrupt')
            raise


class RCLPYThread(Thread):
    def __init__(self):
        rclpy.init()
        Thread.__init__(self)

    def run(self):
        self.node = rclpy.create_node('data_listener')
        sub = self.node.create_subscription(Int64, 'data', my_application.data_callback)
        while rclpy.ok():
            rclpy.spin_once(self.node, 0.05)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()
        return


my_application = MyApplication()
def main():
    try:
        # Start a background thread that will process the subscription
        thread = RCLPYThread()
        thread.start()

        last_plot_time = time.time()
        while(1):
            now = time.time()
            elapsed_time = now - last_plot_time
            if elapsed_time > time_between_plot_updates:
                last_plot_time = now
                my_application.update_plot()
        thread.join()

    except KeyboardInterrupt:
        thread.stop()
        thread.join()


if __name__ == '__main__':
    main()
