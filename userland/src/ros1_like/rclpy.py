from rcl_api import executor_add_node
from rcl_api import executor_create
from rcl_api import executor_execute_any_executable
from rcl_api import executor_get_next_executable

from rcl_api import node_create

from rcl_api import subscription_create
from rcl_api import subscription_take


def ok():
    return True


class Node():
    def __init__(self, name, context=None):
        if not isinstance(name, str):
            raise TypeError("First parameter to Node must be a string, "
                            "the name of the node.")
        self.__node_handle = node_create(name)
        self.__callback_groups = []

    def get_impl_handle(self):
        return self.__node_handle

    def create_subscription(self, topic, callback, queue_size):
        if not isinstance(topic, str):
            raise TypeError(
                "First parameter to create_subscription must be a string, "
                "the name of the node.")
        handle = subscription_create(self.__node_handle, topic, queue_size)
        subscription = Subscription(handle, callback)
        self.__subscriptions.append(subscription)
        return subscription


class CallbackGroup:
    def __init__(self, type):
        self.__subscriptions = []

    def add_subscription(self, subscription):
        self.__subscriptions.append(subscription)


class Subscription:
    def __init__(self, subscription_handle, callback):
        self.__handle = subscription_handle
        self.__callback = callback


class Executor:
    def __init__(self):
        self.executor_handle = executor_create()
        self.__nodes = []

    def add_node(self, node):
        self.__nodes.append(node)

    def execute_any_executable(self, any_exec):
        if any_exec.subscription_handle is not None:
            self.execute_subscription(any_exec.subscription_handle)
        if any_exec.timer_handle is not None:
            self.execute_timer(any_exec.timer_handle)

    def execute_timer(self, timer_handle):
        pass

    def execute_subscription(self, subscription_handle):
        for node in self.__nodes:
            for callback_group in node.__callback_groups:
                for subscription in callback_group.__subscriptions:
                    if subscription.__handle == subscription_handle:
                        msg = subscription_take(subscription.__handle)
                        subscription.__callback(msg)


class SingleThreadedExecutor(Executor):
    def __init__(self):
        Executor.__init__(self)

    def spin(self):
        while ok():
            any_exec = executor_get_next_executable()
            self.execute_any_executable(any_exec)
            executor_execute_any_executable(any_exec)

from threading import Lock
from threading import Thread
from multiprocessing import cpu_count


class MultiThreadedExecutor(Executor):
    def __init__(self):
        Executor.__init__(self)
        self.threads = []
        self.wait_lock = Lock()
        self.__num_threads = cpu_count()

    def spin(self):
        for i in range(self.__num_threads):
            self.threads.append(Thread(target=self.run, args=(i)))
        for thread in self.threads:
            thread.join()

    def get_number_of_threads(self):
        return self.__num_threads

    def run(self, thread_number):
        while ok():
            with self.wait_lock:
                if not ok():
                    return
                any_exec = executor_get_next_executable()
                executor_execute_any_executable(any_exec)
