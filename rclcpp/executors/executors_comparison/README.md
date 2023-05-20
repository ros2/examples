# Simple ros2 package to understand threads

Visit [here](https://motion-boseong.vercel.app/threading-ros2) for explanations.

## Running node

### 1. Single-thread test  
In terminal A (also for the two belows)

```
ros2 run simple_thread_tester publisher_node
```

In terminal B

```
ros2 run simple_thread_tester single_thread_subscriber_node
```
### 2. Multi-thread reentrant callback group test 

```
ros2 run simple_thread_tester multi_thread_reentrant_subscriber_node
```

### 3. Multi-thread mutually exclusive callback group test

```
ros2 run simple_thread_tester multi_thread_reentrant_subscriber_node
# If simulate mutex 
ros2 param set /subscriber_node use_mutex true
```
