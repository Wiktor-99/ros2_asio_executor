# ros2_asio_executor

ROS2 executor with Boost.Asio integration for async service handlers using C++20 coroutines.

## Build

```bash
colcon build --packages-select ros2_asio_executor
source install/setup.bash
```

## Run

```bash
ros2 run ros2_asio_executor example_node
```

## Test Services

```bash
# Async task (10 second delay)
ros2 service call /async_task std_srvs/srv/Trigger

# To call another service - to check if we're not stuck in while handling callbacks using asio event loop
ros2 service call /another_async_task std_srvs/srv/Trigger
```
