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
# Async task (2 second delay)
ros2 service call /async_task std_srvs/srv/Trigger

# Sequential task with flag
ros2 service call /sequential_task std_srvs/srv/SetBool "{data: true}"

# Reset counter
ros2 service call /reset_counter std_srvs/srv/Trigger
```
