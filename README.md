# srv_call_timer
`srv_call_timer` is a ROS2 package which provides a tool to measure the response time of a service.
It measures the response time over a specified number of calls and provides statistics such as min, max, mean and median response time.
Currently, it does not support passing arguments to services.

# Installation and Building
`srv_call_timer` can be cloned into the `src` directory of your ROS2 workspace.
It can be built using `colcon build` in the root directory of your workspace.

# Usage
`srv_call_timer` can be used with `ros2 run`, like other ROS2 packages.
To time a service `/example_service` from package `example_package` of type `ExampleServiceType`, you can use the following:

`ros2 run srv_call_timer srv_call_timer /example_service example_package ExampleServiceType`

Note that this tool works by dynamically importing your service in Python.
It expects your service type to be an attribute of the `srv` module in your package.
In the given example, it would import `ExampleServiceType` from `example_package.srv`.
