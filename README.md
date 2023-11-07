# urdf_launch

This package contains launch files and configurations for common URDF operations.

## Load Description

[`description.launch.py`](launch/description.launch.py)
 * Loads the URDF/Xacro robot model as a parameter, based on launch arguments
 * Launches a single node, `robot_state_publisher`, with the robot model parameter.
    * This results in the robot description being available as a topic.

The result is that you can perform the above actions with just 5 lines (not including boilerplate) in your own launch file.

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'turtlebot3_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'turtlebot3_burger.urdf'])}.items()
    ))

    return ld

```

## Display Robot Model
When developing a URDF robot model, it is often useful to display just the robot model in RViz. [`display.launch.py`](launch/display.launch.py) does the above tasks PLUS:
 * Launches RViz with a preconfigured setup
 * Launches a joint state publisher (with optional GUI)

This can be used in its own launch file, like the previous example, or can be done via command line, e.g.

    ros2 launch urdf_launch display.launch.py urdf_package:=turtlebot3_description urdf_package_path:=urdf/turtlebot3_burger.urdf
