from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located'))
    ld.add_action(DeclareLaunchArgument('urdf_package_path',
                                        description='The path to the robot description relative to the package root'))

    package_dir = FindPackageShare(LaunchConfiguration('urdf_package'))
    urdf_path = PathJoinSubstitution([package_dir, LaunchConfiguration('urdf_package_path')])

    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    ld.add_action(robot_state_publisher_node)
    return ld
