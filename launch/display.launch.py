from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    urdf_launch_package = FindPackageShare('urdf_launch')

    ld.add_action(DeclareLaunchArgument(name='jsp_gui', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui'))

    default_rviz_config_path = PathJoinSubstitution([urdf_launch_package, 'config', 'urdf.rviz'])
    ld.add_action(DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    ld.add_action(DeclareLaunchArgument('use_sim_time',
                                        default_value='false',
                                        choices=['true', 'false'],
                                        description='Use simulation clock if true'))
    SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    # need to manually pass configuration in because of https://github.com/ros2/launch/issues/313
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([urdf_launch_package, 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    ))

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui')),
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui')),
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))
    return ld
