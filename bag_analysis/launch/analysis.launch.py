from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('accel_scale', default_value='9.80665'),

        Node(
            package='bag_analysis',
            executable='odom_analysis',
            name='odom_analysis',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': LaunchConfiguration('odom_topic'),
            }],
        ),

        Node(
            package='bag_analysis',
            executable='imu_analysis',
            name='imu_analysis',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': LaunchConfiguration('imu_topic'),
                'accel_scale': ParameterValue(
                    LaunchConfiguration('accel_scale'),
                    value_type=float,
                ),
            }],
        ),
    ])
