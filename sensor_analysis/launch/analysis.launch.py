from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('sensor_analysis'),
                          'config', 'analysis.yaml')
    simulated = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='sensor_analysis', executable='imu_analysis_node',
             name='imu_analysis', parameters=[config, {'use_sim_time': simulated}],
             output='screen'),
        Node(package='sensor_analysis', executable='odom_analysis_node',
             name='odom_analysis', parameters=[config, {'use_sim_time': simulated}],
             output='screen'),
    ])
