import os
import sys

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import yaml


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ft_sensor_a_name',
            default_value='ft_sensor0'
        ),
        launch_ros.actions.Node(
            package='rokubimini_ethercat',
            executable='rokubimini_ethercat_bus_manager_node',
            name='bus0',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                '/home/kairos/kairos_ws/src/bota_ft_sensor_driver/sensor_offset/rokubimini_sensor_minimal.yaml',
                {
                    'num_spinners': 1
                },
                {
                    'time_step': 0.1
                },
                {
                    'ethercat_bus': 'enp3s0'
                },
                {
                    'rokubiminis': ['ft_sensor0']  # ['ft_sensor0', 'ft_sensor1']
                },
                {
                    'rokubiminis/ft_sensor0/name': launch.substitutions.LaunchConfiguration('ft_sensor_a_name')
                },
                {
                    'rokubiminis/ft_sensor0/product_name': 'BFT-ROKS-ECAT-M8'
                },
                {
                    'rokubiminis/ft_sensor0/ethercat_address': 1
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
