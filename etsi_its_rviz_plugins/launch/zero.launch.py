import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config = 'config/demo.rviz'
    rviz_config_path = os.path.join(
        get_package_share_directory('etsi_its_rviz_plugins'),
        rviz_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if false'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d'+str(rviz_config_path)]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["166021.443", "0.000", "0.0", "0.0", "0.0", "-0.0399933", "utm_31N", "map"],
        ),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r 0.1', '/map_link/navsatfix', 'sensor_msgs/msg/NavSatFix', 
                '''{header:
                        {stamp: {
                            sec: 0,
                            nanosec: 0
                        },
                        frame_id: "map"
                    },
                    latitude: 0.0,
                    longitude: 0.0,
                    altitude: 0.0
                }'''],
            output='screen',
        ),
    ])
