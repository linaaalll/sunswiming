"""
mavros for this robot. Use this instead of `ros2 launch mavros apm.launch`.

    ros2 launch ~/lina/sunswiming/launch_file/launch/mavros.launch.py

It is the stock apm.launch plus ONE change: mavros talks to ArduSub as
system id 255 instead of 1.

WHY: ArduSub only obeys RC override (our stick commands) from the system id
stored in its SYSID_MYGCS parameter, which is 255 by default (QGroundControl's
id). The stock launch makes mavros system 1, so arming and mode changes work
(any id may send those) but every stick command is silently thrown away.
That is what the pool bags of 13 Nov look like: armed, right mode, our
throttle at 1199, and ArduSub's own throttle readout sitting at neutral.

Check what the robot expects (mavros running):

    ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: true}"
    ros2 param get /mavros/param SYSID_MYGCS

If that is not 255, pass the same number here:  system_id:=<number>

Options:
    fcu_url:=udp://:14770@192.168.1.68:14770     where ArduSub is (this is the default)
    system_id:=255                               who mavros says it is
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mavros_launch_dir = Path(get_package_share_directory('mavros')) / 'launch'

    # No name= on purpose: mavros_node starts several nodes in one process
    # (the router, the "mavros" node, every plugin) and these settings have
    # to reach all of them, the same way the stock launch file does it.
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            str(mavros_launch_dir / 'apm_pluginlists.yaml'),
            str(mavros_launch_dir / 'apm_config.yaml'),
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'system_id': ParameterValue(LaunchConfiguration('system_id'), value_type=int),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='udp://:14770@192.168.1.68:14770'),
        DeclareLaunchArgument('system_id', default_value='255',
                              description='must equal ArduSub SYSID_MYGCS'),
        mavros,
    ])
