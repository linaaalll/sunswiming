"""
The qualifier, one command.

Start mavros and the ZED camera first (their own terminals, same as always),
build and source the workspace, then:

    ros2 launch ~/lina/sunswiming/launch_file/launch/qually.launch.py

That is a DRY RUN: everything runs, nothing arms. For a real run:

    ros2 launch ~/lina/sunswiming/launch_file/launch/qually.launch.py dry_run:=false name:=pool_01

Options:
    dry_run:=false         arm and actually drive (default true = motors never armed)
    detector:=colour       colour | none   (yolo comes later, same slot)
    record:=false          don't start the black box recorder
    name:=pool_01          label for the bag folder
    notes:="..."           free text saved next to the bag
    camera:=false          record without the colour image (much smaller bags)
    monitor:=false         don't show the dashboard here (run it in another
                           terminal instead: ros2 run monitor qually_monitor)

What it starts:
    rc_control             cmd_vel -> /mavros/rc/override   (the only node that drives)
    heading_controller     keeps the heading the brain asks for
    colour_gate_detector   camera -> /qually/gate
    qually_fsm             the brain
    qually_monitor         the dashboard
    blackbox               the rosbag recorder

When the brain exits (DONE or ABORT) everything else is shut down, and the
recorder closes its bag properly. Ctrl-C does the same.

NOTE: Ctrl-C kills the brain before it can disarm. rc_control goes neutral so
the thrusters stop, but the vehicle is left ARMED. Disarm it by hand:
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = Path(__file__).resolve().parent
CONFIG = str(HERE.parent / 'config' / 'qually.yaml')
BLACKBOX = str(HERE / 'blackbox.launch.py')


def start_everything(context):
    dry_run = LaunchConfiguration('dry_run')
    detector = LaunchConfiguration('detector')
    show_monitor = LaunchConfiguration('monitor').perform(context).lower() in (
        'true', '1', 'yes', 'on')

    # With the dashboard on screen, every other node's chatter goes to log
    # files (~/.ros/log) so it doesn't scribble over it. Nothing is lost: it
    # is all in /rosout in the bag as well. With monitor:=false you get the
    # normal scrolling log lines instead.
    quiet = {'both': 'log'} if show_monitor else 'screen'

    rc_control = Node(
        package='rc_override', executable='rc_control', name='rc_control',
        parameters=[CONFIG], output=quiet)

    heading_controller = Node(
        package='controller', executable='heading_controller', name='heading_controller',
        parameters=[CONFIG], output=quiet)

    colour_detector = Node(
        package='object_detection', executable='colour_gate_detector',
        name='colour_gate_detector',
        parameters=[CONFIG], output=quiet,
        condition=IfCondition(PythonExpression(["'", detector, "' == 'colour'"])))

    brain = Node(
        package='qually', executable='qually_fsm', name='qually_fsm',
        parameters=[
            CONFIG,
            {
                'dry_run': ParameterValue(dry_run, value_type=bool),
                # no detector running -> don't wait for one
                'require_detector': ParameterValue(
                    PythonExpression(["'", detector, "' != 'none'"]), value_type=bool),
            },
        ],
        output=quiet)

    monitor = Node(
        package='monitor', executable='qually_monitor', name='qually_monitor',
        output='screen', emulate_tty=True,
        output_format='{line}',     # no "[qually_monitor-4]" in front of every line
        condition=IfCondition(LaunchConfiguration('monitor')))

    recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(BLACKBOX),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'notes': LaunchConfiguration('notes'),
            'camera': LaunchConfiguration('camera'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('record')))

    # Brain gone -> take everything else down with it.
    stop_when_brain_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=brain,
            on_exit=[
                LogInfo(msg='[qually] brain exited, shutting everything down'),
                EmitEvent(event=Shutdown(reason='qually brain exited')),
            ]))

    return [
        rc_control,
        heading_controller,
        colour_detector,
        brain,
        monitor,
        recorder,
        stop_when_brain_exits,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='true = never arm, never change mode'),
        DeclareLaunchArgument('detector', default_value='colour',
                              description='colour | none'),
        DeclareLaunchArgument('record', default_value='true',
                              description='start the black box recorder'),
        DeclareLaunchArgument('monitor', default_value='true',
                              description='show the dashboard in this terminal'),
        DeclareLaunchArgument('name', default_value='qually',
                              description='label for the bag folder'),
        DeclareLaunchArgument('notes', default_value='',
                              description='free text saved next to the bag'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='record the colour image'),
        OpaqueFunction(function=start_everything),
    ])
