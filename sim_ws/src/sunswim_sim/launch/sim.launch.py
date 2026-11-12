"""
The simulator, one command. It replaces "the robot": Gazebo + the vehicle +
fake mavros and camera topics.

TELEOP (drive it yourself with the keyboard):

    terminal 1:  ros2 launch ~/sunswiming/sim_ws/src/sunswim_sim/launch/sim.launch.py
    terminal 2:  ros2 run teleop_twist_keyboard teleop_twist_keyboard

    Click terminal 2 so it has the keyboard. i = forward, j / l = turn,
    t / b = up / down, k = stop. Shift + j / l = slide sideways.

AUTONOMOUS (the brain drives, same stack as the real robot):

    terminal 1:  ros2 launch ~/sunswiming/sim_ws/src/sunswim_sim/launch/sim.launch.py mode:=auto
    terminal 2:  ros2 launch ~/sunswiming/launch_file/launch/qually.launch.py dry_run:=false

    dry_run:=false is safe here, the "arming" is fake. Terminal 2 is exactly
    the command you'd run on the Jetson.

    Do NOT run the keyboard teleop in auto mode: it publishes /cmd_vel too
    and would fight the heading controller.

Options:
    mode:=teleop|auto      who drives (default teleop)
    gui:=false             no Gazebo window (faster, for quick tests)
    murk:=1.0              murky water on the camera (water_filter.py). 0 = clear
                           (default), 1 = normal murk, 2 = twice as murky.
    weak:=random           one horizontal thruster is weak (or weak:=FL / FR / RL / RR).
                           The driver prints which one when it starts.
    weak_factor:=0.7       how weak: 0.7 = it gives 70% of its thrust
    yaw_hold:=false        turn off rov_driver's own yaw hold, so only your
                           heading PID steers. Use it with weak:= in auto mode,
                           otherwise the driver hides the fault for you.
    x:=-11 y:=0 z:=1 yaw:=0   where the vehicle spawns. The gate is at x = -2.5,
                              the surface at z = 2, so z:=1 is 1 m deep.

How the two modes are wired:

    teleop:  keyboard -> /cmd_vel -----------------------------------> rov_driver -> Gazebo

    auto:    brain -> heading_controller -> /cmd_vel -> rc_control
                  -> /mavros/rc/override -> sim_bridge -> /sim/cmd_vel -> rov_driver -> Gazebo

In both modes sim_bridge publishes the fake sensors (/mavros/imu/data, depth,
camera), so the dashboard and the gate detector work while you drive by hand:

    ros2 run monitor qually_monitor
    ros2 run object_detection colour_gate_detector
"""

import os
import subprocess
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration

SIM = Path(__file__).resolve().parent.parent                 # .../sunswim_sim
REPO = SIM.parent.parent.parent                              # .../sunswiming
WORLD = SIM / 'worlds' / 'pool_qual.sdf'
MODELS = SIM / 'models'
SCRIPTS = SIM / 'scripts'
XACRO = REPO / 'ros2_ws' / 'src' / 'robot_description' / 'urdf' / 'robot.urdf.xacro'


def start_sim(context):
    mode = LaunchConfiguration('mode').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes', 'on')
    if mode not in ('teleop', 'auto'):
        raise RuntimeError(f"mode must be teleop or auto, got '{mode}'")
    auto = mode == 'auto'

    # 1. Turn the xacro into a plain URDF file Gazebo can load.
    urdf = subprocess.run(['xacro', str(XACRO)], capture_output=True, text=True, check=True).stdout
    urdf_file = Path(tempfile.gettempdir()) / 'sunswim_sim.urdf'
    urdf_file.write_text(urdf)

    # 2. Gazebo itself. -r = start running straight away.
    #    Without the gui: -s = server only, rendering the camera off-screen.
    gz_cmd = ['gz', 'sim', '-r', str(WORLD)]
    if not gui:
        gz_cmd = ['gz', 'sim', '-r', '-s', '--headless-rendering', str(WORLD)]
    gazebo = ExecuteProcess(cmd=gz_cmd, output='screen')

    # 3. Drop the vehicle into the world (a few seconds later, Gazebo needs
    #    time to come up).
    spawn = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-world', 'pool_qual', '-name', 'sunswim', '-file', str(urdf_file),
             '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
             '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
        output='screen')

    # 4. rov_driver: Twist in, thruster forces out.
    #    teleop: it listens on /cmd_vel, where the keyboard publishes.
    #    auto:   /cmd_vel belongs to the real stack (it feeds rc_control), so
    #            rov_driver is moved to /sim/cmd_vel, which sim_bridge feeds.
    driver_cmd = ['python3', str(SCRIPTS / 'rov_driver.py'),
                  '--weak', LaunchConfiguration('weak').perform(context),
                  '--weak-factor', LaunchConfiguration('weak_factor').perform(context)]
    if LaunchConfiguration('yaw_hold').perform(context).lower() not in ('true', '1', 'yes', 'on'):
        driver_cmd.append('--no-yaw-hold')
    if auto:
        driver_cmd += ['--ros-args', '-r', '/cmd_vel:=/sim/cmd_vel']
    driver = ExecuteProcess(cmd=driver_cmd, output='screen')

    # 5. sim_bridge: the fake mavros + camera. --drive only in auto.
    bridge_cmd = ['python3', str(SCRIPTS / 'sim_bridge.py')]
    if auto:
        bridge_cmd.append('--drive')

    # 6. Murky water (optional): water_filter.py tints the camera picture, and
    #    sim_bridge then passes on the tinted one instead of the clean one.
    murk = float(LaunchConfiguration('murk').perform(context))
    extras = []
    if murk > 0.0:
        bridge_cmd += ['--image-topic', '/cam/image_water']
        extras.append(ExecuteProcess(
            cmd=['python3', str(SCRIPTS / 'water_filter.py'), '--murk', str(murk)],
            output='screen'))
    bridge = ExecuteProcess(cmd=bridge_cmd, output='screen')

    if auto:
        hint = ('[sim] AUTO. In another terminal:  ros2 launch '
                '~/sunswiming/launch_file/launch/qually.launch.py dry_run:=false')
    else:
        hint = ('[sim] TELEOP. In another terminal:  '
                'ros2 run teleop_twist_keyboard teleop_twist_keyboard')

    return [
        gazebo,
        TimerAction(period=5.0, actions=[spawn, driver, bridge, *extras, LogInfo(msg=hint)]),
    ]


def generate_launch_description():
    # Tell Gazebo where model://pool and model://sauvc_qual_gate live.
    resource_path = str(MODELS) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='teleop', description='teleop | auto'),
        DeclareLaunchArgument('gui', default_value='true', description='show the Gazebo window'),
        DeclareLaunchArgument('murk', default_value='0.0', description='0 = clear water'),
        DeclareLaunchArgument('weak', default_value='', description='random | FL | FR | RL | RR'),
        DeclareLaunchArgument('weak_factor', default_value='0.7'),
        DeclareLaunchArgument('yaw_hold', default_value='true'),
        DeclareLaunchArgument('x', default_value='-11.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        OpaqueFunction(function=start_sim),
    ])
