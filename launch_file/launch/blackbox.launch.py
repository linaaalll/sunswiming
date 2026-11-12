"""
Black box recorder: records the robot's data into a rosbag so a run can be
replayed and analysed afterwards.

Run it on its own (no build needed, ros2 launch takes a file path):

    ros2 launch ~/lina/sunswiming/launch_file/launch/blackbox.launch.py name:=pool_gate_01

Options:
    name:=pool_gate_01     label for this run (goes in the folder name)
    notes:="what I tested" free text saved next to the bag
    camera:=false          skip the colour image (bags get ~100x smaller)
    depth:=true            also record the depth image (big!)

Stop it with Ctrl-C, the bag is closed properly.

Every run makes one folder:

    bags/<date>_<time>_<name>/
        info.txt    which git commit was running, when, your notes
        bag/        the actual rosbag

To make it start automatically with the brain, put this in the brain's
launch file (then launching the brain also starts the recorder):

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource('<path to>/blackbox.launch.py'),
        launch_arguments={'name': 'qually'}.items(),
    )
"""

import subprocess
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


# ---- what gets recorded ---------------------------------------------------
# A topic that isn't being published yet is fine: the recorder keeps looking
# and picks it up when it appears.

# Flight controller (ArduSub on the pi), through mavros. All small.
MAVROS_TOPICS = [
    '/mavros/state',                         # armed? mode? connected?
    '/mavros/statustext/recv',               # ArduSub's own warnings / errors
    '/mavros/imu/data',                      # orientation, gyro, accel
    '/mavros/imu/static_pressure',           # pressure -> depth
    '/mavros/global_position/rel_alt',       # depth as ArduSub sees it
    '/mavros/local_position/pose',           # ArduSub's position estimate
    '/mavros/local_position/velocity_body',  # ArduSub's velocity estimate
    '/mavros/vfr_hud',                       # heading, throttle, climb rate
    '/mavros/battery',                       # voltage, current
    '/mavros/rc/in',                         # commands ArduSub received
    '/mavros/rc/override',                   # commands OUR code sent
    '/mavros/rc/out',                        # what each thruster was told to do
]

# ZED camera, the small stuff (no images).
ZED_TOPICS = [
    '/zed/zed_node/odom',                    # camera's motion estimate
    '/zed/zed_node/pose',                    # camera's position estimate
    '/zed/zed_node/pose/status',             # is tracking ok or lost?
    '/zed/zed_node/imu/data',                # camera's own imu
    '/zed/zed_node/obj_det/objects',         # what it detected, and where
    '/zed/zed_node/status/health',           # camera health
]

# Colour image, jpeg compressed. This is most of the bag's size.
CAMERA_TOPICS = [
    '/zed/zed_node/rgb/color/rect/image/compressed',
    '/zed/zed_node/rgb/color/rect/camera_info',
]

# Depth image. Only with depth:=true.
DEPTH_TOPICS = [
    '/zed/zed_node/depth/depth_registered/compressedDepth',
    '/zed/zed_node/depth/camera_info',
]

# ROS plumbing. /rosout is every log line from every node, always keep it.
SYSTEM_TOPICS = [
    '/rosout',
    '/tf',
    '/tf_static',
    '/diagnostics',
]

# YOUR nodes. Add every topic the brain / controller / tasks publish, this is
# what explains "why did it do that".
MY_TOPICS = [
    '/qually/state',                         # which FSM state, every tick
    '/qually/gate',                          # what the detector saw
    '/controller/target',                    # what the brain asked for
    '/controller/debug',                     # PID: target, error, P/I/D, output
    '/cmd_vel',                              # what went to rc_control
    # '/qually/gate/debug_image/compressed', # detector overlay (a second video)
]


def find_repo():
    """Walk up from this file until we hit the folder that has .git in it."""
    for folder in Path(__file__).resolve().parents:
        if (folder / '.git').exists():
            return folder
    return None


def git(repo, *args):
    """Run a git command in the repo, return its output ('' if it fails)."""
    try:
        result = subprocess.run(['git', '-C', str(repo), *args],
                                capture_output=True, text=True, timeout=5)
        return result.stdout.rstrip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def is_true(text):
    return text.lower() in ('true', '1', 'yes', 'on')


def start_recording(context):
    name = LaunchConfiguration('name').perform(context)
    notes = LaunchConfiguration('notes').perform(context)
    camera = is_true(LaunchConfiguration('camera').perform(context))
    depth = is_true(LaunchConfiguration('depth').perform(context))

    repo = find_repo()
    bags_root = (repo / 'bags') if repo else (Path.home() / 'sunswim_bags')

    stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    run_dir = bags_root / f'{stamp}_{name}'
    run_dir.mkdir(parents=True, exist_ok=False)

    topics = MAVROS_TOPICS + ZED_TOPICS + SYSTEM_TOPICS + MY_TOPICS
    if camera:
        topics = topics + CAMERA_TOPICS
    if depth:
        topics = topics + DEPTH_TOPICS

    # info.txt: ties this bag to the exact code that was running.
    commit = git(repo, 'rev-parse', 'HEAD') if repo else ''
    uncommitted = git(repo, 'status', '--short') if repo else ''
    lines = [
        f'run:      {name}',
        f'started:  {stamp}  (jetson clock, may be wrong)',
        f'notes:    {notes}',
        f'commit:   {commit or "unknown"}',
        f'branch:   {git(repo, "rev-parse", "--abbrev-ref", "HEAD") if repo else "unknown"}',
        f'clean:    {"yes" if not uncommitted else "NO, uncommitted changes below"}',
        f'camera:   {camera}',
        f'depth:    {depth}',
        '',
        'uncommitted changes:',
        uncommitted or '  (none)',
        '',
        'topics:',
        *[f'  {topic}' for topic in topics],
        '',
    ]
    (run_dir / 'info.txt').write_text('\n'.join(lines))

    print(f'\n[blackbox] recording {len(topics)} topics into {run_dir}\n')

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', str(run_dir / 'bag'), *topics],
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='run',
                              description='label for this run'),
        DeclareLaunchArgument('notes', default_value='',
                              description='free text saved in info.txt'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='record the colour image'),
        DeclareLaunchArgument('depth', default_value='false',
                              description='record the depth image (big)'),
        OpaqueFunction(function=start_recording),
    ])
