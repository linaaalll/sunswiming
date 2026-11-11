# sunswiming

> The whole point of the new structure to keep everything trackable

The structure top down should consists of 

**ros2_ws** on the same level as **computer_vision** <-(contains the current newest cv model)
every build and every source happens within the ros2_ws

## ros2_ws

This contains all of the packages where each of the packages will be responsible for a set of uses and tasks.

---
1. zed ros2 wrapper (once setup don't touch)
2. mavros_msgs (once setup don't touch)
3. utils: controller.cpp, kp_depth.cpp, a set of helpful nodes that will assist the rov to do a series of tasks
4. output_msgs: a helpful interface that will be responsible for reading the data and making it easy to view during testing, e.g. current state of the rov in the task, current sensor value (e.g. pressure and depth and how far to go still), current velocity value for all 8 thrusters
5. qually: everything to do with qually, qual_task: high-level robot brain logic, qual_gate_detector.py: responsible for detecting the qual_gate and assist high-level brain to do stuff
6. tasks: other tasks in sauvc, such as gripper, buckets, flare, orange_flare_avoid.py: responsible for detecting the orange_flare to assist high-level brain in avoiding it...
7. brain: either brain.cpp or bt implementation
8. localisation: everything to do with localisation, vslam, visual_odom

the thing is still how to keep everything input and output easily trackable, so that it will be easy for simulation

Huge boomboclat and the stuff needs to do to run and operate stuff:
Lina likes to operate things like how it should be going during comp, therefore the whole instruction is just tetherless.
1. When decide to run ROV through wifi, which is the case in comp, enter rov through wifi ip address (reminder for myself my phone ip address is 172.20.10.8)
2. if in doubt, check existence by ~ping -c 4 [the ip address]
3. ssh into pi via ~ssh pi@[the ip address]
4. ssh into jetson via pi ~ssh jetson@192.168.1.69 (this can be checked through mavlink endpoint in blueos)

The unholy way to get another jetson terminal up and running ehhhhhhh
ssh -J pi@172.20.10.8 jetson@192.168.1.69

How to get into blueos, ehhhh several ways
1. through the current web ip if connected to phone address (for me its 172.20.10.8)

How to actually have fun in the boomboclat repo
1. update via this shitass comment after git push everything from local 
rsync -av -e "ssh -J pi@172.20.10.8" --exclude '/ros2_ws/build' --exclude '/ros2_ws/install' --exclude '/ros2_ws/log' --exclude '/ros2_ws/src/mavros' --exclude '/ros2_ws/src/mavlink-gbp-release' ~/sunswiming/ jetson@192.168.1.69:~/lina/sunswiming/

2. 