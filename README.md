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

Need to do in lab with Betty:
- 
