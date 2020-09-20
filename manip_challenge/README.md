# manipulation_challenge

# How to test the pick_and_place system?

Terminal 1
~~~~bash
source ./devel/setup.bash
roslaunch manip_challenge ur5_setup.launch
~~~~
If this is your first trial, it may take time. 

Terminal 2
~~~~bash
source ./devel/setup.bash
roslaunch manip_challenge ur5_robotiq_2f85.launch
~~~~

Terminal 3
~~~~bash
source ./devel/setup.bash
rosrun manip_challenge example.py
~~~~
You will be able to see how to send joint command and also close/open the gripper.
