# Tutorial 3

Run the following commands from two terminals:
~~~~bash
roslaunch manip_challenge ur5_setup.launch
roslaunch manip_challenge ur5_robotiq_2f85.launch
~~~~

Then, please complete following tasks!!
## Communication
1. Subscribe current joint angles.
2. Subscribe the transformation from 'world' frame to 'gripper_link' frame.
3. Subscribe the task command after running:
~~~~bash
rosrun manip_challenge item_list_pub.py
~~~~
4. Query the pose of 'eraser'

## Planning
1. Visualize the object and gripper poses in your RViZ
2. Calculate the grasping pose 
3. Create a list of waypoints for grasping

## Execution
1. Open/close your fingers
2. Grasp the 'eraser'!

Repeat the manual planning and execution for placing, too!!!


## Today's attendance quiz!
1. Place the 'eraser' on the right storage
2. Grasp the side of the 'glue'
3. (Option) apply A*
