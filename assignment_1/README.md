## There can be changes on repository. Please, pull the repo. and then re-build.

## After launching the GAZEBO, it will take long time to download some models. Wait about 5-10min and kill the simulator. After then relaunch the simulator again.


# Problem 1
Launch the UR5 robot with PID controllers on Terminal 1:
~~~~bash
roslaunch ur5_gazebo ur5_setup.launch hardwareInterface:=EffortJointInterface
~~~~

Keep moving the shoulder_pan_joint: 
~~~~bash
rosrun assignment_1 move_joint.py
~~~~

Then, observe the joint position using rqt_plot
~~~~bash
rqt_plot rqt_plot /joint_states/position[3]
~~~~
The graph window may pop up behind your simulator screen or other programs. Note that '3' is the index of the shoulder_pan_joint in the joint_state position vector. 

You can set/get the P- or D-gain parameters in ROS Parameter Server using a command line tool.
~~~~bash
rosparam get /trajectory_controller/gains/shoulder_pan_joint/p
rosparam set /trajectory_controller/gains/shoulder_pan_joint/p 1000
rosparam get /trajectory_controller/gains/shoulder_pan_joint/d
rosparam set /trajectory_controller/gains/shoulder_pan_joint/d 10
~~~~



