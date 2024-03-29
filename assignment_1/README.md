## There can be changes on this repository. Please, pull and re-build this repository.

## After launching the GAZEBO, it may hang for a while to download some models. Please, wait 5-10min and kill the simulator once. After then relaunch the simulator again.


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

<p align="center" width="100%">
    Screenshot of the problem1 robot posture.<br>
    <img width="50%" src="docs/problem1.png"> 
</p>
<!-- ![Alt text](docs/problem1.png?raw=true "Screenshot of the problem1 robot posture") -->


# Problem 2 and 3
Launch the UR5 robot with PID controllers on Terminal 1:
~~~~bash
roslaunch ur5_gazebo ur5_setup.launch 
~~~~
Please, ignore any error message like "No p gain" or "Error in REST request".


# FAQ
If you cannot see the same posture as the above picture for the problem 1, please install dependencies in the README.md of https://github.com/rirolab/cs492_IIR
