# Tutorial II
There can be changes on this repository. Please, pull and re-build this repository by using
~~~~bash
catkin clean
catkin_make
source ./devel/setup.bash
~~~~

## Practice 1: Command-line tools
### roslaunch
~~~~bash
roslaunch manip_challenge ur5_setup.launch
~~~~

### rosrun
This command is used for running executable files in a package.
~~~~bash
rosrun tutorial_2 test_1.py
~~~~
Please, make sure that our test_1.py is executable.

### rosparam
This command is used for setting user-defined parameters on the ROS parameter servier:
~~~~bash
rosparam set /user_param 1111
rosparam get /user_param 
~~~~
You can also set/get parameters using a python file
~~~~bash
rosrun tutorial_2 test_2.py
~~~~
Please, check the contents in the python file.



## Practice 2: RViZ and rqt
### Visualize the UR5 robot in RViZ
You can run a 3D visualizer:
~~~~bash
rosrun rviz rviz
~~~~
You can visualize any pose information by publishing and subscribing a Pose message:
~~~~bash
rosrun tutorial_2 test_3.py
~~~~
This means you can visualize a sequence of pose trajectory, too.
~~~~bash
rosrun tutorial_2 test_4.py
~~~~

### Use rqt and plugins
You can adjust the parameters of the controllers using the Dynamic Rconfigure plugin. You can find it from
~~~~bash
Plugins >> Configuration >> Dynamic Reconfigure
~~~~
You can also manually move joints using "Joint trajectory controller". You can find it from
~~~~bash
Plugins >> Robot Tools >> Joint trajectory controller
~~~~

## Practice 3: Gazebo

