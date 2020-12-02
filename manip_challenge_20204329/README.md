# manipulation_challenge 20204329

Terminal 1  
~~~~bash
source ./devel/setup.bash
roslaunch manip_challenge ur5_setup.launch
~~~~

Terminal 2  
~~~~bash
source ./devel/setup.bash
roslaunch manip_challenge ur5_robotiq_2f85.launch
~~~~

Terminal 3  
It will be task commands
~~~~bash
source ./devel/setup.bash
rosrun manip_challenge item_list_pub.py
~~~~

Terminal 4  
pick and place
~~~~bash
source ./devel/setup.bash
roslaunch manip_challenge_20204329 multithread_timelapse_challenge.launch
~~~~
