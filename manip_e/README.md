# manipulation_challenge 20204329

## Timelapse challenge  
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
~~~~bash
source ./devel/setup.bash
roslaunch manip_e timelapse_challenge.launch
~~~~

-----

## Intelligence challenge
Terminal 1  
~~~~bash
source ./devel/setup.bash
roslaunch manip_e ur5_setup.launch
~~~~

Terminal 2  
~~~~bash
source ./devel/setup.bash
roslaunch manip_e ur5_robotiq_2f85.launch
~~~~

Terminal 3  
It will be task commands
~~~~bash
source ./devel/setup.bash
rosrun manip_e item_list_pub.py
~~~~

Terminal 4 
~~~~bash
source ./devel/setup.bash
roslaunch manip_e intelligence_challenge.launch