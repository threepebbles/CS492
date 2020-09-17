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

%Terminal 3
%~~~~bash
%source ./devel/setup.bash
%rosrun manip_challenge fake_grounding_publisher
%~~~~

%You can type: 60 and 61 for grasping and placing a book on a storage.
