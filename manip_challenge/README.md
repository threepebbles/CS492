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
rosrun manip_challenge example1.py
~~~~
You will be able to see how to send joint command and also close/open the gripper.



# How to send a command?
The instructor/TA will send a string that contains a dictionary format of commands via a topic "/task_commands":
~~~~bash
    d = {"storage_left": ['book', 'eraser', 'soap2'],
        "storage_right": ['snacks', 'biscuits', 'glue', 'soap'] }
~~~~
This dictionary represents the final locations of objects on the front table. Your robot should subscribe the topic and pick-and-place the items on the right locations. You can test an example command using following script:
~~~~bash
source ./devel/setup.bash
rosrun manip_challenge item_list_pub.py
~~~~


# How to query the pose of object in the world model?
~~~~bash
source ./devel/setup.bash
rosrun manip_challenge example2.py
~~~~
