## There can be changes on this repository. Please, pull and re-build this repository.

Install new Python dependencies
~~~~bash
cd "your catkin workspace"/src/cs492_IIR/assignment_2
sudo pip install -r requirements.txt
~~~~

Then, update dependencies using rosinstall
~~~~bash
cd "your catkin workspace"/src
wstool init .
wstool merge cs492_IIR/dependencies.rosinstall
wstool update

cd ..
source /opt/ros/melodic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
catkin_make 
~~~~

## Problem 1 and 2
Launch the UR5 robot:
~~~~bash
roslaunch ur5_gazebo ur5_setup.launch
~~~~
Please, ignore any error message like "No p gain" or "Error in REST request".

Then, run executable python files. For example,
~~~~bash
rosrun assignment_2 min_jerk.py
rosrun assignment_2 move_joint.py
...
~~~~

For Problem 2. E, please run following before running your solution
~~~~bash
rosrun assignment_2 add_object.py
~~~~
