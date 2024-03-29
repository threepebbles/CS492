# Tutorial 4

## Pre-requites for this tutorial
First, install new dependencies,
~~~~bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
sudo apt-get install ros-melodic-realsense2-description
cd "tutorial_4 folder"
sudo pip install -r requirements.txt
~~~~

Then, clone a camera-related gazebo plugin repository
~~~~bash
cd src/util
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
~~~~

Finally, recompile all and source the environmental setup file
~~~~bash
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
~~~~

## Test
Run the following commands from two terminals:
~~~~bash
roslaunch manip_challenge ur5_setup.launch
roslaunch manip_challenge ur5_robotiq_2f85.launch
~~~~

### Collision Check
1. Visualize your collision meshes/boxes
2. Check collision between the robot and objects
~~~~bash
roslaunch tutorial_4 collision_check.launch
~~~~

### Vision
1. Load a Gazebo vision plugin
2. Visualize it via RViZ
~~~~bash
rosrun rviz rviz
~~~~

### Today's attendance quiz!
1. Add a new object in Gazebo. Place your arm around the object (virtually overlapping it!) and run the collision checker. Please, capture Gazebo, RViZ screens, and the terminal printout.
~~~~bash
roslaunch manip_challenge ur5_setup.launch
"Please, add your object"
roslaunch manip_challenge ur5_robotiq_2f85.launch
"Please, run the collision checker"
~~~~


2. Randomly place objects. Then, capture the Gazebo screen as well as the point cloud of the environment.

