CS492: Introduction to Intelligent Robotics

# Installation
## Pre-requites for this tutorial
Please, install Ubuntu 18.04 and ROS Melodic with dependencies:
~~~~bash
sudo apt-get install ros-melodic-trac-ik ros-melodic-trac-ik-python ros-melodic-moveit-ros ros-melodic-moveit-planners* ros-melodic-moveit-ros-planning* ros-melodic-moveit-ros-move-group ros-melodic-moveit-ros-control-interface ros-melodic-moveit-kinematics ros-melodic-industrial-msgs ros-melodic-moveit-kinematics ros-melodic-ddynamic-reconfigure ros-melodic-gazebo-plugins ros-melodic-rqt-py-trees ros-melodic-py-trees* ros-melodic-gripper-action-controller ros-melodic-rqt-joint-trajectory-controller ros-melodic-joint-trajectory-controller python-catkin-tools python-pyassimp ros-melodic-soem ros-melodic-effort-controllers -y
~~~~

## Installation of your project repository
~~~~bash
source /opt/ros/melodic/setup.sh
~~~~

Move to anyfolder you want to place the class code. Then, you can create the workspace,
~~~~bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
catkin_init_workspace
~~~~

Let's copy the the class repo, install dependencies, and build it!
~~~~bash
git clone https://github.com/rirolab/cs492_IIR.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
sudo pip install -r requirements.txt
catkin_make
~~~~
If you want catkin_build, you can use it instead of catkin_make.

Lastly, load the new environmental variables and also add to path the environmental setup to your bashrc
~~~~bash
source ./devel/setup.bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
~~~~

You can verify your ROS install by confirming the environmental variables
~~~~bash
env| grep ROS
~~~~

Make sure that if ROS_MASTER_URI and ROS_ROOT, ETC are setup. 


# Links 
- [Tutorial I](tutorial_1/README.md)
- [Tutorial II](tutorial_2/README.md)
- [Tutorial III](tutorial_3/README.md)
- [Tutorial IV](tutorial_4/README.md)


- [Assignment I](assignment_1/README.md)
- [Assignment II](assignment_2/README.md)
- [Assignment III]()

- [Manipulation Challenge](manip_challenge/README.md)


# ETC
There are many useful command-line tools like rostopic, rqt_graph, rosbag, etc. Please, see http://wiki.ros.org/ROS/CommandLineTools






