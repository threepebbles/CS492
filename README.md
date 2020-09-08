# Tutorial I

## Pre-requites for this tutorial
Please, install Ubuntu 18.04 and ROS Melodic.

## Installation of your project repository
~~~~bash
$ source /opt/ros/melodic/setup.sh
~~~~

Move to anyfolder you want to place the class code. Then, you can create the workspace,
~~~~bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
catkin_init_workspace
~~~~

Let's copy the the class repo and build it!
~~~~bash
git clone https://github.com/rirolab/cs492_IIR.git
cd ..
catkin_make
~~~~
If you want catkin_build, you can use it instead of catkin_make.

Lastly, load the new environmental variables and also add to path the environmental setup to your bashrc
~~~~bash
source ./devel/setup.bash
echo 'source devel/setup.bash' >> ~/.bashrc
~~~~

You can verify your ROS install by confirming the environmental variables
~~~~bash
env| grep ROS
~~~~

Make sure that if ROS_MASTER_URI and ROS_ROOT, ETC are setup. 


# Practice 1: Message publication and subscription
Let's test topic publisher and listener! First, check if you have the two files by changing your working directory:

~~~~bash
roscd tutorial_1/src/tutorial_1
~~~~
Note that 'roscd' works after loading the ROS' environment setup.

Please, open the publisher file 
~~~~bash
gedit talker.py
~~~~
You will be able to see there is a publisher that send a string format of message via 'chatter' topic.

Then, let's open a subscriber file 
~~~~bash
gedit listener.py
~~~~
This file subscribes the string format of messages via 'chatter' topic and pringout the contents using the loginfo function.

Please, open three terminals and then run one roscore and the talker&listener nodes.

From terminal 1
~~~~bash
roscore
~~~~
Terminal 2: the talker node will publish a message via chatter topic.
~~~~bash
rosrun tutorial_1 talker.py
~~~~
Terminal 3: the listener node will subscribe and printout the message. 
~~~~bash
roscore tutorial_1 listener.py
~~~~


# Practice 2: Turtlebot3
Let's first install Turtlebot3, a mobile robot, which can run on a physics based simulator, GAZEBO!
~~~~bash
sudo apt install ros-melodic-turtlebot3 ros-melodic-turtlebot3-gazebo
~~~~

Select a robot name and launch the robot simulator. 
~~~~bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
~~~~

Run a keyboard-based controller
~~~~bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
~~~~

You can now control the robot using keyboard and also look at the communication between nodes. 
Please, check following tutorial for more interesting scenarios! 
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/


# Practice 3: Create your own package
You can create a package to run your own nodes
~~~~bash
cd ~/catkin_ws/src/
catkin_create_pkg cs492_practice3 std_msgs rospy roscpp
~~~~
This command creates a package named cs492_practice3 that lists std_msgs, rospy, and roscpp as dependencies. Then you can build your catkin workspace
~~~~bash
cd ~/catkin_ws
catkin_make
~~~~

After then, you should source
~~~~bash
source ~/catkin_ws/devel/setup.bash
~~~~

Verify your package ahs installed by moving to the package folder
~~~~bash
roscd cs492_practice3
~~~~

# ETC
There are many useful command-line tools like rostopic, rqt_graph, rosbag, etc. Please, see http://wiki.ros.org/ROS/CommandLineTools






