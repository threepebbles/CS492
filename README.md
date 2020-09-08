# Tutorial 1

## Pre-requites for this tutorial
Please, install Ubuntu 18.04 and ROS Melodic.

## Installation of your project repository
~~~~bash
$ source /opt/ros/melodic/setup.sh
~~~~

Move to anyfolder you want to place the class code. Then, you can create the workspace,
~~~~bash
mkdir -p catkin_ws/src
cd catkin_ws/src/
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


# Message passing using
Let's test topic publisher and listener! First, check if you have the two files by changing your working directory:

~~~~bash
roscd cs492_IIR/src/cs492_IIR
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

