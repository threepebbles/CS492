#!/usr/bin/env python
import rospy
from control_msgs.msg import *

def state_callback(msg):
    """ 
    A callback function that subscribes joint states.

    Parameters
    ----------
    msg : control_msgs/JointTrajectoryControllerState
    a message of desired, actual, and error state of joints
    """
    js_joint_name     = msg.joint_names
    js_joint_position = list(msg.desired.positions)
    print js_joint_position


if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)


    # Confirm the list of topics: rostopic list

    # How can we get the values for the current joint angles?
    # 1. Check the information of a topic: rostopic info /joint_state
    # 2. Subscribe the topic: rostopic echo /joint_states
    # 3. You can subscribe the topic via a python script

    rospy.Subscriber('/trajectory_controller/state',
                         JointTrajectoryControllerState,
                         state_callback)


    rospy.spin()
