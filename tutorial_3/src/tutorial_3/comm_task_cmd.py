#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json

def command_callback(msg):
    """ 
    A callback function that subscribes ?.

    Parameters
    ----------
    msg : ?
    """
    d = json.loads(msg.data)
    print d
    print d['storage_left']


if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)


    # Confirm the list of topics: rostopic list

    # How can we subscribe the task command?
    # 1. Run the task command script: rosrun manip_challenge item_list_pub.py
    # 2. Subscribe the topic via your python script

    rospy.Subscriber('/task_commands',
                         String,
                         command_callback)


    rospy.spin()
