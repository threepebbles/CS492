#!/usr/bin/env python  
import os, sys
import json, simplejson
import rospy, rospkg

from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('grounding_node_temp')

    print "----------------------------------------------"
    rospy.sleep(1.0)
    pub = rospy.Publisher('/task_commands', String, queue_size=10, latch=True)
    rospy.sleep(1.0)


    # d = {"storage_left": ['book', 'eraser', 'soap2'],
    #     "storage_right": ['snacks', 'biscuits', 'glue', 'soap'] }
    # d = {"storage_left": ['snacks', 'biscuits', 'soap2'],
    #     "storage_right": ['book', 'glue', 'eraser', 'soap'] }
    d = {"storage_left": ['snacks'],
        "storage_right": ['book'] }
    print("tsk: ", d)
    pub.publish(json.dumps(d, encoding='ascii'))
    rospy.sleep(1.0)
    rospy.spin()
