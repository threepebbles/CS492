#!/usr/bin/env python
import rospy

if __name__ == '__main__':
    rospy.init_node("test_1")

    rospy.loginfo("This is loginfo!")
    rospy.logwarn("This is warning!")
    rospy.logerr("This is error!")
    print "This is python printout!"
