#!/usr/bin/env python
import rospy

if __name__ == '__main__':
    rospy.init_node("test_1")

    rospy.loginfo("This is log-information!")
    rospy.logwarn("This is log-warning!")
    rospy.logerr("This is log-error!")
    print "These are ROS printouts!"
