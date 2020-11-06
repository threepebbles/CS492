#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String



if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)


    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/gripper_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue

        print trans, rot

