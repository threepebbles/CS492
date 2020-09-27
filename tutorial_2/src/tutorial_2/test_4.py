#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose

if __name__ == '__main__':
    rospy.init_node("test_4")
    rospy.sleep(1)

    pub = rospy.Publisher('test_goal', PoseArray, queue_size=10)
    msg = PoseArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'base_link'


    z_list = np.linspace(0.4,1.2,20)
    y_list = np.linspace(-0.4,0.4,20)
    
    for i in range(len(y_list)):
        pose = Pose()
        pose.position.x = 0.9
        pose.position.y = y_list[i]
        pose.position.z = z_list[i]
        pose.orientation.x = 0.
        pose.orientation.y = 0.
        pose.orientation.z = 0.
        pose.orientation.w = 1.
        msg.poses.append(pose)

    

    rospy.loginfo("Running ... ")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
    
