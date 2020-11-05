#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node("test_3")
    rospy.sleep(1)

    pub = rospy.Publisher('test_goal', PoseStamped, queue_size=10)
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x = 0.9
    ps.pose.position.y = 0.23
    ps.pose.position.z = 0.9
    ps.pose.orientation.x = 0.
    ps.pose.orientation.y = 0.
    ps.pose.orientation.z = 0.
    ps.pose.orientation.w = 1.
    
    
    rospy.loginfo("Running ... ")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        ps.pose.position.x += 0.005
        pub.publish(ps)
        rate.sleep()
    

    
