#!/usr/bin/env python
import rospy

if __name__ == '__main__':
    rospy.init_node("test_1")


    
    rospy.set_param("is_test", False)
    print rospy.get_param("is_test")
    
