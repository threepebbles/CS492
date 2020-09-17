#!/usr/bin/python2
import rospy
from complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--sim', action='store_true', dest='sim',
                 help='use this option if you use simulated ur5')
    opt, args = p.parse_args()

    rospy.init_node('gripper_example')

    arm = UR5ArmClient(timeout_scale=1., sim=opt.sim)
    rospy.sleep(1.0)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        ret = raw_input("Press 1 to close or press 2 to open...")
        if int(ret)==1: 
            rospy.sleep(1.0)
            arm.gripperClose()
        elif int(ret)==2:
            rospy.sleep(1.0)
            arm.gripperOpen()
        rate.sleep()
