#!/usr/bin/env python2.7

import rospy
import time
from send_move_base_goal import SendMoveBaseGoalClient


if __name__ == '__main__':
    rospy.init_node('main_node')
    smbg = SendMoveBaseGoalClient()
rospy.spin()
