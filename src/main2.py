#!/usr/bin/env python

import rospy
import time

from send_move_base_goal import SendMoveBaseGoalClient

# The main function of the program
if __name__ == '__main__':

    # Wait for simulator and SLAM to initialize
    print "Waiting 10 seconds for initialization"
    time.sleep(10)
    
    # Initializes the ROS node
    rospy.init_node('main_node')
    # Creates a RobotController object
    smbg = SendMoveBaseGoalClient()
    # ROS waits for events
    rospy.spin()
