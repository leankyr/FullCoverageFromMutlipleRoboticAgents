#!/usr/bin/env python

import rospy
import time

from send_move_base_goal_two_robots import SendMoveBaseGoalClient

# The main function of the program
if __name__ == '__main__':

    # Wait for simulator and SLAM to initialize
    print "Waiting 5 seconds for initialization"
    time.sleep(5)
    
    # Initializes the ROS node
    rospy.init_node('main_node')
    # Creates a RobotController object
    smbg = SendMoveBaseGoalClient()
    # ROS waits for events
    rospy.spin()
