#!/usr/bin/env python

import rospy
import time
### Remeber to get it back to original form if it does not work
#from send_move_base_goal_two_robots import SendMoveBaseGoalClient
from send_goal_robot1 import SendMoveBaseGoalClient1
from send_goal_robot2 import SendMoveBaseGoalClient2

# The main function of the program
if __name__ == '__main__':

    # Wait for simulator and SLAM to initialize
    print "Waiting 5 seconds for initialization"
    time.sleep(5)
    
    # Initializes the ROS node
    rospy.init_node('main_node')
    # Creates a RobotController object
    smbg1 = SendMoveBaseGoalClient1()
    smbg2 = SendMoveBaseGoalClient2()
    # ROS waits for events
    rospy.spin()
