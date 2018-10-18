#!/usr/bin/env python

import sys
import rospy
import actionlib
import random 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now() 

    print x
    print y

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y 
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    x = int(sys.argv[1])
    y = int(sys.argv[2]) 
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client(x,y)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
