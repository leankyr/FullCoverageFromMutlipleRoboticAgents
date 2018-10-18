#!/usr/bin/env python

import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped


#def my_map_callback(msg):
#    return

def my_pose_callback(msg):
    print "I am in callback!!"
    header = msg.header
    x = msg.polygon.points[4].x
    print x
    return 

if __name__ == '__main__':
    print 'whatever'
    rospy.init_node('target_select',anonymous=True)
    rospy.Subscriber("/move_base/global_costmap/footprint", \
            PolygonStamped, my_pose_callback)
    print "In the end"
    rospy.spin()
