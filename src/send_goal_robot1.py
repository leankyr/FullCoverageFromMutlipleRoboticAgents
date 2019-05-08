#!/usr/bin/env python

import rospy
import actionlib
import time
import math
import random
import tf 
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from subscriber_node_two_robots import SubscriberNode
#from target_selection_class import TargetSelect #,selectRandomTarget
#from cov_based_target_select import TargetSelect
from topo_graph_target_select1 import TargetSelect
#from two_robots_cov_based_target_select import TargetSelect




class SendMoveBaseGoalClient1:

    def __init__(self):
        self.subNode = SubscriberNode()
        self.selectTarget = TargetSelect()
        self.moveBaseGoal1 = MoveBaseGoal()
        
        self.moveBaseGoal1.target_pose.header.frame_id = "map"
        self.moveBaseGoal1.target_pose.header.stamp = rospy.Time.now()

        rospy.loginfo("[Main Node] Wait 20 seconds for the subscribers to be ready!")
        time.sleep(20)

#        self.velocityPub = rospy.Publisher(rospy.get_param('velocity_pub1'), Twist, queue_size = 1)
#        self.rotateRobot()

        rospy.Timer(rospy.Duration(1.0), self.calculateSendGoal)


    def calculateSendGoal(self, event):
        ogm = self.subNode.getSlamMap()
        #costmap = self.subNode.getCostMap()
        goal = self.subNode.getGoal2()
        coverage = self.subNode.getCoverage()
        origin = self.subNode.origin
        robotPose1 = self.subNode.robotPose1
        resolution = rospy.get_param('resolution')

        flag = 0
        force_random = False

        rospy.logwarn("Robot2 Goal is: [x, y] = [%f, %f] ", goal['x'], goal['y'])



        target1 = self.selectTarget.targetSelection(ogm, coverage, origin, \
                                    resolution, robotPose1, flag, goal, force_random)


        rospy.loginfo("target 1 from Send_move_base_goal_two_robots_is:[%f, %f] ", target1[0], target1[1])
        
        moveBaseClient1 = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)

        self.moveBaseGoal1.target_pose.pose.position.x = float(target1[0])
        self.moveBaseGoal1.target_pose.pose.position.y = float(target1[1])
        self.moveBaseGoal1.target_pose.pose.position.z = 0.0
        
        q_angle = tf.transformations.quaternion_from_euler(0, 0, target1[2], axes='sxyz')
        q = Quaternion(*q_angle)
        
        self.moveBaseGoal1.target_pose.pose.orientation = q




        moveBaseClient1.wait_for_server()
        rospy.loginfo("[Main Node] Sending goal 1....")
        rospy.loginfo("[Main Node] Goal at [%f, %f, %f]!", \
                        self.moveBaseGoal1.target_pose.pose.position.x, \
                        self.moveBaseGoal1.target_pose.pose.position.y, \
                         self.moveBaseGoal1.target_pose.pose.position.z)
        moveBaseClient1.send_goal(self.moveBaseGoal1)
        moveBaseClient1.wait_for_result()


#

    
