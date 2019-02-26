#!/usr/bin/env python

import rospy
import actionlib
import time
import math
import random
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from subscriber_node_two_robots import SubscriberNode
#from target_selection_class import TargetSelect #,selectRandomTarget
#from cov_based_target_select import TargetSelect
#from topo_graph_target_select import TargetSelect
from two_robots_cov_based_target_select import TargetSelect


class SendMoveBaseGoalClient:

    def __init__(self):
        self.subNode = SubscriberNode()
        self.selectTarget = TargetSelect()
        self.moveBaseGoal1 = MoveBaseGoal()
        self.moveBaseGoal2 = MoveBaseGoal()
        
        self.moveBaseGoal1.target_pose.header.frame_id = "map"
        self.moveBaseGoal1.target_pose.header.stamp = rospy.Time.now()

        self.moveBaseGoal2.target_pose.header.frame_id = "map"
        self.moveBaseGoal2.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("[Main Node] Wait 5 seconds for the subscribers to be ready!")
        time.sleep(5)

        # self.velocityPub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)

        rospy.Timer(rospy.Duration(1.0), self.calculateSendGoal)


    def calculateSendGoal(self, event):
        ogm = self.subNode.getSlamMap()
        #costmap = self.subNode.getCostMap()
        coverage = self.subNode.getCoverage()
        origin = self.subNode.origin
        robotPose1 = self.subNode.robotPose1
        robotPose2 = self.subNode.robotPose2
        resolution = rospy.get_param('resolution')

        target1, target2 = self.selectTarget.targetSelection(ogm, coverage, origin, \
                                    resolution, robotPose1, robotPose2)


        rospy.loginfo("target 1 from Send_move_base_goal_two_robots_is:[%f, %f] ",target1[0], target1[1])
        rospy.loginfo("target 2 from Send_move_base_goal_two_robots_is:[%f, %f] ",target2[0], target2[1])
        
        moveBaseClient1 = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)

        self.moveBaseGoal1.target_pose.pose.position.x = float(target1[0])
        self.moveBaseGoal1.target_pose.pose.position.y = float(target1[1])

#        self.moveBaseGoal.target_pose.pose.position.x = 1.0
#        self.moveBaseGoal.target_pose.pose.position.y = 1.0

        self.moveBaseGoal1.target_pose.pose.position.z = 0.0
        self.moveBaseGoal1.target_pose.pose.orientation.x = 0.0
        self.moveBaseGoal1.target_pose.pose.orientation.y = 0.0
        self.moveBaseGoal1.target_pose.pose.orientation.z = 0.0
        self.moveBaseGoal1.target_pose.pose.orientation.w = 1.0

        moveBaseClient1.wait_for_server()
        rospy.loginfo("[Main Node] Sending goal 1....")
        rospy.loginfo("[Main Node] Goal at [%f, %f, %f]!", \
                        self.moveBaseGoal1.target_pose.pose.position.x, \
                        self.moveBaseGoal1.target_pose.pose.position.y, \
                         self.moveBaseGoal1.target_pose.pose.position.z)
        moveBaseClient1.send_goal(self.moveBaseGoal1)
        moveBaseClient1.wait_for_result()


        moveBaseClient2 = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)

        self.moveBaseGoal2.target_pose.pose.position.x = float(target2[0])
        self.moveBaseGoal2.target_pose.pose.position.y = float(target2[1])

#        self.moveBaseGoal.target_pose.pose.position.x = 1.0
#        self.moveBaseGoal.target_pose.pose.position.y = 1.0

        self.moveBaseGoal2.target_pose.pose.position.z = 0.0
        self.moveBaseGoal2.target_pose.pose.orientation.x = 0.0
        self.moveBaseGoal2.target_pose.pose.orientation.y = 0.0
        self.moveBaseGoal2.target_pose.pose.orientation.z = 0.0
        self.moveBaseGoal2.target_pose.pose.orientation.w = 1.0


        moveBaseClient2.wait_for_server()
        rospy.loginfo("[Main Node] Sending goal 2......")
        rospy.loginfo("[Main Node] Goal at [%f, %f, %f]!", \
                        self.moveBaseGoal2.target_pose.pose.position.x, \
                        self.moveBaseGoal2.target_pose.pose.position.y, \
                         self.moveBaseGoal2.target_pose.pose.position.z)
        moveBaseClient2.send_goal(self.moveBaseGoal2)
        moveBaseClient2.wait_for_result()









        # self.rotateRobot()

#        moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#
#        self.moveBaseGoal.target_pose.pose.position.x = float(target1[0])
#        self.moveBaseGoal.target_pose.pose.position.y = float(target1[1])
#
##        self.moveBaseGoal.target_pose.pose.position.x = 1.0
##        self.moveBaseGoal.target_pose.pose.position.y = 1.0
#
#        self.moveBaseGoal.target_pose.pose.position.z = 0.0
#        self.moveBaseGoal.target_pose.pose.orientation.x = 0.0
#        self.moveBaseGoal.target_pose.pose.orientation.y = 0.0
#        self.moveBaseGoal.target_pose.pose.orientation.z = 0.0
#        self.moveBaseGoal.target_pose.pose.orientation.w = 1.0




#
#        moveBaseClient.wait_for_server()
#        rospy.loginfo("[Main Node] Sending goal")
#        rospy.loginfo("[Main Node] Goal at [%f, %f, %f]!", \
#                        self.moveBaseGoal.target_pose.pose.position.x, \
#                        self.moveBaseGoal.target_pose.pose.position.y, \
#                         self.moveBaseGoal.target_pose.pose.position.z)
#        moveBaseClient.send_goal(self.moveBaseGoal)
#        moveBaseClient.wait_for_result()


#    def rotateRobot(self):
#        velocityMsg = Twist()
#        angularSpeed = 0.3
#        relativeAngle = 2*math.pi
#        currentAngle = 0
#
#        velocityMsg.linear.x = 0
#        velocityMsg.linear.y = 0
#        velocityMsg.linear.z = 0
#        velocityMsg.angular.x = 0
#        velocityMsg.angular.y = 0
#        velocityMsg.angular.z = angularSpeed
#
#        t0 = rospy.Time.now().to_sec()
#        rospy.logwarn(rospy.get_caller_id() + ": Rotate Robot! Please wait...")
#        while currentAngle < relativeAngle:
#            self.velocityPub.publish(velocityMsg)
#            t1 = rospy.Time.now().to_sec()
#            currentAngle = angularSpeed * (t1 - t0)
#
#        velocityMsg.angular.z = 0
#        self.velocityPub.publish(velocityMsg)
#        rospy.logwarn(rospy.get_caller_id() + ": Robot Rotation OVER!")


    
