#!/usr/bin/env python

import rospy
import actionlib
import time
import math
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from subscriber_node import SubscriberNode
from target_select import SelectTarget #,selectRandomTarget


class SendMoveBaseGoalClient:

    def __init__(self):
        self.subNode = SubscriberNode()
        self.selectTarget = SelectTarget()
        # self.selectTarget = selectRandomTarget()
        self.moveBaseGoal = MoveBaseGoal()

        self.moveBaseGoal.target_pose.header.frame_id = "map"
        self.moveBaseGoal.target_pose.header.stamp = rospy.Time.now()

        rospy.loginfo("[Main Node] Wait 5 seconds for the subscribers to be ready!")
        time.sleep(5)

        # self.velocityPub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)

        rospy.Timer(rospy.Duration(1.0), self.calculateSendGoal)


    def calculateSendGoal(self, event):
        ogm = self.subNode.getSlamMap()
        coverage = self.subNode.getCoverage()
        origin = self.subNode.origin
        robotPose = self.subNode.robotPose
        resolution = rospy.get_param('resolution')

        target = self.selectTarget.targetSelection(ogm, coverage, origin, \
                                    resolution, robotPose)

        # self.rotateRobot()

        moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.moveBaseGoal.target_pose.pose.position.x = float(target[0])
        self.moveBaseGoal.target_pose.pose.position.y = float(target[1])
        self.moveBaseGoal.target_pose.pose.position.z = 0.0
        self.moveBaseGoal.target_pose.pose.orientation.x = 0.0
        self.moveBaseGoal.target_pose.pose.orientation.y = 0.0
        self.moveBaseGoal.target_pose.pose.orientation.z = 0.0
        self.moveBaseGoal.target_pose.pose.orientation.w = 1.0

        moveBaseClient.wait_for_server()
        rospy.loginfo("[Main Node] Sending goal")
        rospy.loginfo("[Main Node] Goal at [%f, %f, %f]!", \
                        self.moveBaseGoal.target_pose.pose.position.x, \
                        self.moveBaseGoal.target_pose.pose.position.y, \
                         self.moveBaseGoal.target_pose.pose.position.z)
        moveBaseClient.send_goal(self.moveBaseGoal)
        moveBaseClient.wait_for_result()
        

    def rotateRobot(self):
        velocityMsg = Twist()
        angularSpeed = 0.3
        relativeAngle = 2*math.pi
        currentAngle = 0

        velocityMsg.linear.x = 0
        velocityMsg.linear.y = 0
        velocityMsg.linear.z = 0
        velocityMsg.angular.x = 0
        velocityMsg.angular.y = 0
        velocityMsg.angular.z = angularSpeed
        
        t0 = rospy.Time.now().to_sec()
        rospy.logwarn(rospy.get_caller_id() + ": Rotate Robot! Please wait...")
        while currentAngle < relativeAngle:
            self.velocityPub.publish(velocityMsg)
            t1 = rospy.Time.now().to_sec()
            currentAngle = angularSpeed * (t1 - t0)

        velocityMsg.angular.z = 0
        self.velocityPub.publish(velocityMsg)
        rospy.logwarn(rospy.get_caller_id() + ": Robot Rotation OVER!")
