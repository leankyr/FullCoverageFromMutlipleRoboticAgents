#!/usr/bin/env python

import rospy
import actionlib
import time
import math
import tf
from timeit import default_timer as timer 

from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from subscriber_node import SubscriberNode
# from target_selection_class import TargetSelect #,selectRandomTarget
# from cov_based_target_select import TargetSelect
from topo_graph_target_select import TargetSelect
# from two_robots_cov_based_target_select import TargetSelect
from std_msgs.msg import Float32

class SendMoveBaseGoalClient:

    def __init__(self):
        self.subNode = SubscriberNode()
        self.selectTarget = TargetSelect()
        self.moveBaseGoal = MoveBaseGoal()
        self.timeStampMsg = Float32()

        self.moveBaseGoal.target_pose.header.frame_id = "map"
        self.moveBaseGoal.target_pose.header.stamp = rospy.Time.now()

        rospy.loginfo("[Main Node] Wait 20 seconds for the subscribers to be ready!")
        time.sleep(20)

        self.timestampPub = rospy.Publisher('/timestamps_topic', Float32, queue_size = 1)

        rospy.Timer(rospy.Duration(1.0), self.calculateSendGoal)

        

    def calculateSendGoal(self, event):
        ogm = self.subNode.getSlamMap()
        #costmap = self.subNode.getCostMap()
        coverage = self.subNode.getCoverage()
        origin = self.subNode.origin
        robotPose = self.subNode.robotPose
        resolution = rospy.get_param('resolution')

        force_random = True

        # self.rotateRobot()
        target = self.selectTarget.targetSelection(ogm, coverage, origin, 
                                    resolution, robotPose, force_random)


        moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.moveBaseGoal.target_pose.pose.position.x = float(target[0])
        self.moveBaseGoal.target_pose.pose.position.y = float(target[1])
        self.moveBaseGoal.target_pose.pose.position.z = 0.0
        
        q_angle = tf.transformations.quaternion_from_euler(0, 0, target[2], axes='sxyz')
        q = Quaternion(*q_angle)
        
        self.moveBaseGoal.target_pose.pose.orientation = q

        tinit = time.time()
        rospy.loginfo('Initial time is: %s', str(tinit))
        moveBaseClient.wait_for_server()
        rospy.loginfo("[Main Node] Sending goal")
        rospy.loginfo("[Main Node] Goal at [%f, %f, %f]!", 
                        self.moveBaseGoal.target_pose.pose.position.x, 
                        self.moveBaseGoal.target_pose.pose.position.y, 
                         self.moveBaseGoal.target_pose.pose.position.z)
        moveBaseClient.send_goal(self.moveBaseGoal)
        moveBaseClient.wait_for_result()
        tend = time.time()
        rospy.loginfo('End time is: %s', str(tend))
        rospy.logwarn('Clear Explore time is: %s', str(tend - tinit))
        self.timeStampMsg.data = tend - tinit
        self.timestampPub.publish(self.timeStampMsg)

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



