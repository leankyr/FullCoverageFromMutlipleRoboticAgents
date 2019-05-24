#!/usr/bin/env python

import rospy
import actionlib
import time
import math
import random
import tf

from timeit import default_timer as tmpimer
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from subscriber_node_two_robots import SubscriberNode
#from target_selection_class import TargetSelect #,selectRandomTarget
#from cov_based_target_select import TargetSelect
from topo_graph_target_select2 import TargetSelect
#from two_robots_cov_based_target_select import TargetSelect
from std_msgs.msg import Float64

class SendMoveBaseGoalClient2:

    def __init__(self):
        self.subNode = SubscriberNode()
        self.selectTarget = TargetSelect()
        self.moveBaseGoal2 = MoveBaseGoal()
        self.timeInit = Float64()
        self.timeEnd = Float64()

        self.moveBaseGoal2.target_pose.header.frame_id = "map"
        self.moveBaseGoal2.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("[Main Node] Wait 20 seconds for the subscribers to be ready!")
        time.sleep(20)

        self.TimeInitPub = rospy.Publisher('/robot2/timeinit', Float64, queue_size = 1)
        self.TimeEndPub = rospy.Publisher('/robot2/timeend', Float64, queue_size = 1)

        rospy.Timer(rospy.Duration(1.0), self.calculateSendGoal)


    def calculateSendGoal(self, event):
        ogm = self.subNode.getSlamMap()
        #costmap = self.subNode.getCostMap()
        goal = self.subNode.getGoal1()
        coverage = self.subNode.getCoverage()
        origin = self.subNode.origin
        robotPose2 = self.subNode.robotPose2
        resolution = rospy.get_param('resolution')

        flag = 1
        force_random = True

        rospy.logwarn("Robot1 Goal is: [x, y] = [%f, %f] ", goal['x'], goal['y'])
        target2 = self.selectTarget.targetSelection(ogm, coverage, origin, \
                                    resolution, robotPose2, flag, goal, force_random)

        rospy.loginfo("target 2 from Send_move_base_goal_two_robots_is:[%f, %f] ", target2[0], target2[1])
        
        

        moveBaseClient2 = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)

        self.moveBaseGoal2.target_pose.pose.position.x = float(target2[0])
        self.moveBaseGoal2.target_pose.pose.position.y = float(target2[1])

        self.moveBaseGoal2.target_pose.pose.position.z = 0.0

        q_angle = tf.transformations.quaternion_from_euler(0, 0, target2[2], axes='sxyz')
        q = Quaternion(*q_angle)
        
        self.moveBaseGoal2.target_pose.pose.orientation = q
        
        self.timeInit.data = time.time()
        rospy.logwarn('Initial time Robot 2(Two) is: %s', str(self.timeInit.data))
        moveBaseClient2.wait_for_server()
        rospy.loginfo("[Main Node] Sending goal 2......")
        rospy.loginfo("[Main Node] Goal at [%f, %f, %f]!", \
                        self.moveBaseGoal2.target_pose.pose.position.x, \
                        self.moveBaseGoal2.target_pose.pose.position.y, \
                         self.moveBaseGoal2.target_pose.pose.position.z)
        moveBaseClient2.send_goal(self.moveBaseGoal2)
        moveBaseClient2.wait_for_result()
        self.timeEnd.data = time.time()
        rospy.logwarn('End time Robot2 (Two) is: %s', str(self.timeEnd.data))
        rospy.logerr('Clear Explore Robot2 (Two) time is: %s', str(self.timeEnd.data - self.timeInit.data))
        self.TimeInitPub.publish(self.timeInit)
        self.TimeEndPub.publish(self.timeEnd)



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


    
