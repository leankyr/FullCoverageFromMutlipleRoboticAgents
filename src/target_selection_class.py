#!/usr/bin/env python


import rospy
import actionlib
import time
import math
import tf
import numpy


from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid


class TargetSelect:

    def __init__(self):
        # velocity publisher
        self.velocityPub = rospy.Publisher(rospy.get_param('velocity_pub'),Twist,queue_size = 10)
        # subscriber to OGmap 
        self.mapSub = rospy.Subscriber(rospy.get_param('slam_map'),OccupancyGrid,self.slamCallback,queue_size = 1, buff_size=2**24)
        # Map Dimensions
        self.coverage = OccupancyGrid()
        self.xLimitUp = 0
        self.xLimitDown = 0
        self.yLimitUp = 0
        self.yLimitDown = 0
        self.resolution = rospy.get_param('resolution')
        # self.height = data.info.height
        # self.width = data.info.width

        self.listener = tf.TransformListener()
        self.mapFrame = rospy.get_param('map_frame')
        self.baseFrame = rospy.get_param('base_footprint_frame')

    def slamCallback(self,data):
        rospy.loginfo("------------------------------")
        #rospy.loginfo("[Target Select Node] map_origin: [x,y] = [%f, %f]", \
        #                data.info.origin.position.x , data.info.origin.position.x)
        #rospy.loginfo("[Target Select Node] dims: [width,height] = [%f, %f]", \
        #                data.info.width , data.info.height)

        # Calculate Robot's Pose
        try:
            (translation, rotation) = self.listener.lookupTransform\
            (self.mapFrame, self.baseFrame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, \
            tf.ExtrapolationException):
            rospy.logwarn("Error in tf")


        rospy.loginfo("[Target Select Node] Robot Pose: [x,y] = [%f, %f]", \
                        translation[0], translation[1])
        
        x_coord = round(translation[0] + data.info.width/2)
        y_coord = round(translation[1] + data.info.height/2)

        print x_coord
        print y_coord
        
        # the grids dimensions is in meters
        # I don't take into consideration resolution (yet)
        grid = numpy.reshape(data.data,(data.info.width,data.info.height))
        print grid[x_coord,y_coord]
        # This returns 0 (fee space) as expected

        # Somewhere here I have to start BrushFire I guess
        # Check P110 Phd Tsardoulias

        return

    def get_init_rot(self):
        return self.init_rot

    def rotateRobot(self):
        velocityMsg = Twist()
        angularSpeed = 0.3
        relativeAngle = 2*math.pi
        currentAngle = 0
        
        rospy.loginfo("Roatating robot...")
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






