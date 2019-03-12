#!/usr/bin/env python

import rospy
import numpy
import math
import random
import tf
import time
from nav_msgs.msg import OccupancyGrid


class SubscriberNode:

    def __init__(self):
        # Parameters from the .yaml file
        slamMapTopic = rospy.get_param('slam_map')
        coverageMapTopic = rospy.get_param('coverage_map')
        self.resolution = rospy.get_param('resolution')
        self.mapFrame = rospy.get_param('map_frame')
        self.baseFootprintFrame1 = rospy.get_param('base_footprint_frame1')
        self.baseFootprintFrame2 = rospy.get_param('base_footprint_frame2')

        # Useful variables
        self.ogm = [] # list
        self.coverage = []
#        self.coverage1 = []
#        self.coverage2 = []
        self.previousOgmWidth = 0
        self.previousOgmHeight = 0
        self.previousCovWidth = 0
        self.previousCovHeight = 0
        self.globalCostmap = OccupancyGrid()
        self.mapDataReady = False

        # dict that holds the map origin
        self.origin = {} # dict
        self.origin['x'] = 0
        self.origin['y'] = 0
        self.origin['x_px'] = 0
        self.origin['y_px'] = 0

        # dict that holds the robot1 pose
        self.robotPose1 = {}
        self.robotPose1['x'] = 0
        self.robotPose1['y'] = 0
        self.robotPose1['z'] = 0
        self.robotPose1['th'] = 0
        self.robotPose1['x_px'] = 0
        self.robotPose1['y_px'] = 0
        self.robotPose1['z_px'] = 0

        ## dict that holds the robot2 pose
        self.robotPose2 = {}
        self.robotPose2['x'] = 0
        self.robotPose2['y'] = 0
        self.robotPose2['z'] = 0
        self.robotPose2['th'] = 0
        self.robotPose2['x_px'] = 0
        self.robotPose2['y_px'] = 0
        self.robotPose2['z_px'] = 0

#       Robot pose tf listener and read function
        # Robot1
        self.robotPoseListener1 = tf.TransformListener()
        rospy.Timer(rospy.Duration(0.11), self.readRobot1Pose)

        self.robotPoseListener2 = tf.TransformListener()
        rospy.Timer(rospy.Duration(0.11), self.readRobot2Pose)





        # tf listener for the camera frame
        # self.cameraListener = tf.TransformListener()

        # Subscribers and publishers
        rospy.Subscriber(slamMapTopic, OccupancyGrid, self.ogmCallback, queue_size=1, \
                            buff_size=2**24)
        rospy.Subscriber(coverageMapTopic, OccupancyGrid, self.coverageCallback, \
                            queue_size=1, buff_size=2**24)



    def ogmCallback(self, data):
        # Map origin data to struct
        self.origin['x'] = data.info.origin.position.x
        self.origin['y'] = data.info.origin.position.y
        self.origin['x_px'] = int(data.info.origin.position.x / self.resolution)
        self.origin['y_px'] = int(data.info.origin.position.y / self.resolution)

        # Resize SLAM and Coverage OGMs when the SLAM map expands
        if data.info.width != self.previousOgmWidth or \
                data.info.height != self.previousOgmHeight:
            rospy.logwarn("[Main Node] Resizing SLAM OGM! New size: [%u, %u]", \
                            data.info.width, data.info.height)
            self.ogm = numpy.zeros((data.info.width, data.info.height), dtype = numpy.int)

            # Update previous SLAM OGM width and height
            self.previousOgmWidth = data.info.width
            self.previousOgmHeight = data.info.height

        # Construct 2-D OGM matrix
        for i in range(0, data.info.width):
            for j in range(0, data.info.height):
                self.ogm[i][j] = data.data[i + data.info.width * j]

        self.mapDataReady = True


    def coverageCallback(self, data):
        # Map origin data to struct
        self.origin['x'] = data.info.origin.position.x
        self.origin['y'] = data.info.origin.position.y
        self.origin['x_px'] = int(data.info.origin.position.x / self.resolution)
        self.origin['y_px'] = int(data.info.origin.position.y / self.resolution)

        if self.previousCovWidth != data.info.width or \
                self.previousCovHeight != data.info.height:
            rospy.logwarn("[Main Node] Resizing Coverage OGM! New size: [%u, %u]", \
                            data.info.width, data.info.height)

            self.coverage = numpy.zeros((data.info.width, data.info.height), \
                                dtype = numpy.int)

            self.previousCovWidth = data.info.width
            self.previousCovHeight = data.info.height

        for i in range(0, data.info.width):
            for j in range(0, data.info.height):
                self.coverage[i][j] = data.data[i + data.info.width * j]


    def readRobot1Pose(self, event):
        try:
            # Reads the robot pose from tf
            (translation, rotation) = self.robotPoseListener1.lookupTransform\
                    (self.mapFrame, self.baseFootprintFrame1, rospy.Time(0))
        # Catch the exception if something is wrong
        except (tf.LookupException, tf.ConnectivityException, \
                tf.ExtrapolationException):
            # Just print the error to try again
            rospy.logwarn("[Main Node] Error in tf transform from robot1 to map frame!")
            return

        # Updating the robot pose
        self.robotPose1['x'] = translation[0]
        self.robotPose1['y'] = translation[1]
        self.robotPose1['x_px'] = self.robotPose1['x'] / self.resolution
        self.robotPose1['y_px'] = self.robotPose1['y'] / self.resolution

        # Getting the Euler angles
        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robotPose1['th'] = angles[2]

    def readRobot2Pose(self, event):
        try:
            # Reads the robot pose from tf
            (translation, rotation) = self.robotPoseListener2.lookupTransform\
                    (self.mapFrame, self.baseFootprintFrame2, rospy.Time(0))
        # Catch the exception if something is wrong
        except (tf.LookupException, tf.ConnectivityException, \
                tf.ExtrapolationException):
            # Just print the error to try again
            rospy.logwarn("[Main Node] Error in tf transform from robot2 to map frame!")
            return

        # Updating the robot pose
        self.robotPose2['x'] = translation[0]
        self.robotPose2['y'] = translation[1]
        self.robotPose2['x_px'] = self.robotPose2['x'] / self.resolution
        self.robotPose2['y_px'] = self.robotPose2['y'] / self.resolution

        # Getting the Euler angles
        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robotPose2['th'] = angles[2]

    def getCoverage(self):
        return numpy.copy(self.coverage)

#    def getCoverage1(self):
#        return numpy.copy(self.coverage1)
#
#    def getCoverage2(self):
#        return numpy.copy(self.coverage2)


    def getSlamMap(self):
        return numpy.copy(self.ogm)
#
#    def getCostMap(self):
#        return numpy.copy(self.costmap)
