#!/usr/bin/env python

import rospy
import numpy
import math
import random
import tf
import time
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

class SubscriberNode:

    def __init__(self):
        # Parameters from the .yaml file
        slamMapTopic = rospy.get_param('slam_map')
        coverageMapTopic = rospy.get_param('coverage_map')
        coverageMapTopic1 = '/robot1/coverage_map'
        coverageMapTopic2 = '/robot2/coverage_map'
        self.resolution = rospy.get_param('resolution')
        self.mapFrame = rospy.get_param('map_frame')
        self.baseFootprintFrame = rospy.get_param('base_footprint_frame')

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

        # dict that holds the robot pose
        self.robotPose = {}
        self.robotPose['x'] = 0
        self.robotPose['y'] = 0
        self.robotPose['z'] = 0
        self.robotPose['th'] = 0
        self.robotPose['x_px'] = 0
        self.robotPose['y_px'] = 0
        self.robotPose['z_px'] = 0

        # timestamp list
        self.timestamps = [0]

        # Robot pose tf listener and read function
        self.robotPoseListener = tf.TransformListener()
        rospy.Timer(rospy.Duration(0.11), self.readRobotPose)

        # tf listener for the camera frame
        # self.cameraListener = tf.TransformListener()

        # Subscribers and publishers
        rospy.Subscriber(slamMapTopic, OccupancyGrid, self.ogmCallback, queue_size=1, \
                            buff_size=2**24)
        rospy.Subscriber(coverageMapTopic, OccupancyGrid, self.coverageCallback, \
                            queue_size=1, buff_size=2**24)
        rospy.Subscriber('/timestamps_topic', Float32, self.TimeStampCallback,
                            queue_size=1, buff_size=2**24)
#        rospy.Subscriber(coverageMapTopic1, OccupancyGrid, self.coverage1Callback, \
#                            queue_size=1, buff_size=2**24)
#        rospy.Subscriber(coverageMapTopic2, OccupancyGrid, self.coverage2Callback, \
#                            queue_size=1, buff_size=2**24)



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



    def readRobotPose(self, event):
        try:
            # Reads the robot pose from tf
            (translation, rotation) = self.robotPoseListener.lookupTransform\
                    (self.mapFrame, self.baseFootprintFrame, rospy.Time(0))
        # Catch the exception if something is wrong
        except (tf.LookupException, tf.ConnectivityException, \
                tf.ExtrapolationException):
            # Just print the error to try again
            rospy.logwarn("[Main Node] Error in tf transform from robot to map frame!")
            return

        # Updating the robot pose
        self.robotPose['x'] = translation[0]
        self.robotPose['y'] = translation[1]
        self.robotPose['x_px'] = self.robotPose['x'] / self.resolution
        self.robotPose['y_px'] = self.robotPose['y'] / self.resolution

        # Getting the Euler angles
        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robotPose['th'] = angles[2]

    def TimeStampCallback(self, data):
        self.timestamps.append(data.data)

    def getCoverage(self):
        return numpy.copy(self.coverage)

    def getTimeStamp(self):
        return self.timestamps
#
#    def getCoverage2(self):
#        return numpy.copy(self.coverage2)


    def getSlamMap(self):
        return numpy.copy(self.ogm)
#
#    def getCostMap(self):
#        return numpy.copy(self.costmap)
