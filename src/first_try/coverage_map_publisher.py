#!/usr/bin/env python

import rospy
import numpy
import tf
import time
import numpy
import math
from nav_msgs.msg import OccupancyGrid

class CoverageMapPublisher:

    def __init__(self):
        self.coverage = OccupancyGrid()
        self.previousWidth = 0
        self.previousHeight = 0
        self.previousOriginX = 0
        self.previousOriginY = 0
        self.haveMap = False
        self.listener = tf.TransformListener()

        self.numOfSensors = rospy.get_param('num_of_sensors')
        self.slamMapTopic = rospy.get_param('slam_map')
        self.coverageMapTopic = rospy.get_param('coverage_map')
        self.mapFrame = rospy.get_param('map_frame')
        self.distanceOfUse = 0
        self.hfov = 0
        self.sensorFrame = 0

        rospy.Subscriber(self.slamMapTopic, OccupancyGrid, self.slamCallback, \
                            queue_size=1, buff_size=2**24)
        self.coveragePub = \
                rospy.Publisher(self.coverageMapTopic, OccupancyGrid, queue_size=1)

    def slamCallback(self, data):
        if data.info.width != self.previousWidth or data.info.height != self.previousHeight:
            rospy.logwarn("[Coverage Node] Resizing coverage map! New size: [%u, %u]", \
                            data.info.width, data.info.height)
            self.coverage.header.frame_id = data.header.frame_id
            self.coverage.info.width = data.info.width
            self.coverage.info.height = data.info.height
            self.coverage.info.resolution = data.info.resolution
            self.coverage.info.origin = data.info.origin

            if self.haveMap == True:
                prevX = int(abs(self.previousOriginX - data.info.origin.position.x) / \
                            data.info.resolution)
                prevY = int(abs(self.previousOriginY - data.info.origin.position.y) / \
                            data.info.resolution)
                coverageCopy = numpy.zeros(self.previousWidth * self.previousHeight)
                for i in range(0, self.previousWidth):
                    for j in range(0, self.previousHeight):
                        index = i + self.previousWidth * j
                        coverageCopy[index] = self.coverage.data[index]
                self.coverage.data = numpy.zeros(data.info.width * data.info.height)
                for i in range(prevX, data.info.width):
                    for j in range(prevY, data.info.height):
                        index = i + data.info.width * j
                        indexCopy = (i - prevX) + self.previousWidth * (j - prevY)
                        self.coverage.data[index] = coverageCopy[indexCopy]

                self.previousOriginX = data.info.origin.position.x
                self.previousOriginY = data.info.origin.position.y
                self.previousHeight = data.info.height
                self.previousWidth = data.info.width
            else:
                self.coverage.data = numpy.zeros(data.info.width * data.info.height)
                self.previousOriginX = data.info.origin.position.x
                self.previousOriginY = data.info.origin.position.y
                self.previousHeight = data.info.height
                self.previousWidth = data.info.width

        for i in range(1, self.numOfSensors + 1):
            self.distanceOfUse = rospy.get_param('dist_of_use_' + str(i))
            self.hfov = rospy.get_param('hfov_' + str(i))
            self.sensorFrame = rospy.get_param('sensor_frame_' + str(i))

            maxDepth = int(self.distanceOfUse / data.info.resolution)
            theta = int(self.hfov / 2)

            try:
                (translation, rotation) = self.listener.lookupTransform\
                    (self.mapFrame, self.sensorFrame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, \
                    tf.ExtrapolationException):
                rospy.logwarn("Error in tf")
                return
            angles = tf.transformations.euler_from_quaternion(rotation)

            for th in range(-theta, theta + 1):
                thRad = angles[2] + th * math.pi / 180
                for crosscut in range(0, maxDepth + 1):
                    x = (translation[0] / data.info.resolution + crosscut * math.cos(thRad)) - \
                            data.info.origin.position.x / data.info.resolution
                    y = (translation[1] / data.info.resolution + crosscut * math.sin(thRad)) - \
                            data.info.origin.position.y / data.info.resolution
                    index = int(round(x) + self.coverage.info.width * round(y))
                    if data.data[index] != 100 and data.data[index] != -1:
                        self.coverage.data[index] = 100

            self.coveragePub.publish(self.coverage)

        if self.haveMap == False:
            self.haveMap = True


if __name__ == '__main__':
    rospy.init_node("coverage_publisher_node")
    CoverageMapPublisher()
    rospy.spin()
