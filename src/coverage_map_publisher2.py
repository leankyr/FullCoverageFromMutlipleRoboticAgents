#!/usr/bin/env python

import rospy
import tf
import numpy
import time
import math
import time
# as the coverage map is also an OGM we need:
from nav_msgs.msg import OccupancyGrid


class CoverageMapPublisher:

    # Constructor
    def __init__(self):
        self.coverage_ogm = OccupancyGrid()
        self.previousWidth = 0
        self.previousHeight = 0
        self.previousOriginX = 0
        self.previousOriginY = 0
        # Flags for debugging and synchronization
        self.haveMap = False
        self.mapToken = False
        self.mapCompute = False
        self.listener = tf.TransformListener()
        # Holds the occupancy grid map
        self.ogm = 0
        self.ros_ogm = 0
        self.ogm_copy = 0

        # Holds the ogm info for copying reasons -- do not change
        self.ogm_info = 0
        self.prev_ogm_info = 0

        self.slamMapTopic = rospy.get_param('slam_map')
        self.coverageMapTopic = rospy.get_param('coverage_map')
        self.map_frame = rospy.get_param('map_frame')
        self.resolution = rospy.get_param('resolution')
        self.base_footprint_frame = rospy.get_param('sensor_frame_1')

        # Holds the coverage information. This has the same size as the ogm
        # If a cell has the value of 0 it is uncovered
        # In the opposite case the cell's value will be 100
        self.coverage = list()

        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = dict()
        self.origin['x'] = 0
        self.origin['y'] = 0

        # Initialization of robot pose
        # x,y are in meters
        # x_px, y_px are in pixels
        self.robot_pose = dict()
        self.robot_pose['x'] = 0
        self.robot_pose['y'] = 0
        self.robot_pose['th'] = 0
        self.robot_pose['x_px'] = 0
        self.robot_pose['y_px'] = 0

        self.coverage_ogm.header.frame_id = "map"


        # Use tf to read robot pose
        self.listener = tf.TransformListener()

        # Read robot pose with a timer
        rospy.Timer(rospy.Duration(0.11), self.readRobotPose)

        # Subscriber to the occupancy grid map
        rospy.Subscriber(self.slamMapTopic, OccupancyGrid, self.readMap)

        # Publisher of the coverage field
        self.coverage_publisher = rospy.Publisher(self.coverageMapTopic, \
            OccupancyGrid, queue_size = 10)

    # Getter for OGM. Must use flags since its update is asynchronous
#    def getMap(self):
#        print "Robot perception: Map requested"
#        # The map is being processed ... waiting
#        while self.mapCompute == True:
#            pass
#
#        # Locking the map
#        self.mapToken = True
#         # Copying it
#        cp = numpy.copy(self.ogm)
#        # Unlocking it
#        self.mapToken = False
#
#        # Return the copy
#        return cp

    def getRosMap(self):
        return self.ros_ogm

    # Getter for Coverage
    def getCoverage(self):
        return numpy.copy(self.coverage)

    # Reading the robot pose
    def readRobotPose(self, event):
        try:
            # Reads the robot pose from tf
            (translation, rotation) = self.listener.lookupTransform\
                    (self.map_frame, self.base_footprint_frame, rospy.Time(0))
        # Catch the exception if something is wrong
        except (tf.LookupException, tf.ConnectivityException, \
                tf.ExtrapolationException):
            # Just print the error to try again
            print "Error in tf"
            return

        # Updating the robot pose
        self.robot_pose['x'] = translation[0]
        self.robot_pose['y'] = translation[1]
        self.robot_pose['x_px'] = int(self.robot_pose['x'] / self.resolution)
        self.robot_pose['y_px'] = int(self.robot_pose['y'] / self.resolution)

        # Getting the Euler angles
        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robot_pose['th'] = angles[2]

    # Getting the occupancy grid map
    def readMap(self, data):

        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 or -1 is the unknown
        # Locking the map
        self.mapCompute = True

        self.ros_ogm = data
        # Reading the map pixels
        self.ogm_info = data.info

        if self.haveMap == False or \
                self.ogm_info.width != self.prev_ogm_info.width or \
                self.ogm_info.height != self.prev_ogm_info.height:

            self.ogm = numpy.zeros((data.info.width, data.info.height), \
                    dtype = numpy.int)
            #Print.art_print("Map & coverage expansion!", Print.GREEN)
            self.prev_ogm_info = self.ogm_info

            # Update coverage container as well
            coverage_copy = numpy.zeros([self.ogm_info.width, self.ogm_info.height])
            print "Coverage copy new size: " + str(coverage_copy.shape)
            self.coverage_ogm.info = self.ogm_info
            self.coverage_ogm.data = \
                numpy.zeros(self.ogm_info.width * self.ogm_info.height)

            if self.haveMap == True:
                print "Copying coverage field" + str(self.coverage.shape)
                for i in range(0, self.coverage.shape[0]):
                    for j in range(0, self.coverage.shape[1]):
                        coverage_copy[i][j] = self.coverage[i][j]

            # Coverage now gets the new size
            self.coverage = numpy.zeros([self.ogm_info.width, self.ogm_info.height])
            for i in range(0, self.coverage.shape[0]):
                for j in range(0, self.coverage.shape[1]):
                    self.coverage[i][j] = coverage_copy[i][j]
            print "New coverage info: " + str(self.coverage.shape)

            for i in range(0, self.coverage.shape[0]):
                for j in range(0, self.coverage.shape[1]):
                    index = int(i + self.ogm_info.width * j)
                    self.coverage_ogm.data[index] = self.coverage[i][j]
                    # TODO: Thats not quite right - the origins must be checked

        for x in range(0, data.info.width):
          for y in range(0, data.info.height):
            self.ogm[x][y] = data.data[x + data.info.width * y]

        # Get the map's resolution - each pixel's side in meters
        self.resolution = data.info.resolution

        # Get the map's origin
        self.origin['x'] = data.info.origin.position.x
        self.origin['y'] = data.info.origin.position.y

        # Keep a copy
        self.ogm_copy = numpy.copy(self.ogm)

        # Update coverage
        x = self.robot_pose['x_px']
        y = self.robot_pose['y_px']
        xx = self.robot_pose['x_px'] + abs(self.origin['x'] / self.resolution)
        yy = self.robot_pose['y_px'] + abs(self.origin['y'] / self.resolution)
        for i in range(-20, 20):
            for j in range(-20, 20):
                if self.ogm[int(xx + i),int( yy + j)] > 19 or self.ogm[int(xx + i), int(yy + j)] == -1:
                    continue
                self.coverage[int(xx + i), int(yy + j)] = 100
                index = int((xx + i) + self.ogm_info.width * (yy + j))
                self.coverage_ogm.data[index] = 100
        self.coverage_publisher.publish(self.coverage_ogm)

        #NOTE: uncomment this to save coverage in image
        #scipy.misc.imsave('~/Desktop/test.png', self.coverage)

        # Unlock the map
        self.mapCompute = False

        # If it is copied wait ...
        while self.mapToken == True:
          pass


if __name__ == '__main__':
    rospy.init_node('coverage_map_node')
    CoverageMapPublisher()
    rospy.spin()











