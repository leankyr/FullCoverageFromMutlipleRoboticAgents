#!/usr/bin/env python

import rospy
import tf
import numpy
import math
import time

from bresenham import bresenham
# as the coverage map is also an OGM we need:
from nav_msgs.msg import OccupancyGrid
from subscriber_node import SubscriberNode
class CoverageMapPublisher:

    # constructor
    def __init__(self):
        # init sub node object
        self.subNode = SubscriberNode()

        # details for coverage map
        self.coverage = OccupancyGrid()
        self.previous_width = 0
        self.previous_height = 0
        self.previous_origin_x = 0
        self.previous_origin_x = 0
        self.listener = tf.TransformListener()
        self.have_map = False

        # topics and stuff
        self.coverage_map_topic = rospy.get_param('coverage_map')
        self.mapFrame = rospy.get_param('map_frame')
        self.footprint = rospy.get_param('base_footprint_frame')
        self.resolution = rospy.get_param('resolution')

        # Publishers
        self.cov_pub = rospy.Publisher(self.coverage_map_topic, OccupancyGrid, queue_size=1)

        # Radius for the circle of around pose
        self.radius = numpy.full([4], rospy.get_param('radius'))

        # Pose of robot
        self.pose = self.subNode.robotPose

        # origin of  map
        # Weird Bug # Does not Work!!!!
        # Or it does??
        self.origin = self.subNode.origin

        # coverage list
        self.coverage_ogm = list()
        # origin of our coverage map
        # will try to get from yaml cause from sub node does 
        # not seem to work
        self.coverage.info.origin.position.x = rospy.get_param('origin_x')
        self.coverage.info.origin.position.y = rospy.get_param('origin_y')

        # size of our coverage map
        self.coverage.info.width = rospy.get_param('width')
        self.coverage.info.height = rospy.get_param('height')

        # initialize coverage data
        self.coverage.data = numpy.zeros(self.coverage.info.width * self.coverage.info.height)

        # resolution of our coverage map
        self.coverage.info.resolution = self.resolution

        # Summon method
        # We need the event for the same reason 
        rospy.Timer(rospy.Duration(1.0), self.calcSquare)

        # get the ogm through subcriber node

    # Class Methods 
    def calcSquare(self,event):
        #rospy.loginfo("Robot Pose is x,y = [%f,%f]", self.pose['x'],self.pose['y'])
        #rospy.loginfo("Size of our map is: width,height = [%f,%f]", self.coverage.info.width,\
                #self.coverage.info.height)
        #rospy.loginfo("origin of our map is: origin_x,origin_y = [%f,%f]", self.coverage.info.origin.position.x,\
        #                    self.coverage.info.origin.position.y)
        #############################################################################
        #########################  Here I implement Bresenham #######################
        #############################################################################

        ogm = self.subNode.getSlamMap()
        xx = self.pose['x_px'] - self.origin['x_px']
        yy = self.pose['y_px'] - self.origin['y_px']

#        print 'the radius at line 88 is: '
#        print self.radius
#
        #print 'self.radius[0] + xx =='
        #print self.radius[0] + xx
        #print int(self.radius[0] + xx)
        ### LINE East ####
        line = list(bresenham(int(xx),int(yy),int(xx),int(yy + self.radius[0]/self.resolution)))
        #print "the line is:"
        #print line
        for idx,coord in enumerate(line):
            if ogm[coord[0]][coord[1]] > 59:
                self.radius[0] = len(line[0: idx]) * self.resolution
                #print self.radius[0]
                #print 'got East'
                break

        ### LINE West ####
        line = list(bresenham(int(xx),int(yy),int(xx),int(yy - self.radius[1]/self.resolution)))
        #print "the line is:"
        #print line1
        for idx,coord in enumerate(line):
            if ogm[coord[0]][coord[1]] > 59:
                self.radius[1] = len(line[0: idx]) * self.resolution
                #print self.radius[1]
                #print 'got West'
                break

        ### LINE SOUTH ####
        line = list(bresenham(int(xx),int(yy),int(xx + self.radius[2]/self.resolution),int(yy)))
        #print "the line is:"
        #print line1
        for idx,coord in enumerate(line):
            if ogm[coord[0]][coord[1]] > 59:
                self.radius[2] = len(line[0: idx]) * self.resolution
                #print self.radius[2]
                #print 'got South'
                break

        ### LINE NORTH ###
        line = list(bresenham(int(xx),int(yy),int(xx - self.radius[3]/self.resolution),int(yy)))
        #print "the line is:"
        #print line1
        for idx,coord in enumerate(line):
            if ogm[coord[0]][coord[1]] > 59:
                self.radius[3] = len(line[0: idx]) * self.resolution
                #print self.radius[3]
                #print 'got North'
                break

        ### LINE East-South ####
        line = list(bresenham(int(xx),int(yy),int(xx + rospy.get_param('radius')/self.resolution),int(yy + rospy.get_param('radius')/self.resolution)))
        #print "the line is:"
        #print line
        for coord in line:
            if ogm[coord[0]][coord[1]] > 59:
                if coord[1] - yy < coord[0] - xx:
                    self.radius[0] = (coord[1] - yy) * self.resolution
                else:
                    self.radius[2] = (coord[0] - xx) * self.resolution
                #print 'Got East South!!'
                break


#         ### LINE West-South ####
        line = list(bresenham(int(xx),int(yy),int(xx + rospy.get_param('radius')/self.resolution),int(yy - rospy.get_param('radius')/self.resolution)))
        #print "the line is:"
        #print line
        for coord in line:
            if ogm[coord[0]][coord[1]] > 59:
                if yy - coord[1] < coord[0] - xx:
                    self.radius[1] = (yy - coord[1]) * self.resolution
                else:
                    self.radius[2] = (coord[0] - xx) * self.resolution
                #print 'Got West South'
                break

#        ### LINE East-North ####
        line = list(bresenham(int(xx),int(yy),int(xx - rospy.get_param('radius')/self.resolution),int(yy + rospy.get_param('radius')/self.resolution)))
        #print "the line is:"
        #print line
        for coord in line:
            if ogm[coord[0]][coord[1]] > 59:
                if coord[1] - yy < xx - coord[0]:
                    self.radius[0] = (coord[1] - yy) * self.resolution
                else:
                    self.radius[3] = (xx - coord[0]) * self.resolution
                #print 'got East North'
                break

#        ### LINE West-North ####
        line = list(bresenham(int(xx),int(yy),int(xx - rospy.get_param('radius')/self.resolution),int(yy - rospy.get_param('radius')/self.resolution)))
        #print "the line is:"
        #print line
        for coord in line:
            if ogm[coord[0]][coord[1]] > 59:
                if yy - coord[1] < xx - coord[0]:
                    self.radius[1] = (yy - coord[1]) * self.resolution
                else:
                    self.radius[3] = (xx - coord[0]) * self.resolution
                #print 'got West North'
                break

        #print self.radius


                #line_pxls = list(bresneham)
        for i in range(int(-self.radius[3]/self.resolution),int(self.radius[2]/self.resolution) + 1):
            for j in range(int(-self.radius[1]/self.resolution),int(self.radius[0]/self.resolution) + 1):
                index = int(self.pose['x_px'] - self.origin['x_px']) + i \
                        + self.coverage.info.width \
                        * (int(self.pose['y_px'] - self.origin['y_px']) + j)

#                if ogm[int(xx + i),int(yy + j)] > 80 or ogm[int(xx + i), int(yy + j)] == -1:
#                    self.coverage.data[index] = 0
#                else:
#                    self.coverage.data[index] = 100

                self.coverage.data[index] = 100
        self.cov_pub.publish(self.coverage)
        self.radius = numpy.full([4], rospy.get_param('radius'))
        pass


if __name__ == '__main__':
    rospy.init_node('coverage_map_publisher')
    CoverageMapPublisher()
    rospy.spin()

