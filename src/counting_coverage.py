#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import rospy
import tf
import numpy
import math
import time

from bresenham import bresenham
# as the coverage map is also an OGM we need:
from nav_msgs.msg import OccupancyGrid
# from subscriber_node import SubscriberNode
from subscriber_node_two_robots import SubscriberNode

class CoverageCounterPublisher:

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

        # origin of map
        self.origin_x = rospy.get_param('origin_x')
        self.origin_y = rospy.get_param('origin_y')

        # define the timer
        rospy.Timer(rospy.Duration(45.0), self.countCoverage)


    def countCoverage(self, event):
        # get the maps

        ogm = self.subNode.getSlamMap()
        cov = self.subNode.getCoverage()
        
        dim_ogm = numpy.shape(ogm)
        dim_cov = numpy.shape(cov)
        
        num_of_zero_boxes_ogm = 0
        for i in range(dim_ogm[0]):
            for j in range(dim_ogm[1]):
                if ogm[i][j] == 0:
                    num_of_zero_boxes_ogm += 1

        # print (num_of_zero_boxes_ogm)

        
        num_of_hundred_boxes_cov = 0
        for i in range(dim_ogm[0]):
            for j in range(dim_ogm[1]):
                if ogm[i][j] < 20 and ogm[i][j] != -1 \
                and cov[i][j] > 80:
                    num_of_hundred_boxes_cov += 1

        # print (num_of_hundred_boxes_cov)


        now = time.ctime()
        print ('%.3f' %(num_of_hundred_boxes_cov/num_of_zero_boxes_ogm), now)

        if num_of_hundred_boxes_cov/num_of_zero_boxes_ogm > 0.95: 
            rospy.signal_shutdown('Coverage limit reached!!!!')


if __name__ == '__main__':
    rospy.init_node('coverage_counter')
    CoverageCounterPublisher()
    rospy.spin()

