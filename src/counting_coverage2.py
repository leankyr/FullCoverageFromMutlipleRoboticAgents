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
#from subscriber_node import SubscriberNode
from subscriber_node_two_robots import SubscriberNode

# from send_move_base_goal import SendMoveBaseGoalClient

class CoverageCounterPublisher:

    # constructor
    def __init__(self):
        # init sub node object
        self.subNode = SubscriberNode()
  #      self.moveBaseClient = SendMoveBaseGoalClient()

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
        self.stamps = []
        self.k = 1

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
        
        stamp = self.calculateDiff()
        # stamp = sum(self.subNode.getTimeStamp())
        print ('%.3f' %(num_of_hundred_boxes_cov/num_of_zero_boxes_ogm), stamp)

        if num_of_hundred_boxes_cov/num_of_zero_boxes_ogm > 0.95: 
            rospy.signal_shutdown('Coverage limit reached!!!!')


    def calculateDiff(self):

#        print ('exp Time Robot1', tend1 - tinit1)
#        print ('exp Time Robot2', tend1 - tinit1)

        tinit1 = self.subNode.getTimeStampInit1()
        tend1 = self.subNode.getTimeStampEnd1()

        tinit2 = self.subNode.getTimeStampInit2()
        tend2 = self.subNode.getTimeStampEnd2()

        return sum(tend1)/2 + sum(tend2)/2 - sum(tinit1)/2 - sum(tinit2)/2
    
    
    
    
    
    
#        return sum(self.stamps)
    




#        print('Tinit1 is:', tinit1)
#        print('Tend1 is:', tend1,'\n')
#        
#        print('Tinit2 is:', tinit2)
#        print('Tend2 is:', tend2, '\n')
        
#        if len(tinit1) == self.k and len(tinit2) == self.k:
#            if tend1[len(tend1) - 1] <= tinit2[len(tinit2) - 1] or \
#                tend2[len(tend2) -1] <= tinit1[len(tinit1) - 1]:
#                tstamp = tend1[len(tend1) - 1] - tinit1[len(tinit1) - 1]\
#                       + tend2[len(tend1) - 1] - tinit2[len(tinit1) - 1]
#            
#            if tend1[len(tend1) - 1] > tinit2[len(tinit2) - 1] or \
#                tend2[len(tend2) -1] > tinit1[len(tinit1) - 1]:
#                
#                if tinit1[len(tinit1) -1] < tinit2[len(tinit2) -1]:
#                    tstamp = - tinit1[len(tinit1) - 1] + tend2[len(tend1) - 1] 
#
#                else:
#                    tstamp = - tend1[len(tend1) - 1] + tinit2[len(tinit1) - 1]
#
#
#            if tinit1[len(tinit1) - 1] < tinit2[len(tinit2) - 1] and  \
#                tend2[len(tend2) -1] < tend1[len(tend1) - 1]:
#                if tinit1 <= tinit2:
#                    tstamp = tend1[len(tend1) -1]-tinit1[len(tinit1) - 1]
#                else:
#                    tstamp = tend2[len(tend1) -1]-tinit2[len(tinit1) - 1]
#
#            if tinit2[len(tinit2) - 1] < tinit1[len(tinit1) - 1] and  \
#                tend1[len(tend1) -1] < tend2[len(tend2) - 1]:
#                if tinit1 <= tinit2:
#                    tstamp = tend1[len(tend1) -1]-tinit1[len(tinit1) - 1]
#                else:
#                    tstamp = tend2[len(tend2) -1]-tinit2[len(tinit2) - 1]
#
#            self.k += 1
#            self.stamps.append(tstamp)
#            return sum(self.stamps)
            




if __name__ == '__main__':
    rospy.init_node('coverage_counter')
    CoverageCounterPublisher()
    rospy.spin()

