#!/usr/bin/env python

import rospy
import numpy
import math
import random
import tf
import time

from nav_msgs.msg import OccupancyGrid
#from subscriber_node import SubscriberNode

class CoverageMerger:

    def __init__(self):
 #       self.subNode = SubscriberNode()
        self.common_coverage = 'common_coverage_map'
        self.coverage1 = []
        self.coverage2 = []
        self.coverageAll = OccupancyGrid()
        coverageMapTopic1 = '/robot1/coverage_map'
        coverageMapTopic2 = '/robot2/coverage_map'

        # Publishers and Subscribers

        self.cov_pub = rospy.Publisher(self.common_coverage, OccupancyGrid, queue_size=1)
        rospy.Subscriber(coverageMapTopic1, OccupancyGrid, self.coverage1Callback, \
                            queue_size=1, buff_size=2**24)
        rospy.Subscriber(coverageMapTopic2, OccupancyGrid, self.coverage2Callback, \
                            queue_size=1, buff_size=2**24)

        self.Width = rospy.get_param('width')
        self.Height = rospy.get_param('height')
        #self.origin = self.subNode.origin

        self.coverageAll.info.origin.position.x = rospy.get_param('origin_x')
        self.coverageAll.info.origin.position.y = rospy.get_param('origin_y')
       
        # size of our coverage map
        self.coverageAll.info.width = self.Width
        self.coverageAll.info.height = self.Height
         
        # initialize coverage data
        self.coverageAll.data = [0] * self.Width * self.Height
        self.coverage1 = numpy.zeros((self.Width, self.Height), \
                                dtype = numpy.int)
        self.coverage2 = numpy.zeros((self.Width, self.Height), \
                                dtype = numpy.int)

        # resolution of our coverage map
        self.coverageAll.info.resolution = 0.1
        
        #if numpy.size(self.coverage1) != 0:
        rospy.Timer(rospy.Duration(1.0), self.mergeMaps)

    def mergeMaps(self, event):

        cov1 = self.coverage1
        cov2 = self.coverage2
        print "cov1 shape is:"
        print numpy.shape(cov1)
        print "cov2 shape is:"
        print numpy.shape(cov2)
        #print numpy.shape(self.coverageAll.data)

        for i in range(0, self.Width):
            for j in range(0, self.Height):
                if cov1[i][j] == 100 or cov2[i][j] == 100:
                    self.coverageAll.data[i + self.Width * j] = 100
                else:
                    self.coverageAll.data[i + self.Width * j] = 0
        
        print 'publishing to topic.....' 
        self.cov_pub.publish(self.coverageAll)
        print 'published to topic.....'
        pass


    def coverage1Callback(self, data):
        
                    
        for i in range(0, self.Width):
            for j in range(0, self.Height):
                self.coverage1[i][j] = data.data[i + self.Width * j]
        print "got Through Coverage 1 ogm!!"
        return

    def coverage2Callback(self, data):

        
        for i in range(0, self.Width):
            for j in range(0, self.Height):
                self.coverage2[i][j] = data.data[i + self.Width * j]
        print "got Through Coverage 2 ogm!!!!"
        return


#    def getCoverage1(self):
#        return numpy.copy(self.coverage1)
#
#    def getCoverage2(self):
#        return numpy.copy(self.coverage2)

if __name__ == '__main__':
    rospy.init_node('coverage_map_merger')
    CoverageMerger()
    rospy.spin()



