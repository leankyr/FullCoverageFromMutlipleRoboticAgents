#!/usr/bin/env python

import rospy
import tf
import numpy
import time
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
        self.sensor_frame = rospy.get_param('sensor_frame')
        self.resolution = rospy.get_param('resolution')
        self.map_frame = rospy.get_param('map_frame')

        self.distance_of_use = rospy.get_param( 'dist_of_use')
        self.hfov = rospy.get_param('field_of_view')
        self.sensor_frame = rospy.get_param('sensor_frame')

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
        rospy.Timer(rospy.Duration(1.0), self.calcCone)
        
    # Class Methods 
    def calcCone(self,event):
        
        ogm = self.subNode.getSlamMap()
        xx = self.pose['x_px'] - self.origin['x_px']
        yy = self.pose['y_px'] - self.origin['y_px']

        for th in range(0, 360):
            thRad = th * math.pi / 180
            for cc in range(0, 5):
                x = xx + cc * math.cos(thRad) 
                y = yy + cc * math.sin(thRad) 
                index = int(x) + self.coverage.info.width * int(y)
                self.coverage.data[index] = 100
                
                
            for i in range(1, 2):
                minDepth = int(0.2 / self.resolution)
                maxDepth = int(self.distance_of_use / self.resolution)
                theta = int(self.hfov / 2)
                angles = self.pose['th']

                for th in range(-theta, theta + 1):
                    thRad = angles + th * math.pi / 180
                    for crosscut in range(minDepth, maxDepth + 1):
                        x = xx + crosscut * math.cos(thRad)
                        y = yy + crosscut * math.sin(thRad) 
                        index = int(x) + self.coverage.info.width * int(y)
                        if ogm[int(x)][int(y)] > 49:
                            break
                        self.coverage.data[index] = 100
        self.cov_pub.publish(self.coverage)               

if __name__ == '__main__':
    rospy.init_node('coverage_map_publisher')
    CoverageMapPublisher()
    rospy.spin()

