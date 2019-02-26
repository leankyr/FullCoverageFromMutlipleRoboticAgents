#!/usr/bin/env python


import rospy
import actionlib
import scipy
import time
import math
import tf
import numpy
import random
from bresenham import bresenham

from utilities import OgmOperations
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from brushfires import Brushfires
from topology import Topology
from visualization_msgs.msg import Marker

import operator
class TargetSelect:

    def __init__(self):
        self.xLimitUp = 0
        self.xLimitDown = 0
        self.yLimitUp = 0
        self.yLimitDown = 0

        self.brush = Brushfires()
        self.topo = Topology()
        self.target1 = [-1, -1]
        self.target2 = [-1, -1]
        self.previousTarget = [-1, -1]
        self.costs = []


    def targetSelection(self, initOgm, coverage, origin, resolution, robotPose1, robotPose2):
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("[Target Select Node] Robot_Pose1[x, y, th] = [%f, %f, %f]", \
                    robotPose1['x'], robotPose1['y'], robotPose1['th'])
        rospy.loginfo("[Target Select Node] Robot_Pose2[x, y, th] = [%f, %f, %f]", \
                    robotPose2['x'], robotPose2['y'], robotPose2['th'])
        rospy.loginfo("[Target Select Node] OGM_Origin = [%i, %i]", origin['x'], origin['y'])
        rospy.loginfo("[Target Select Node] OGM_Size = [%u, %u]", initOgm.shape[0], initOgm.shape[1])

        # Blur the OGM to erase discontinuities due to laser rays
        #ogm = OgmOperations.blurUnoccupiedOgm(initOgm, ogmLimits)
        ogm = initOgm

        rospy.loginfo("Calculating brushfire of first robot...")
        brush1 = self.brush.coverageLimitsBrushfire(initOgm, coverage, robotPose1, origin, resolution )
        rospy.loginfo("Calculating brushfire of second robot...")
        brush2 = self.brush.coverageLimitsBrushfire(initOgm, coverage, robotPose2, origin, resolution )
        
        print 'size of brush1:'
        print len(brush1)
        print 'size of brush2:'
        print len(brush2)


        min_dist = 10**24
        store_goal1 = ()
        throw1 = set()
        store_goal2 = ()
        throw2 = set()


        throw1 = self.filterGoal(brush1, ogm, resolution, origin)
        throw2 = self.filterGoal(brush2, ogm, resolution, origin)

        print 'size of throw1:'
        print len(throw1)
        print 'size of throw2:'
        print len(throw2)


        brush1.difference_update(throw1)
        brush2.difference_update(throw2)

        print 'size of brush1 after update:'
        print len(brush1)
        print 'size of brush2 after update:'
        print len(brush2)


        # sampled brush2
        ########## Here I Sample the Goals for performance ############
        ########## However I do not feel it's the main set back #######
#        brush2 = random.sample(brush2, int(len(brush2)/10))
#        print 'size of brush 2 after sampling... '
#        print len(brush2)
        ###############################################################
        ###################################################################################
        ##################### Here I implement Topological Cost ###########################
        ###################################################################################

        half_side = rospy.get_param('radius')
        topo_gain1 = dict()
        for goal in brush1:
            xx = goal[0]/resolution
            yy = goal[1]/resolution
            rays_len = numpy.full([8], rospy.get_param('radius'))

            line = list(bresenham(int(xx), int(yy), int(xx), int(yy + half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[0] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx),int(yy - half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[1] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx + half_side/resolution),int(yy)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[2] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx - half_side/resolution),int(yy)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[3] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx + half_side/resolution),int(yy + half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[4] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx + half_side/resolution),int(yy - half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[5] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx - half_side/resolution),int(yy + half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[6] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx - half_side/resolution),int(yy - half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[7] = len(line[0: idx]) * resolution
                    break

            #topo_gain[goal] = sum(rays_len)/len(rays_len)
            topo_gain1[goal] = sum(rays_len) #/len(rays_len)
            #rospy.loginfo('The topo gain for goal = [%f,%f] is %f', xx, yy, topo_gain[goal])

        ###################################################################################
        ##################### Here I implement Topological Cost ###########################
        ###################################################################################

        half_side = rospy.get_param('radius')
        topo_gain2 = dict()
        for goal in brush2:
            xx = goal[0]/resolution
            yy = goal[1]/resolution
            rays_len = numpy.full([8], rospy.get_param('radius'))

            line = list(bresenham(int(xx), int(yy), int(xx), int(yy + half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[0] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx),int(yy - half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[1] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx + half_side/resolution),int(yy)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[2] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx - half_side/resolution),int(yy)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[3] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx + half_side/resolution),int(yy + half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[4] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx + half_side/resolution),int(yy - half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[5] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx - half_side/resolution),int(yy + half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[6] = len(line[0: idx]) * resolution
                    break

            line = list(bresenham(int(xx),int(yy),int(xx - half_side/resolution),int(yy - half_side/resolution)))
            for idx,coord in enumerate(line):
                if ogm[coord[0] - origin['x_px']][coord[1] - origin['y_px']] > 80:
                    rays_len[7] = len(line[0: idx]) * resolution
                    break

            #topo_gain[goal] = sum(rays_len)/len(rays_len)
            topo_gain2[goal] = sum(rays_len) #/len(rays_len)
            #rospy.loginfo('The topo gain for goal = [%f,%f] is %f', xx, yy, topo_gain[goal])

        ##################################################################################
        ##################################################################################
        ##################################################################################

        ## Sort Topo Gain goal ##

        #sorted_topo_gain = sorted(topo_gain.items(), key=operator.itemgetter(1), reverse = True)
        #rospy.loginfo('the length of sorted_topo_gain is %d !!', len(sorted_topo_gain))


        distance_map1 = dict()

        for goal in brush1:
            #print goal
            #goal = list(goal)
            #rospy.loginfo('goal is = [%f,%f]!!', goal[0], goal[1])
            dist = math.hypot(goal[0] - robotPose1['x'], goal[1] - robotPose1['y'])
            distance_map1[goal] = dist


        rospy.loginfo('the length of distance map1 is %d !!', len(distance_map1))

        distance_map2 = dict()

        for goal in brush2:
            #print goal
            #goal = list(goal)
            #rospy.loginfo('goal is = [%f,%f]!!', goal[0], goal[1])
            dist = math.hypot(goal[0] - robotPose2['x'], goal[1] - robotPose2['y'])
            distance_map2[goal] = dist


        rospy.loginfo('the length of distance map2 is %d !!', len(distance_map2))

        ######################################################################################
        ##################### Here I calculate the gain of my Goals ##########################
        ######################################################################################
        normTopo1 = dict()
        normDist1 = dict()
        for goal in brush1:
            if max(topo_gain1.values()) - min(topo_gain1.values()) == 0:
                normTopo1[(0,0)] = 0
            else:
                normTopo1[goal] = 1 - (topo_gain1[goal] - min(topo_gain1.values())) \
                            / (max(topo_gain1.values()) - min(topo_gain1.values()))
            if max(distance_map1.values()) - min(distance_map1.values()) == 0:
                normDist1[(0,0)] = 0
            else:
                normDist1[goal] = 1 - (distance_map1[goal] - max(distance_map1.values())) \
                            / (max(distance_map1.values()) - min(distance_map1.values()))

        # Calculate Priority Weight
        priorWeight1 = dict()
        for goal in brush1:
            pre = 1 * round((normTopo1[goal] / 0.5), 0) + \
                    8 * round((normDist1[goal] / 0.5), 0)
            priorWeight1[goal] = pre

        # Calculate smoothing factor
        smoothFactor1 = dict()
        for goal in brush1:
            coeff = (1 * (1 - normTopo1[goal]) + 8 * (1 - normDist1[goal]))  / (2**2 - 1)
            # coeff = (4 * (1 - wDistNorm[i]) + 2 * (1 - wCoveNorm[i]) + \
            #             (1 - wRotNorm[i])) / (2**3 - 1)
            smoothFactor1[goal] = coeff

        # Calculate costs
        goalGains1 = dict()
        for goal in brush1:
            goalGains1[goal] = priorWeight1[goal] * smoothFactor1[goal]

        # Choose goal with max gain 
        store_goal1 = set()
        for goal in brush1:
            if goalGains1[goal] == max(goalGains1.values()):
                store_goal1 = goal
                rospy.loginfo("[Main Node] Goal1 at = [%u, %u]!!!", goal[0], goal[1])
                rospy.loginfo("The Gain1 is = %f!!",goalGains1[goal] )
            else:
                pass
                #rospy.logwarn("[Main Node] Did not find any goals :( ...")
                #self.target = self.selectRandomTarget(ogm, coverage, brush2, \
                #                        origin, ogm_limits, resolution)
#
#        goal = list(goal)
        goal1 = list(store_goal1)
        print 'the ogm value is'
        print ogm[int(goal1[0] - origin['x_px'])][int(goal1[1] - origin['y_px'])]
        print goal
        self.target1 = goal1
        
        
        
        ######################################################################################
        ##################### Here I calculate the gain of my Goals ##########################
        ######################################################################################
        normTopo2 = dict()
        normDist2 = dict()
        for goal in brush2:
            if max(topo_gain2.values()) - min(topo_gain2.values()) == 0:
                normTopo2[(0,0)] = 0
            else:
                normTopo2[goal] = 1 - (topo_gain2[goal] - min(topo_gain2.values())) \
                            / (max(topo_gain2.values()) - min(topo_gain2.values()))
            if max(distance_map2.values()) - min(distance_map2.values()) == 0:
                normDist2[(0,0)] = 0
            else:
                normDist2[goal] = 1 - (distance_map2[goal] - max(distance_map2.values())) \
                            / (max(distance_map2.values()) - min(distance_map2.values()))

        # Calculate Priority Weight
        priorWeight2 = dict()
        for goal in brush2:
            pre = 1 * round((normTopo2[goal] / 0.5), 0) + \
                    8 * round((normDist2[goal] / 0.5), 0)
            priorWeight2[goal] = pre

        # Calculate smoothing factor
        smoothFactor2 = dict()
        for goal in brush2:
            coeff = (1 * (1 - normTopo2[goal]) + 8 * (1 - normDist2[goal]))  / (2**2 - 1)
            # coeff = (4 * (1 - wDistNorm[i]) + 2 * (1 - wCoveNorm[i]) + \
            #             (1 - wRotNorm[i])) / (2**3 - 1)
            smoothFactor2[goal] = coeff

        # Calculate costs
        goalGains2 = dict()
        for goal in brush2:
            goalGains2[goal] = priorWeight2[goal] * smoothFactor2[goal]

        # Choose goal with max gain 
        store_goal2 = set()
        for goal in brush2:
            if goalGains2[goal] == max(goalGains2.values()):
                store_goal2 = goal
                rospy.loginfo("[Main Node] Goal at = [%u, %u]!!!", goal[0], goal[1])
                rospy.loginfo("The Gain is = %f!!",goalGains2[goal] )
            else:
                pass
                #rospy.logwarn("[Main Node] Did not find any goals :( ...")
                #self.target = self.selectRandomTarget(ogm, coverage, brush2, \
                #                        origin, ogm_limits, resolution)
#
#        goal = list(goal)
        goal2 = list(store_goal2)
        print 'the ogm value is'
        print ogm[int(goal2[0] - origin['x_px'])][int(goal2[1] - origin['y_px'])]
        print goal2
        self.target2 = goal2

        return self.target1, self.target2

    
    def filterGoal(self, brush2, ogm, resolution, origin):
        throw = set()
        for goal in brush2:
            goal = list(goal)
            for i in range(-3,4):
                if int(goal[0]/resolution - origin['x']/resolution) + i >= len(ogm):
                    break
                if ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution) ] > 49 \
                or ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution) ] == -1:
                    goal = tuple(goal)
                    throw.add(goal)
                    break

        for goal in brush2:
            goal = list(goal)
            for j in range(-3,4):
                if int(goal[1]/resolution - origin['y']/resolution) + j >= len(ogm[0]):
                    break
                if ogm[int(goal[0]/resolution - origin['x']/resolution)]\
                [int(goal[1]/resolution - origin['y']/resolution) + j] > 49 \
                or ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution) ] == -1:
                    goal = tuple(goal)
                    throw.add(goal)
                    break
        return throw
