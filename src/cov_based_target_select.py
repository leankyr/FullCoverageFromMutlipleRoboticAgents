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
        self.target = [-1, -1]
        self.previousTarget = [-1, -1]
        self.costs = []


    def targetSelection(self, initOgm, coverage, origin, resolution, robotPose):
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("[Target Select Node] Robot_Pose[x, y, th] = [%f, %f, %f]", \
                    robotPose['x'], robotPose['y'], robotPose['th'])
        rospy.loginfo("[Target Select Node] OGM_Origin = [%i, %i]", origin['x'], origin['y'])
        rospy.loginfo("[Target Select Node] OGM_Size = [%u, %u]", initOgm.shape[0], initOgm.shape[1])

        # Blur the OGM to erase discontinuities due to laser rays
        #ogm = OgmOperations.blurUnoccupiedOgm(initOgm, ogmLimits)
        ogm = initOgm

        rospy.loginfo("Calculating brush2....")

        brush2 = self.brush.coverageLimitsBrushfire(initOgm, coverage, robotPose, origin, resolution )

        print 'size of brush2:'
        print len(brush2)
        min_dist = 10**24
        store_goal = ()
        throw = set()

        throw = self.filterGoal(brush2, ogm, resolution, origin)

        print 'size of throw:'
        print len(throw)

        brush2.difference_update(throw)

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
        topo_gain = dict()
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
            topo_gain[goal] = sum(rays_len) #/len(rays_len)
            #rospy.loginfo('The topo gain for goal = [%f,%f] is %f', xx, yy, topo_gain[goal])

        ##################################################################################
        ##################################################################################
        ##################################################################################

        ## Sort Topo Gain goal ##

        #sorted_topo_gain = sorted(topo_gain.items(), key=operator.itemgetter(1), reverse = True)
        #rospy.loginfo('the length of sorted_topo_gain is %d !!', len(sorted_topo_gain))


        distance_map = dict()

        for goal in brush2:
            #print goal
            #goal = list(goal)
            #rospy.loginfo('goal is = [%f,%f]!!', goal[0], goal[1])
            dist = math.hypot(goal[0] - robotPose['x'], goal[1] - robotPose['y'])
            distance_map[goal] = dist


        rospy.loginfo('the length of distance map is %d !!', len(distance_map))


        ######################################################################################
        ##################### Here I calculate the gain of my Goals ##########################
        ######################################################################################
        normTopo = dict()
        normDist = dict()
        for goal in brush2:
            if max(topo_gain.values()) - min(topo_gain.values()) == 0:
                normTopo[(0,0)] = 0
            else:
                normTopo[goal] = 1 - (topo_gain[goal] - min(topo_gain.values())) \
                            / (max(topo_gain.values()) - min(topo_gain.values()))
            if max(distance_map.values()) - min(distance_map.values()) == 0:
                normDist[(0,0)] = 0
            else:
                normDist[goal] = 1 - (distance_map[goal] - min(distance_map.values())) \
                            / (max(distance_map.values()) - min(distance_map.values()))

        # Calculate Priority Weight
        priorWeight = dict()
        for goal in brush2:
            pre = 2 * round((normTopo[goal] / 0.5), 0) + \
                    1 * round((normDist[goal] / 0.5), 0)
            priorWeight[goal] = pre

        # Calculate smoothing factor
        smoothFactor = dict()
        for goal in brush2:
            coeff = (2 * (1 - normTopo[goal]) + 1 * (1 - normDist[goal]))  / (2**2 - 1)
            # coeff = (4 * (1 - wDistNorm[i]) + 2 * (1 - wCoveNorm[i]) + \
            #             (1 - wRotNorm[i])) / (2**3 - 1)
            smoothFactor[goal] = coeff

        # Calculate costs
        goalGains = dict()
        for goal in brush2:
            goalGains[goal] = priorWeight[goal] * smoothFactor[goal]

        # Choose goal with max gain 
        store_goal = set()
        for goal in brush2:
            if goalGains[goal] == max(goalGains.values()):
                store_goal = goal
                rospy.loginfo("[Main Node] Goal at = [%u, %u]!!!", goal[0], goal[1])
                rospy.loginfo("The Gain is = %f!!",goalGains[goal] )
            else:
                pass
                #rospy.logwarn("[Main Node] Did not find any goals :( ...")
                #self.target = self.selectRandomTarget(ogm, coverage, brush2, \
                #                        origin, ogm_limits, resolution)




#        sorted_dist_map = sorted(distance_map.items(), key=operator.itemgetter(1))
#
#        sorted_goal_list = list()
#        for key, value in sorted(distance_map.iteritems(), key=lambda (k,v): (v,k)):
#            sorted_goal_list.append(key)
#            pass
            #print "%s: %s" % (key, value)


#        sorted_goal_list_sampled = sorted_goal_list[0:len(sorted_goal_list):10]
        #print sorted_goal_list_top_10

#        stored_goal = list()
#        dist = 1000
#        for goal in distance_map:
#            if distance_map[goal] < dist:
#                dist = distance_map[goal]
#                stored_goal = goal
#
#        rospy.loginfo('The stored goal is = [%f,%f]!!' ,stored_goal[0], stored_goal[1])
#        rospy.loginfo('The distance is %f!!', distance_map[stored_goal])
#        rospy.loginfo('The gain is %f!!', topo_gain[stored_goal])
#        #rand_target = random.choice(distance_map.keys())
#        #goal = rand_target
#        ind = random.randrange(0,min(4,len(sorted_goal_list)))
#        print 'ind is'
#        print ind
#        goal = sorted_goal_list[ind]
#        print 'the dist is'
#        print distance_map[goal]
#
#        goal = list(goal)
        goal = list(store_goal)
        print 'the ogm value is'
        print ogm[int(goal[0] - origin['x_px'])][int(goal[1] - origin['y_px'])]
        print goal
        self.target = goal
        return self.target

    def selectRandomTarget(self, ogm, brush, origin, ogmLimits, resolution):
        rospy.logwarn("[Main Node] Random Target Selection!")
        target = [-1, -1]
        found = False
        while not found:
          x_rand = random.randint(0, int(ogm.shape[0] - 1))
          y_rand = random.randint(0, int(ogm.shape[1] - 1))
          if ogm[x_rand][y_rand] <= 49 and brush[x_rand][y_rand] > 3:# and \#coverage[x_rand][y_rand] != 100:
            tempX = x_rand * resolution + int(origin['x'])
            tempY = y_rand * resolution + int(origin['y'])
            target = [tempX, tempY]
            found = True
            rospy.loginfo("[Main Node] Random node selected at [%f, %f]", target[0], target[1])
            rospy.loginfo("-----------------------------------------")
            return self.target



    def rotateRobot(self):
        velocityMsg = Twist()
        angularSpeed = 0.3
        relativeAngle = 2*math.pi
        currentAngle = 0

        rospy.loginfo("Roatating robot...")
        velocityMsg.linear.x = 0
        velocityMsg.linear.y = 0
        velocityMsg.linear.z = 0
        velocityMsg.angular.x = 0
        velocityMsg.angular.y = 0
        velocityMsg.angular.z = angularSpeed

        t0 = rospy.Time.now().to_sec()
        rospy.logwarn(rospy.get_caller_id() + ": Rotate Robot! Please wait...")
        while currentAngle < relativeAngle:
            self.velocityPub.publish(velocityMsg)
            t1 = rospy.Time.now().to_sec()
            currentAngle = angularSpeed * (t1 - t0)

        velocityMsg.angular.z = 0
        self.velocityPub.publish(velocityMsg)
        rospy.logwarn(rospy.get_caller_id() + ": Robot Rotation OVER!")
        return



    def publish_markers(self, marker_pub, pose_x, pose_y):
        msg = Marker()
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = rospy.Time(0)
        msg.ns = "lines"
        msg.action = msg.ADD
        msg.type = msg.CUBE
        msg.id = 0
        #msg.scale.x = 1.0
        #msg.scale.y = 1.0
        #msg.scale.z = 1.0
        # I guess I have to take into consideration resolution too
        msg.pose.position.x = pose_x;
        msg.pose.position.y = pose_y;
        msg.pose.position.z = 0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.05;
        msg.pose.orientation.w = 0.05;
        msg.scale.x = 0.1;
        msg.scale.y = 0.1;
        msg.scale.z = 0.1;
        msg.color.a = 1.0; # Don't forget to set the alpha!
        msg.color.r = 0.0;
        msg.color.g = 1.0;
        msg.color.b = 0.0;

        marker_pub.publish(msg)
        return

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
