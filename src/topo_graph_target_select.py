#!/usr/bin/env python


import rospy
import actionlib
import scipy
import time
import math
import tf
import numpy
import random
import bresenham

from utilities import OgmOperations
from utilities import RvizHandler
from utilities import Print
from utilities import Cffi

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from brushfires import Brushfires
from topology import Topology
from visualization_msgs.msg import Marker

from timeit import default_timer as timer

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

        ogm_limits = {}
        ogm_limits['min_x'] = 10
        ogm_limits['max_x'] = 650
        ogm_limits['min_y'] = 30
        ogm_limits['max_y'] = 440

        # Find only the useful boundaries of OGM. Only there calculations have meaning
#        ogm_limits = OgmOperations.findUsefulBoundaries(initOgm, origin, resolution)
        print ogm_limits
#        while ogm_limits['min_x'] < 0 or ogm_limits['max_x'] < 0 or \
#                ogm_limits['min_y'] < 0 or ogm_limits['max_y'] < 0:
#            rospy.logwarn("[Main Node] Negative OGM limits. Retrying...")
#            ogm_limits = OgmOperations.findUsefulBoundaries(initOgm, origin, resolution)
#            ogm_limits['min_x'] = abs(int(ogm_limits['min_x']))
#            ogm_limits['min_y'] = abs(int(ogm_limits['min_y']))
#            ogm_limits['max_x'] = abs(int(ogm_limits['max_x']))
#            ogm_limits['max_y'] = abs(int(ogm_limits['max_y']))
#        rospy.loginfo("[Target Select] OGM_Limits[x] = [%i, %i]", \
#                            ogm_limits['min_x'], ogm_limits['max_x'])
#        rospy.loginfo("[Target Select] OGM_Limits[y] = [%i, %i]", \
#                            ogm_limits['min_y'], ogm_limits['max_y'])

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(initOgm, ogm_limits)
        #ogm = initOgm
        # find brushfire field
        brush2 = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)

        # Calculate skeletonization
        skeleton = self.topo.skeletonizationCffi(ogm, origin, resolution, ogm_limits)

        # Find Topological Graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                                    resolution, brush2, ogm_limits)
        # print took to calculate....
        rospy.loginfo("Calculation time: %s",str(time.time() - tinit))

        if len(nodes) > 0:
            rospy.loginfo("[Main Node] Nodes ready! Elapsed time = %fsec", time.time() - tinit)
            rospy.loginfo("[Main Node] # of nodes = %u", len(nodes))

            # Remove previous targets from the list of nodes
            rospy.loginfo("[Main Node] Prev. target = [%u, %u]", self.previousTarget[0], \
                            self.previousTarget[1])
            nodes = [i for i in nodes if i != self.previousTarget]

            vis_nodes = []
            for n in nodes:
                vis_nodes.append([
                    n[0] * resolution + origin['x'],
                    n[1] * resolution + origin['y']
                ])
            RvizHandler.printMarker(\
                vis_nodes,\
                1, # Type: Arrow
                0, # Action: Add
                "map", # Frame
                "art_topological_nodes", # Namespace
                [0.3, 0.4, 0.7, 0.5], # Color RGBA
                0.1 # Scale
            )


#        print nodes
#        #print nodes[0]
#        ind = random.randrange(0,len(nodes))
#        print ind
##
#        goal = list()
#        goal = [nodes[ind][0]*resolution + origin['x'], nodes[ind][1]*resolution + origin['y']]
#        print 'the goal is:'
#        print goal
##
#        self.target = goal

        # Calculate topological cost
        rayLength = 20  # in pixels
        obstThres = 49
        wTopo = []
        dRad = []
        for i in range(0, 8):
            dRad.append(rayLength)
        for k in range(0, len(nodes)):
            # Determine whether the ray length passes the OGM limits
            if nodes[k][0] + rayLength > ogm.shape[0]:
                self.xLimitUp = ogm.shape[0] - 1
            else:
                self.xLimitUp = nodes[k][0] + rayLength
            if nodes[k][0] - rayLength < 0:
                self.xLimitDown = 0
            else:
                self.xLimitDown = nodes[k][0] - rayLength
            if nodes[k][1] + rayLength > ogm.shape[1]:
                self.yLimitUp = ogm.shape[1] - 1
            else:
                self.yLimitUp = nodes[k][1] + rayLength
            if nodes[k][1] - rayLength < 0:
                self.yLimitDown = 0
            else:
                self.yLimitDown = nodes[k][1] - rayLength
            #### Here We Do the Ray Casting ####  
            # Find the distance between the node and obstacles
            for i in range(nodes[k][0], self.xLimitUp):
                if ogm[i][nodes[k][1]] > obstThres:
                    dRad[0] = i - nodes[k][0]
                    break
            for i in range(self.xLimitDown, nodes[k][0]):
                if ogm[i][nodes[k][1]] > obstThres:
                    dRad[1] = nodes[k][0] - i
                    break
            for i in range(nodes[k][1], self.yLimitUp):
                if ogm[nodes[k][0]][i] > obstThres:
                    dRad[2] = i - nodes[k][1]
                    break
            for i in range(self.yLimitDown, nodes[k][1]):
                if ogm[nodes[k][0]][i] > obstThres:
                    dRad[3] = nodes[k][1] - i
                    break
            for i in range(nodes[k][0], self.xLimitUp):
                for j in range(nodes[k][1], self.yLimitUp):
                    if ogm[i][j] > obstThres:
                        crosscut = \
                            math.sqrt((i - nodes[k][0])**2 + (j - nodes[k][1])**2)
                        dRad[4] = crosscut
                        break
                    else:
                        break
                if ogm[i][j] > obstThres:
                    break
            for i in range(self.xLimitDown, nodes[k][0]):
                for j in range(self.yLimitDown, nodes[k][1]):
                    if ogm[i][j] > obstThres:
                        crosscut = \
                            math.sqrt((nodes[k][0] - i)**2 + (nodes[k][1] - j)**2)
                        dRad[5] = crosscut
                        break
                    else:
                        break
                if ogm[i][j] > obstThres:
                    break
            for i in range(nodes[k][0], self.xLimitUp):
                for j in range(self.yLimitDown, nodes[k][1]):
                    if ogm[i][j] > obstThres:
                        crosscut = \
                            math.sqrt((i - nodes[k][0])**2 + (nodes[k][1] - j)**2)
                        dRad[6] = crosscut
                        break
                    else:
                        break
                if ogm[i][j] > obstThres:
                    break
            for i in range(self.xLimitDown, nodes[k][0]):
                for j in range(nodes[k][1], self.yLimitUp):
                    if ogm[i][j] > obstThres:
                        crosscut = \
                            math.sqrt((nodes[k][0] - i)**2 + (j - nodes[k][1])**2)
                        dRad[7] = crosscut
                        break
                    else:
                        break
                if ogm[i][j] > obstThres:
                    break

            wTopo.append(sum(dRad) / 8)
#        for i in range(len(nodes)):
#            rospy.logwarn("Topo Cost is: %f ",wTopo[i])

        # Calculate distance cost
        wDist = []
        nodesX = []
        nodesY = []
        for i in range(0, len(nodes)):
            nodesX.append(nodes[i][0])
            nodesY.append(nodes[i][1])
        for i in range(0, len(nodes)):
            dist = math.sqrt((nodes[i][0] - robotPose['x_px'])**2 + \
                        (nodes[i][1] - robotPose['y_px'])**2)
            # numpy.var is covariance
            tempX = ((robotPose['x_px'] - nodesX[i])**2) / (2 * numpy.var(nodesX))
            tempY = ((robotPose['y_px'] - nodesY[i])**2) / (2 * numpy.var(nodesY))
            temp = 1 - math.exp(tempX + tempY)
            gaussCoeff = 1 / temp
            wDist.append(dist * gaussCoeff)

#        for i in range(len(nodes)):
#            rospy.logwarn("Distance Cost is: %f ",wDist[i])

        #return self.target

        # Calculate coverage cost
        dSamp = 1 / resolution
        wCove = []
        for k in range(0, len(nodes)):
            athr = 0
            for i in range(-1, 1):
                indexX = int(nodes[k][0] + i * dSamp)
                if indexX >= 0:
                    for j in range(-1, 1):
                        indexY = int(nodes[k][1] + j * dSamp)
                        if indexY >= 0:
                            athr += coverage[indexX][indexY]
            wCove.append(athr)

#        for i in range(len(nodes)):
#            rospy.logwarn("Cove Cost is: %f ",wCove[i])

        # Calculate rotational cost
        wRot = []
        for i in range(0, len(nodes)):
            dTh = math.atan2(nodes[i][1] - robotPose['y_px'], \
                        nodes[i][0] - robotPose['x_px']) - robotPose['th']
            wRot.append(dTh)

#        for i in range(len(nodes)):
#            rospy.logwarn("Rot Cost is: %f ",wRot[i])

        # Normalize costs
        wTopoNorm = []
        wDistNorm = []
        wCoveNorm = []
        wRotNorm = []
        for i in range(0, len(nodes)):
            if max(wTopo) - min(wTopo) == 0:
                normTopo = 0
            else:
                normTopo = 1 - (wTopo[i] - min(wTopo)) / (max(wTopo) - min(wTopo))
            if max(wDist) - min(wDist) == 0:
                normDist = 0
            else:
                normDist = 1 - (wDist[i] - min(wDist)) / (max(wDist) - min(wDist))
            if max(wCove) - min(wCove) == 0:
                normCove = 0
            else:
                normCove = 1 - (wCove[i] - min(wCove)) / (max(wCove) - min(wCove))
            if max(wRot) - min(wRot) == 0:
                normRot = 0
            else:
                normRot = 1 - (wRot[i] - min(wRot)) / (max(wRot) - min(wRot))
            wTopoNorm.append(normTopo)
            wDistNorm.append(normDist)
            wCoveNorm.append(normCove)
            wRotNorm.append(normRot)

        # Calculate Priority Weight
        priorWeight = []
        for i in range(0, len(nodes)):
            pre = 8 * round((wTopoNorm[i] / 0.5), 0) + \
                    4 * round((wDistNorm[i] / 0.5), 0) + \
                    2 * round((wCoveNorm[i] / 0.5), 0) \
                    + round((wRotNorm[i] / 0.5), 0)
            # pre = 4 * round((wDistNorm[i] / 0.5), 0) + \
            #         2 * round((wCoveNorm[i] / 0.5), 0) \
            #         + round((wRotNorm[i] / 0.5), 0)
            priorWeight.append(pre)

        # Calculate smoothing factor
        smoothFactor = []
        for i in range(0, len(nodes)):
            coeff = (8 * (1 - wTopoNorm[i]) + 4 * (1 - wDistNorm[i]) + \
                        2 * (1 - wCoveNorm[i]) + (1 - wRotNorm[i])) / (2**4 - 1)
            # coeff = (4 * (1 - wDistNorm[i]) + 2 * (1 - wCoveNorm[i]) + \
            #             (1 - wRotNorm[i])) / (2**3 - 1)
            smoothFactor.append(coeff)

        # Calculate costs
        self.costs = []
        for i in range(0, len(nodes)):
            self.costs.append(smoothFactor[i] * priorWeight[i])

        for i in range(0, len(nodes)):
            if self.costs[i] == max(self.costs):
                rospy.loginfo("[Main Node] Raw node = [%u, %u]", nodes[i][0], nodes[i][1])
                tempX = nodes[i][0] * resolution + int(origin['x'])
                tempY = nodes[i][1] * resolution + int(origin['y'])
                self.target = [tempX, tempY]
                rospy.loginfo("[Main Node] Eligible node found at [%f, %f]", \
                                self.target[0], self.target[1])
                rospy.loginfo("[Main Node] Node Index: %u", i)
                rospy.loginfo("[Main Node] Node Cost = %f", self.costs[i])
                rospy.loginfo("-----------------------------------------")
                self.previousTarget = [nodes[i][0], nodes[i][1]]
                #(self.target[0], self.target[1])
        # while i < len(nodes):
        #     if self.costs[i] == max(self.costs):
        #         tempX = nodes[i][0] * resolution + int(origin['x'])
        #         tempY = nodes[i][1] * resolution + int(origin['y'])
        #         self.target = [tempX, tempY]
        #         if self.target[0] == self.previousTarget[0] and \
        #                 self.target[1] == self.previousTarget[1]:
        #             self.costs[i] = 0
        #             i = -1
        #         else:
        #             rospy.loginfo("Eligible node found at [%f, %f]", \
        #                             self.target[0], self.target[1])
        #             rospy.loginfo("Node Index: %u", i)
        #             rospy.loginfo("Node Cost = %f", self.costs[i])
        #             self.previousTarget = self.target
        #     i = i + 1
            else:
                pass
                #rospy.logwarn("[Main Node] Did not find any nodes...")
                #self.target = self.selectRandomTarget(ogm, coverage, brush2, \
                #                        origin, ogm_limits, resolution)

        return self.target
        #return [-1,-1]





    def selectRandomTarget(self, ogm, coverage, brush, origin, ogm_limits, resolution):
        rospy.logwarn("[Main Node] Random Target Selection!")
        target = [-2, -2]
        found = False
        while not found:
          x_rand = random.randint(0, int(ogm.shape[0] - 1))
          y_rand = random.randint(0, int(ogm.shape[1] - 1))
          if ogm[x_rand][y_rand] <= 49 and brush[x_rand][y_rand] > 3 and coverage[x_rand][y_rand] != 100:
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
