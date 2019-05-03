#/usr/bin/env python

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
from visualization_msgs.msg import Marker, MarkerArray

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


    def targetSelection(self, initOgm, coverage, origin, resolution, robotPose, force_random):
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("[Target Select Node] Robot_Pose[x, y, th] = [%f, %f, %f]", 
                    robotPose['x'], robotPose['y'], robotPose['th'])
        rospy.loginfo("[Target Select Node] OGM_Origin = [%i, %i]", origin['x'], origin['y'])
        rospy.loginfo("[Target Select Node] OGM_Size = [%u, %u]", initOgm.shape[0], initOgm.shape[1])

##        # willow stuff
#        ogm_limits = {}
#        ogm_limits['min_x'] = 150  # used to be 200
#        ogm_limits['max_x'] = 800  # used to be 800
#        ogm_limits['min_y'] = 200
#        ogm_limits['max_y'] = 800


#        # corridor
#        ogm_limits = {}
#        ogm_limits['min_x'] = 100  # used to be 200
#        ogm_limits['max_x'] = 500  # used to be 800
#        ogm_limits['min_y'] = 100
#        ogm_limits['max_y'] = 500

##        # Big Map
        ogm_limits = {}
        ogm_limits['min_x'] = 200  # used to be 200
        ogm_limits['max_x'] = 800  # used to be 800
        ogm_limits['min_y'] = 300
        ogm_limits['max_y'] = 1080
#        ogm_limits['max_y'] = 1080



        # publisher
        marker_pub = rospy.Publisher("/vis_nodes", MarkerArray, queue_size = 1)
        # Find only the useful boundaries of OGM. Only there calculations have meaning
#        ogm_limits = OgmOperations.findUsefulBoundaries(initOgm, origin, resolution)
        print ogm_limits

        # Blur the OGM to erase discontinuities due to laser rays
        #ogm = OgmOperations.blurUnoccupiedOgm(initOgm, ogm_limits)
        ogm = initOgm
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
        
        if len(nodes) == 0 and force_random: 
            brush = self.brush.coverageLimitsBrushfire(initOgm, 
                      coverage, robotPose, origin, resolution)
            throw = set()
            throw = self.filterGoal(brush, initOgm, resolution, origin)
            brush.difference_update(throw)
            goal = random.sample(brush, 1)
            
            rospy.loginfo("nodes are zero and random node chosen!!!!")
            th_rg = math.atan2(goal[0][1] - robotPose['y'], \
                    goal[0][0] - robotPose['x'])

            self.target = [goal[0][0], goal[0][1], th_rg]
            return self.target


        if len(nodes) == 0:
            brush = self.brush.coverageLimitsBrushfire(initOgm, 
                      coverage, robotPose, origin, resolution)
            throw = set()
            throw = self.filterGoal(brush, initOgm, resolution, origin)
            brush.difference_update(throw)
            distance_map = dict()
            distance_map = self.calcDist(robotPose, brush)
            self.target = min(distance_map, key = distance_map.get)
            
            th_rg = math.atan2(self.target[1] - robotPose['y'], \
                    self.target[0] - robotPose['x'])
            self.target = list(self.target)
            self.target.append(th_rg)
            return self.target

        if len(nodes) > 0:
            rospy.loginfo("[Main Node] Nodes ready! Elapsed time = %fsec", time.time() - tinit)
            rospy.loginfo("[Main Node] # of nodes = %u", len(nodes))

            # Remove previous targets from the list of nodes
            rospy.loginfo("[Main Node] Prev. target = [%u, %u]", self.previousTarget[0], \
                self.previousTarget[1])
            if len(nodes) > 1:
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
            self.publish_markers(marker_pub, vis_nodes)

        
        # Check distance From Other goal

#        for node in nodes:
#            node_x = node[0] * resolution + origin['x']
#            node_y = node[1] * resolution + origin['y']
#            dist = math.hypot(node_x - other_goal['x'], node_y - other_goal['y']) 
#            if dist < 5 and len(nodes) > 2:
#                nodes.remove(node)

    
        # Calculate topological cost
        rayLength = 800  # in pixels
        obstThres = 49
        wTopo = []
        dRad = []
        for i in range(8):
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
        for i in range(len(nodes)):
            nodesX.append(nodes[i][0])
            nodesY.append(nodes[i][1])
        for i in range(len(nodes)):
            #dist = math.sqrt((nodes[i][0] * resolution + origin['x'] - robotPose['x'])**2 + \
             #           (nodes[i][1] * resolution + origin['y'] - robotPose['y'])**2)
#            dist = math.sqrt((nodes[i][0] + origin['x_px'] - robotPose['x_px'])**2 + \
#                        (nodes[i][1]  + origin['y_px'] - robotPose['y_px'])**2)
            # Manhattan Dist
            dist = abs(nodes[i][0] + origin['x_px'] - robotPose['x_px']) + \
                   abs(nodes[i][1] + origin['y_px'] - robotPose['y_px'])

            # numpy.var is covariance
#            tempX = ((robotPose['x'] - nodesX[i] * resolution + origin['x'])**2) / (2 * numpy.var(nodesX))
#            tempY = ((robotPose['y'] - nodesY[i] * resolution + origin['y'])**2) / (2 * numpy.var(nodesY))
            tempX = ((robotPose['x_px'] - nodesX[i] + origin['x_px'])**2) / (2 * numpy.var(nodesX))
            tempY = ((robotPose['y_px'] - nodesY[i] + origin['y_px'])**2) / (2 * numpy.var(nodesY))

            try:
                temp = 1 - math.exp(tempX + tempY) + 0.001 # \epsilon << 1
            except OverflowError:
                print 'OverflowError!!!'
                temp = 10**30
            gaussCoeff = 1 / temp
            #gaussCoeff = 1               
            wDist.append(dist * gaussCoeff)

        #for i in range(len(nodes)):
        #    rospy.logwarn("Distance Cost is: %f ",wDist[i])

        #return self.target

        # Calculate coverage cost
        dSamp = 1 / resolution
        wCove = []
        for k in range(len(nodes)):
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
        for i in range(len(nodes)):
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
        for i in range(len(nodes)):
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

#        rospy.logwarn("Printing TopoNorm....\n")
#        print wTopoNorm

#        rospy.logwarn("Printing DistNorm....\n")
#        print wDistNorm

#        rospy.logwarn("Printing CoveNorm....\n")
#        print wCoveNorm

#        rospy.logwarn("Printing RotNorm....\n")
#        print wRotNorm




        # Calculate Priority Weight
        priorWeight = []
        for i in range(len(nodes)):
            #pre = round((wDistNorm[i] / 0.5), 0)
            pre = 1 * round((wTopoNorm[i] / 0.5), 0) + \
                    8 * round((wDistNorm[i] / 0.5), 0) + \
                    4 * round((wCoveNorm[i] / 0.5), 0) + \
                    2 * round((wRotNorm[i] / 0.5), 0)
            #pre = 1 * round((wDistNorm[i] / 0.5), 0) + \
            #         2 * round((wTopoNorm[i] / 0.5), 0) 
           #          + round((wRotNorm[i] / 0.5), 0)
            priorWeight.append(pre)

        # Calculate smoothing factor
        smoothFactor = []
        for i in range(len(nodes)):
            #coeff = 1 - wDistNorm[i]           
            coeff = (1 * (1 - wTopoNorm[i]) + 8 * (1 - wDistNorm[i]) + \
                        4 * (1 - wCoveNorm[i]) + 2 * (1 - wRotNorm[i])) / (2**4 - 1)
            #coeff = ((1 - wDistNorm[i]) + 2 * (1 - wTopoNorm[i])) / (2**2 - 1)
            smoothFactor.append(coeff)

        # Calculate costs
        self.costs = []
        for i in range(len(nodes)):
            self.costs.append(smoothFactor[i] * priorWeight[i])

#        print 'len nodes is:'
#        print len(nodes) 
    
        goals_and_costs = zip(nodes, self.costs)
        #print goals_and_costs

        goals_and_costs.sort(key = lambda t: t[1], reverse = True)
        #sorted(goals_and_costs, key=operator.itemgetter(1))
#        print goals_and_costs
       # ind = random.randrange(0,min(6, len(nodes)))
        rospy.loginfo("[Main Node] Raw node = [%u, %u]", goals_and_costs[0][0][0], goals_and_costs[0][0][1])
        tempX = goals_and_costs[0][0][0] * resolution + origin['x']
        tempY = goals_and_costs[0][0][1] * resolution + origin['y']
        th_rg = math.atan2(tempY - robotPose['y'], \
                    tempX - robotPose['x'])
        self.target = [tempX, tempY, th_rg]
        rospy.loginfo("[Main Node] Eligible node found at [%f, %f]", 
                        self.target[0], self.target[1])
        rospy.loginfo("[Main Node] Node Index: %u", i)
        rospy.loginfo("[Main Node] Node Cost = %f", goals_and_costs[0][1])
        rospy.loginfo("-----------------------------------------")
        self.previousTarget = [goals_and_costs[0][0][0], goals_and_costs[0][0][1]]
        
        # pick Random node!!
        if force_random:
            ind = random.randrange(0,len(nodes))
            rospy.loginfo('index is: %d', ind)
            rospy.loginfo('Random raw node is: [%u, %u]', nodes[ind][0], nodes[ind][1])
            tempX = nodes[ind][0] * resolution + origin['x']
            tempY = nodes[ind][1] * resolution + origin['y']
            th_rg = math.atan2(tempY - robotPose['y'], \
                    tempX - robotPose['x'])
            self.target = [tempX, tempY, th_rg]
            rospy.loginfo("[Main Node] Random target found at [%f, %f]", 
                            self.target[0], self.target[1])
            rospy.loginfo("-----------------------------------------")
            return self.target

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



    def publish_markers(self, marker_pub, vis_nodes):
        markers = MarkerArray()
        c = 0
        for n in vis_nodes:
            c += 1
            msg = Marker()
            msg.header.frame_id = "map"
            msg.ns = "lines"
            msg.action = msg.ADD
            msg.type = msg.CUBE
            msg.id = c
            #msg.scale.x = 1.0
            #msg.scale.y = 1.0
            #msg.scale.z = 1.0
            # I guess I have to take into consideration resolution too
            msg.pose.position.x = n[0]
            msg.pose.position.y = n[1]
            msg.pose.position.z = 0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.05
            msg.pose.orientation.w = 0.05
            msg.scale.x = 0.2
            msg.scale.y = 0.2
            msg.scale.z = 0.2
            msg.color.a = 1.0 # Don't forget to set the alpha!
            msg.color.r = 0.0
            msg.color.g = 1.0
            msg.color.b = 0.0
    #        print 'I publish msg now!!!'
            markers.markers.append(msg)
            
        marker_pub.publish(markers)
#
        return

    def selectRandomTarget(self, ogm, brush, origin, ogmLimits, resolution):
        rospy.logwarn("[Main Node] Random Target Selection!")
        target = [-1, -1]
        found = False
        while not found:
          x_rand = random.randint(0, int(ogm.shape[0] - 1))
          y_rand = random.randint(0, int(ogm.shape[1] - 1))
          if ogm[x_rand][y_rand] <= 49 and brush[x_rand][y_rand] != -1:# and \#coverage[x_rand][y_rand] != 100:
            tempX = x_rand * resolution + int(origin['x'])
            tempY = y_rand * resolution + int(origin['y'])
            target = [tempX, tempY]
            found = True
            rospy.loginfo("[Main Node] Random node selected at [%f, %f]", target[0], target[1])
            rospy.loginfo("-----------------------------------------")
            return self.target


    def calcDist(self, robotPose, brush):
        distance_map = dict()
        for goal in brush:
            dist = math.hypot(goal[0] - robotPose['x'], goal[1] - robotPose['y'])
            distance_map[goal] = dist
        return distance_map

    def filterGoal(self, brush2, ogm, resolution, origin):
        throw = set()
        for goal in brush2:
            goal = list(goal)
            for i in range(-9,10):
                if int(goal[0]/resolution - origin['x']/resolution) + i >= len(ogm):
                    break
                if ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution)] > 49 \
                or ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution)] == -1:
                    goal = tuple(goal)
                    throw.add(goal)
                    break

        for goal in brush2:
            goal = list(goal)
            for j in range(-9,10):
                if int(goal[1]/resolution - origin['y']/resolution) + j >= len(ogm[0]):
                    break
                if ogm[int(goal[0]/resolution - origin['x']/resolution)]\
                [int(goal[1]/resolution - origin['y']/resolution) + j] > 49 \
                or ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution)] == -1:
                    goal = tuple(goal)
                    throw.add(goal)
                    break

        for goal in brush2:
            goal = list(goal)
            for i in range(-9,10):
                if int(goal[0]/resolution - origin['x']/resolution) + i >= len(ogm) or \
                    int(goal[1]/resolution - origin['y']/resolution) + i >= len(ogm[0]):
                    break
                if ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution) + i] > 49 \
                or ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution) + i] == -1:
                    goal = tuple(goal)
                    throw.add(goal)
                    break


        for goal in brush2:
            goal = list(goal)
            for i in range(-9, 10):
                if int(goal[0]/resolution - origin['x']/resolution) + i >= len(ogm) or \
                    int(goal[1]/resolution - origin['y']/resolution) + i >= len(ogm[0]):
                    break
                if ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution) - i] > 49 \
                or ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution) - i] == -1:
                    goal = tuple(goal)
                    throw.add(goal)
                    break

        return throw
