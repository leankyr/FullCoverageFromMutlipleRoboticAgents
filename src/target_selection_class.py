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
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from brushfires import Brushfires
from topology import Topology

visited = set()
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


    def targetSelection(self, initOgm,costmap, origin, resolution, robotPose):
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("[Target Select Node] Robot_Pose[x, y, th] = [%f, %f, %f]", \
                    robotPose['x'], robotPose['y'], robotPose['th'])
        rospy.loginfo("[Target Select Node] OGM_Origin = [%i, %i]", origin['x'], origin['y'])
        rospy.loginfo("[Target Select Node] OGM_Size = [%u, %u]", initOgm.shape[0], initOgm.shape[1])

        ogmLimits = {}
        ogmLimits['min_x'] = -1
        ogmLimits['max_x'] = -1
        ogmLimits['min_y'] = -1
        ogmLimits['max_y'] = -1

        # Find only the useful boundaries of OGM. Only there calculations have meaning
        ogmLimits = OgmOperations.findUsefulBoundaries(initOgm, origin, resolution)
        print (ogmLimits)
        while ogmLimits['min_x'] < 0 or ogmLimits['max_x'] < 0 or \
                ogmLimits['min_y'] < 0 or ogmLimits['max_y'] < 0:
            rospy.logwarn("[Main Node] Negative OGM limits. Retrying...")
            ogmLimits = OgmOperations.findUsefulBoundaries(initOgm, origin, resolution)
            ogmLimits['min_x'] = abs(int(ogmLimits['min_x']))
            ogmLimits['min_y'] = abs(int(ogmLimits['min_y']))
            ogmLimits['max_x'] = abs(int(ogmLimits['max_x']))
            ogmLimits['max_y'] = abs(int(ogmLimits['max_y']))
        rospy.loginfo("[Target Select] OGM_Limits[x] = [%i, %i]", \
                            ogmLimits['min_x'], ogmLimits['max_x'])
        rospy.loginfo("[Target Select] OGM_Limits[y] = [%i, %i]", \
                            ogmLimits['min_y'], ogmLimits['max_y'])

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(initOgm, ogmLimits)

        # Calculate Brushfire field
        #itime = time.time()
        #brush = self.brush.obstaclesBrushfireCffi(ogm, ogmLimits)
        #rospy.loginfo("[Target Select] Brush ready! Elapsed time = %fsec", time.time() - itime)

        #obst = self.brush.coverageLimitsBrushfire2(initOgm,ogm,robotPose,origin, resolution )
        rospy.loginfo("Calculating brush2....")
        brush2 = self.brush.coverageLimitsBrushfire(ogm,ogm,robotPose,origin, resolution )

        #robotPosePx = []
        #robotPosePx[0] = robotPose['x']/resolution
        #robotPosePy[1] = robotPose['y']/resolution
        #print 'brush2 is :'
        #print brush2
        min_dist = 10**24
        store_goal = ()
       # rospy.loginfo("finding the difference between the two sets...")
       # brush2.difference(visited)
        #max_dist = random.randrange(1,10)
        #rospy.loginfo("max_dist for this it is: %d ", max_dist)
        for goal in brush2:
            #goal = list(goal)
            #dist = math.hypot(goal[0] - robotPose['x'],goal[1] - robotPose['y'])\
            # manhattan dist
            dist = abs(goal[0] - robotPose['x']) + abs(goal[1] - robotPose['y'])
            if dist < min_dist: #and dist > max_dist:
                found = False
                ### in this loop I try to see if goal is a frontier but does not seem to work
                for i in range(-15,16):
                    # this found shot worked in the end
                    for j in range(-15,16):
                        if ogm[goal[0]/resolution - origin['x'] + i][goal[1]/resolution - origin['y'] + j]\
                                    != -1:
                         #if costmap[goal[0]][goal[1]] == 100:
                            print "continued!!"
                            found = True
                            break
                    if found == True:
                        break

                if found == True:
                    continue

                if costmap[goal[0]/resolution - origin['x']][goal[1]/resolution - origin['y']] >= 128:
                    print 'Goal on obstacle!!!'
                    continue

                #goal[0] = round(goal[0],2)
                #goal[1] = round(goal[1],2)
                #goal = tuple(goal)
                #if goal in visited:
                #    print "goal in Banlist!!"
                    #store_goal = self.selectRandomTarget(self, ogm, brush2, origin, ogmLimits, resolution)
                #    continue

                #visited.add(goal)
                # print "ban list is:"
                # print ban_list
                min_dist = dist
                visited.add(goal)
                store_goal = goal


        #rospy.loginfo(" Costmap vaule of our goal is: %d",costmap[store_goal[0]][store_goal[1]] )
        rospy.loginfo("min dist is %f" , min_dist)

        #rospy.loginfo("Costmap Value is: %d", costmap[goal[0]/resolution - origin['x']][goal[1]/resolution\
        #                - origin['y']])
       # store_goal[0] = store_goal[0] + random.randrange(-2,2)
       # store_goal[1] = store_goal[1] + random.randrange(-2,2)
        #visited.add(store_goal)
        store_goal = list(store_goal)
        rospy.loginfo("goal BEFORE unifrom is: goal = [%f,%f]" , store_goal[0],store_goal[1])
        if min_dist < 0.5:
            while(1):
                store_goal[0] = random.uniform(store_goal[0] - 2,store_goal[0] + 2)
                store_goal[1] = random.uniform(store_goal[1] - 2,store_goal[1] + 2)
                if costmap[store_goal[0]][store_goal[1]] < 128:
                    break

        rospy.loginfo("goal AFTER unifrom is: goal = [%f,%f]" , store_goal[0],store_goal[1])
        self.target = store_goal

        print self.target
        # print brush2
        #print brush
        # Calculate skeletonization
        #itime = time.time()
        #skeleton = self.topo.skeletonizationCffi(ogm, origin, resolution, ogmLimits)
        #rospy.loginfo("[Target Select] Skeleton ready! Elapsed time = %fsec", time.time() - itime)

        # Find topological graph (nodes coordinates are in PIXELS!!!)
        #itime = time.time()
        #nodes = self.topo.topologicalNodes(ogm, skeleton, origin, resolution, \
        #                                    brush, ogmLimits)
        #for i in range(len(nodes)):
        #    rospy.loginfo("[Target Select Nodes are: %d]",nodes[i])
        #rospy.loginfo("[Target Select] Number of nodes: %d",len(nodes))
        #print nodes

        #if len(nodes) == 0:
        #    self.target == self.selectRandomTarget(ogm,brush,origin,ogmLimits,resolution)
       
        return self.target
        #return [-3,-2]

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






