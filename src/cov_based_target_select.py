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
        #ogm = OgmOperations.blurUnoccupiedOgm(initOgm, ogmLimits)
        ogm = initOgm
#        for i in range(len(ogm)):
#            for j in range(len(ogm)):
#                if ogm[i][j] == 100:
#                    rospy.loginfo('i,j = [%d, %d]', i, j)
#
        # Calculate Brushfire field
        #itime = time.time()
        #brush = self.brush.obstaclesBrushfireCffi(ogm, ogmLimits)
        #rospy.loginfo("[Target Select] Brush ready! Elapsed time = %fsec", time.time() - itime)

        #obst = self.brush.coverageLimitsBrushfire2(initOgm,ogm,robotPose,origin, resolution )
        rospy.loginfo("Calculating brush2....")
        # brush = self.brush.obstaclesBrushfireCffi(ogm,ogmLimits)
        brush2 = self.brush.coverageLimitsBrushfire(initOgm, coverage, robotPose, origin, resolution )


        #goals = self.brush.closestUncoveredBrushfire(ogm, ogm, brush, robotPose, origin, resolution  )
        #robotPosePx = []
        #robotPosePx[0] = robotPose['x']/resolution
        #robotPosePy[1] = robotPose['y']/resolution
        print 'size of brush2:'
        print len(brush2)
        min_dist = 10**24
        store_goal = ()
       # rospy.loginfo("finding the difference between the two sets...")
       # brush2.difference(visited)
        #max_dist = random.randrange(1,10)
        #rospy.loginfo("max_dist for this it is: %d ", max_dist)
        throw = set()
        for goal in brush2:
            goal = list(goal)
            for i in range(-3,4):
                if int(goal[0]/resolution - origin['x']/resolution) + i >= len(ogm):
                    break
                if ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                [int(goal[1]/resolution - origin['y']/resolution) ] > 49: #\
                #or ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                #[int(goal[1]/resolution - origin['y']/resolution) ] == -1:
                    goal = tuple(goal)
                    throw.add(goal)
                    break

        for goal in brush2:
            goal = list(goal)
            for j in range(-3,4):
                if int(goal[1]/resolution - origin['y']/resolution) + j >= len(ogm[0]):
                    break
                if ogm[int(goal[0]/resolution - origin['x']/resolution)]\
                [int(goal[1]/resolution - origin['y']/resolution) + j] > 49: #\
                #or ogm[int(goal[0]/resolution - origin['x']/resolution) + i]\
                #[int(goal[1]/resolution - origin['y']/resolution) ] == -1:
                    goal = tuple(goal)
                    throw.add(goal)
                    break

        print 'size of throw:'
        print len(throw)

        brush2.difference_update(throw)

        print 'size of brush2 after update:'
        print len(brush2)

#        marker_pub = rospy.Publisher('Marker_pub_topic',Marker,queue_size = 10)
#        for i in brush2:
#            self.publish_markers(marker_pub,i[0],i[1])
#
        #print brush2
        distance_map = dict()
        for goal in brush2:
            dist = math.hypot(goal[0] - robotPose['x'],goal[1] - robotPose['y'])
            distance_map[goal] = dist


        sorted_dist_map = sorted(distance_map.items(), key=operator.itemgetter(1))

        sorted_goal_list = list()
        for key, value in sorted(distance_map.iteritems(), key=lambda (k,v): (v,k)):
            sorted_goal_list.append(key)
            pass
            #print "%s: %s" % (key, value)


        #rand_target = random.choice(distance_map.keys())
        #goal = rand_target
        ind = random.randrange(0,min(8,len(sorted_goal_list)))
        print 'ind is'
        print ind
        goal = sorted_goal_list[ind]
        print 'the dist is'
        print distance_map[goal]
        #goal = key
#        for key in sorted_goal_list:
#            if distance_map[key] > random.randrange(1,5):
#                print 'the dist is:'
#                print distance_map[key]
#                print '!!! FOUND !!!!'
#                goal = key
#                break
#        for key in distance_map.keys():
#            print 'the value of ogm is:'
#            print ogm[int(key[0] - origin['x_px'])][int(key[1] - origin['y_px'])]

        #while ogm[int(goal[0])][int(goal[1])] != 0:
        #rand_target = random.choice(distance_map.keys())
        #goal = rand_target
        #    print ogm[int(goal[0])][int(goal[1])]

        goal = list(goal)

        print 'the ogm value is'
        print ogm[int(goal[0] - origin['x_px'])][int(goal[1] - origin['y_px'])]
        #goal[0] = goal[0] + random.uniform(-0.5,0.5)
        #goal[1] = goal[1] + random.uniform(-0.5,0.5)
        print goal
        self.target = goal
        #for goal in brush2:
        #    print sorted_distance_map[goal]


        return self.target


        rospy.loginfo("goal AFTER unifrom is: goal = [%f,%f]" , store_goal[0],store_goal[1])

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
