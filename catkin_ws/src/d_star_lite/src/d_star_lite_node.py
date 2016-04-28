#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal

from __future__ import division
import math
import numpy as np
from d_star_lite.world import World
from d_star_lit.grid import *
from d_star_lite.queue import PriorityQueue
from d_star_lite.graph import get_intended_path

inf = float("inf")


class DStarLiteNode():
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters:
        self.fsm_mode = self.setupParameter("~grid_resolution",1.0)

        # Subscribers:
        self.sub_pose = rospy.Subscriber("~pose", PoseWithCovarianceStamped, self.updatePose, queue_size=1)
        self.sub_grid = rospy.Subscriber("~grid", OccupancyGrid, self.updateGraph, queue_size = 1)

        # Publishers:
        self.pub_goal = rospy.Publisher("~goal", MoveBaseActionGoal, queue_size=1, latch=True)

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updatePose(self, data):
        # callback for pose message

    def updateGraph(self, data):
        # callback for occupancy grid message

    def onShutdown(self):
        rospy.loginfo("[DStarLiteNode] Shutdown.")

if __name__ == "__main__":
    rospy.init_node('d_star_lite_node')
    d_star_lite_node = DStarLiteNode()
    rospy.on_shutdown(d_star_lite_node.onShutdown)
    rospy.spin()