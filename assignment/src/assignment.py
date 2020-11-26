#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

import numpy as np
import tf
import cv_bridge, cv2
import actionlib

from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Map:
    def __init__(self, (x, y), (size_x, size_y), resolution):
        self.origin = (x, y)
        self.size = (size_x, size_y)
        self.resolution = resolution
        self.waypoints = []
        self.grid = OccupancyGrid()
        self.robot_pos = 0

        self.initGrid()

    def initGrid(self):
        self.grid.header.seq = 1
        self.grid.header.frame_id = "/map"
        self.grid.info.resolution = self.resolution
        self.grid.info.width = self.size[0]
        self.grid.info.height = self.size[1]
        self.grid.info.origin.position.x = self.origin[0]
        self.grid.info.origin.position.y = self.origin[1]
        self.grid.info.origin.position.z = 0
        self.grid.info.origin.orientation.x = 0
        self.grid.info.origin.orientation.y = 0
        self.grid.info.origin.orientation.z = 0
        self.grid.info.origin.orientation.w = 1
        self.grid.data = [-1] * (self.size[0] * self.size[1])

    def updateOcGrid(self, world_point):
        gp = self.to_grid(world_point)
        if gp == None:
            print "Out of Bounds"
        else:
            grid_index = self.to_index(gp[0], gp[1], self.size[0])
            self.robot_pos = grid_index
            self.grid.data[grid_index] = 1
    
    def add_waypoint(self, point):
        self.waypoints.append(point)
    
    def to_grid(self, point):
        gp = ( 
                int( (point[0] - self.origin[0]) / self.resolution),
                int( ( point[1] - self.origin[1] ) / self.resolution)
            )
        if gp[0] >= self.size[0] or gp[1] >= self.size[1] or gp[0] < 0 or gp[1] < 0:
            return None
        else:
            return gp

    def to_world(self, point):
        wp = (
                ( ( self.origin[0] + (point[0] * self.resolution)) + self.resolution / 2.0),
                ( ( self.origin[1] + (point[1] * self.resolution)) + self.resolution / 2.0)
            )

        if point[0] >= self.size[0] or point[0] < 0 or point[1] >= self.size[1] or point[1] < 0:
            return None
        else:
            return wp

    def to_index(self, gx, gy, size_x):
        return gy * size_x + gx

    def display(self):
        row = []
        for x in range(self.size[0] * self.size[1], 0, -1):
            if x % self.size[0] == 0:
                print("".join(row))
                row = []
            item = self.grid.data[x-1]
            if x-1 == self.robot_pos:
                row.insert(0,"@@")
            else:
                if item == -1:
                    row.insert(0,"▓▓")
                else:
                    if item <= 90:
                        row.insert(0,"░░")
                    else:
                        row.insert(0,"██")

class TurtleBot():
    def __init__(self):
        rospy.init_node('turtlebot', anonymous=True)
        self.rate = rospy.Rate(10)
        self.map = Map( (-10, -10 ), (20,20), 1)
        self.ranges = [0] * 360
        self.found = {"fire_hydrant":False, "green_box":False, "mail_box":False, "number_5":False}
        self.odom_sub = rospy.Subscriber('odom',Odometry,self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
    def initialisePose(self):
        posepub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped, queue_size=1)
        initialpose = PoseWithCovarianceStamped()
        initialpose.header.frame_id = "map"
        initialpose.header.stamp = rospy.Time.now()
        initialpose.pose.pose.position.x = -1
        initialpose.pose.pose.position.y = 4
        initialpose.pose.pose.position.z = 0
        initialpose.pose.pose.orientation.x = 0
        initialpose.pose.pose.orientation.y = 0
        initialpose.pose.pose.orientation.z = 0
        initialpose.pose.pose.orientation.w = 1
        initialpose.pose.covariance = [0] * 36
        posepub.publish(initialpose)

    def initialise_waypoints(self):
        """creates waypoints for each room"""
        self.map.add_waypoint(Point(-1, 3, 0)) # start corridor A
        self.map.add_waypoint(Point(-1, -2, 0)) # start corridor B obstacle
        self.map.add_waypoint(Point(2, -3, 0)) # open room A
        self.map.add_waypoint(Point(2, 0, 0)) # open room B
        self.map.add_waypoint(Point(2, 2, 0)) # closed room A
        self.map.add_waypoint(Point(4, 4, 0)) # closed room B
        self.map.add_waypoint(Point(6, 3, 0)) # far corridor A
        self.map.add_waypoint(Point(5,-3,0)) # far corridor B

    def entry_function(self):
        self.initialisePose()
        self.initialise_waypoints()
        self.robot_movement()

    def robot_movement(self):
        while not rospy.is_shutdown() and not all(x==True for x in self.found.values()):
            for wp in self.map.waypoints:
                self.move_to_waypoint(wp)

    def odom_callback(self, msg):
        pass

    def image_callback(self, msg):
        pass

    def move_to_waypoint(self, wp):
        """Sends commands to move the robot to a desired goal location.
        
        arguments:
            wp: Point object that represents the goal location.
        returns:
            Boolean: True if destination is reached, False if 120 seconds pass without reaching the destination."""

        ac = actionlib.SimpleActionClient("move_base",MoveBaseAction) # action client communicates with the move_base server

        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = wp
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(120))

        if(ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def scan_callback(self, msg):
        self.ranges = msg.ranges


    def frontier_exploration(self):
        pass

    def sense(self):
        pass


if __name__ == '__main__':
    try:
        robot = TurtleBot()
        robot.entry_function()
    except rospy.ROSInterruptException:
        pass