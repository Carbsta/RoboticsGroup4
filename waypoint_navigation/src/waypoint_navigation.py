#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, OccupancyGrid
from actionlib_msgs.msg import *
from math import pi, sqrt
import numpy as np
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

        if point[0] >= self.size[0] or point[0] < 0 or point[1] > self.size[1] or point[1] < 0:
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
                    row.insert(0,"░░")
                else:
                    row.insert(0,"▓▓")

class TurtlebotDriving:
    """Robot Class
    
    Contains methods for both RANDOM WALK and OBSTACLE AVOIDANCE.
    Subscribes to /odom with the method odom_callback
    
    member variables:
        rate: refresh rate in Hz of actions, used to set intervals of sleep
        pose: robot pose object
        map: Map object containing occupancy grid and waypoints"""
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(10)
        self.map = Map( (-10, -10 ), (20,20), 1)
        self.initialise_waypoints()
        self.sub = rospy.Subscriber('odom',Odometry,self.odom_callback)

    def odom_callback(self, msg):
        """Updates the occupancy grid, displays robot position and calls method to display the grid
        
        returns:
            nothing
        arguments:
            msg: Pose message"""
        self.map.updateOcGrid([msg.pose.pose.position.x, msg.pose.pose.position.y])
        print (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.map.display()

    def initialise_waypoints(self):
        """creates waypoints for each room"""
        self.map.add_waypoint(Point(-3, 1, 0))
        self.map.add_waypoint(Point(-6, 3, 0))
        self.map.add_waypoint(Point(-6, -3, 0))
        self.map.add_waypoint(Point(1, 2, 0))
        self.map.add_waypoint(Point(5, 3, 0))
        self.map.add_waypoint(Point(6, -4, 0))

    def robot_movement(self):
        """Robot Movement Control
        
        Entry function from main, iterates over the waypoints continuously to navigate between them.
        
        returns: 
            nothing
        arguments: 
            none"""
        while not rospy.is_shutdown():
            for wp in self.map.waypoints:
                self.move_to_waypoint(wp)
    
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

if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.robot_movement()
    except rospy.ROSInterruptException:
        pass
