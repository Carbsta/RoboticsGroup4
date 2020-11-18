#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, OccupancyGrid
from actionlib_msgs.msg import *
from math import pi, sqrt
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

""" class Waypoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y """

class Map:
    def __init__(self, origin, size, resolution):
        self.origin = origin
        self.size = size
        self.resolution = resolution
        self.waypoints = []
        self.grid = OccupancyGrid()

        self.initGrid()

    def initGrid():
        self.grid.header.seq = 1
        self.grid.header.frame_id = "/map"
        self.grid.info.origin.position.z = 0
        self.grid.info.origin.orientation.x = 0
        self.grid.info.origin.orientation.y = 0
        self.grid.info.origin.orientation.z = 0
        self.grid.info.origin.orientation.w = 1
        self.grid.data = [-1] * (self.size[0] * self.size[1])

    def updateOcGrid(self, world_point):
        gp = self.to_grid(world_point)
        grid_index = self.to_index(gp[0], gp[1], self.size[0])
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

    def to_index(gx, gy, size_x):
        return gy * size_x + gx

class TurtlebotDriving:
    """Robot Class
    
    Contains methods for both RANDOM WALK and OBSTACLE AVOIDANCE.
    Publishes to /cmd_vel
    Subscribes to /odom with the method odom_callback
    Subscribes to /scan with the method scan_callback
    Subscribes to /camera/rgb/image_raw with the method image_callback
    
    member variables:
        rate: refresh rate in Hz of actions, used to set intervals of sleep
        pose: robot pose object"""
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(10)
        self.map = Map((20,20), 1)
        self.initialise_waypoints()
        self.sub = rospy.Subscriber('odom',Odometry,self.odom_callback)

    def odom_callback(self, msg):
        self.updateOcGrid([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def initialise_waypoints(self):
        self.map.add_waypoint(Point(-3, 1, 0))
        self.map.add_waypoint(Point(-6, 3, 0))
        self.map.add_waypoint(Point(-6, -3, 0))
        self.map.add_waypoint(Point(1, 2, 0))
        self.map.add_waypoint(Point(5, 3, 0))
        self.map.add_waypoint(Point(6, -4, 0))

    def robot_movement(self):
        """Robot Movement Control
        
        Entry function from main, calls random_walk() continuously
        Can be extended to run wall following behaviour.
        Currently just starts random walk method which is the current top level routine.
        
        returns: 
            nothing
        arguments: 
            none"""
        while not rospy.is_shutdown():
            for wp in self.map.waypoints:
                self.move_to_waypoint(wp)
    
    def move_to_waypoint(self, wp):
        ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)

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
