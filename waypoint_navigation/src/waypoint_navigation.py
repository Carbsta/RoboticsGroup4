#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import *
from math import pi, sqrt, floor
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

""" class Waypoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y """

class Map:
    def __init__(self, size, resolution):
        self.size = size
        self.resolution = resolution
        self.grid = [-1] * (size[0] * size[1])
        self.waypoints = []
    
    def add_waypoint(self, point):
        self.waypoints.append(point)
    
    def to_grid(point, origin, size, resolution):
        gp = ( 
                floor( point.x - origin.x) / res),
                floor( point.y - origin.y ) / res)
                )
        if gp[0] >= size[0] or gp[1] >= size[1] or gp[0] < 0 or gp[1] < 0:
            return None
        else:
            return gp

    def to_world(point, origin, size, resolution):
        wp = (
                ( ( (point.x + origin.x) * res) + res / 2 ),
                ( ( (point.y + origin.y) * res) + res / 2 )
            )

        lowerbound_x = origin[0] - ((size[0] * res) / 2)
        lowerbound_y = origin[1] - ((size[1] * res) / 2)
        upperbound_x = origin[0] + ((size[0] * res) / 2)
        upperbound_y = origin[1] + ((size[1] * res) / 2)

        if wp[0] > upperbound_x or wp[0] < lowerbound_x or wp[1] > upperbound_y or wp[1] < lowerbound_y:
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
