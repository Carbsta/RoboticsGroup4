#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from math import pi, sqrt
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Pose:
    """Simple Pose Object

    Contains the values set by odom_callback"""
    def __init__(self,theta,x,y):
        self.theta = theta
        self.x = x
        self.y = y

class Waypoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Map:
    def __init__(self, size, resolution):
        self.size = size
        self.resolution = resolution
        self.grid = [-1] * (size[0] * size[1])
        self.waypoints = []
    
    def add_waypoint(point):
        self.waypoints.append(point)
    
    def to_grid(point, origin_, size, resolution):
        return Waypoint( (point.x - origin.x) / res, (point.y - origin.y ) / res )

    def to_world(point, origin, size, resolution):
        return Waypoint( ( ( (point.x + origin.x) * res) + res / 2 ), ( (point.y + origin.y) * res) + res / 2 )

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
        self.pose = Pose(0,0,0)
        self.map = Map((20,20), 1)
        self.map.add_waypoint(Waypoint(-3, 1))
        self.map.add_waypoint(Waypoint(-6, 3))
        self.map.add_waypoint(Waypoint(-6, -3))
        self.map.add_waypoint(Waypoint(1, 2))
        self.map.add_waypoint(Waypoint(5, 3))
        self.map.add_waypoint(Waypoint(6, -4))


    def robot_movement(self):
        """Robot Movement Control
        
        Entry function from main, calls random_walk() continuously
        Can be extended to run wall following behaviour.
        Currently just starts random walk method which is the current top level routine.
        
        returns: 
            nothing
        arguments: 
            none"""
        self.rate.sleep()
        self.random_walk()
    
    def move_to_waypoint():

    def choose_waypoint():

if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.robot_movement()
    except rospy.ROSInterruptException:
        pass
