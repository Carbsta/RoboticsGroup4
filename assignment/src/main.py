import rospy

import numpy as np
import tf
import cv_bridge, cv2
import actionlib

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Map():
    pass

class TurtleBot():
    def __init__(self):
        rospy.init_node('turtlebot', anonymous=True)
        self.rate = rospy.Rate(10)
        self.map = Map( (-10, -10 ), (20,20), 1)
        self.odom_sub = rospy.Subscriber('odom',Odometry,self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

    def odom_callback(self, msg):
        pass

    def image_callback(self, msg):
        pass

    def scan_callback(self, msg):
        pass

if __name__ == '__main__':
    try:
        robot = TurtleBot()
        robot.entry_function()
    except rospy.ROSInterruptException:
        pass