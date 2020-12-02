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
from math import cos, sin, pi, isnan

class Map:
    def __init__(self, (x, y), (size_x, size_y), resolution):
        self.origin = (x, y)
        self.size = (size_x, size_y)
        self.resolution = resolution
        self.waypoints = []
        self.grid = OccupancyGrid()
        self.robot_pos = 0
        self.occupied_thresh = 0.65
        self.free_thresh = 0.196

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
    
    def updateMap(self, angle_cones, max_range, robot_pose):
        # update robot position
        robot_grid_pose = self.to_grid((robot_pose.x, robot.pose.y))
        self.robot_pos = self.to_index(robot_grid_pose[0], robot_grid_pose[1], self.size[0])

        # fill occupancy grid
        for i in range(len(angle_cones)):
            if angle_cones[i] != float("inf"):
                px_world = angle_cones[i] * cos((i * 20 + 10) * pi / 180) + robot_pose.x
                py_world = angle_cones[i] * sin((i * 20 + 10) * pi / 180) + robot_pose.y

                gp = self.to_grid((px_world, py_world))
                print('robot_pose: {}, robot_grid: {}, pworld: {}, gp: {}, distance: {}'.format(
                    (robot_pose.x, robot_pose.y),
                    robot_grid_pose,
                    (px_world, py_world),
                    gp,
                    angle_cones[i]
                ))
                if gp != None:
                    grid_index = self.to_index(gp[0], gp[1], self.size[0])
                    if angle_cones[i] < max_range:
                        self.grid.data[grid_index] = 100 #occupied by anything
                    else:
                        self.grid.data[grid_index] = 10
                        

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

class Pose:
    """Simple Pose Object

    Contains the values set by odom_callback"""
    def __init__(self,theta,x,y):
        self.theta = theta
        self.x = x
        self.y = y

class TurtleBot():
    def __init__(self):
        rospy.init_node('turtlebot', anonymous=True)
        self.rate = rospy.Rate(10)
        self.map = Map( (-10, -10 ), (20,20), 1)
        self.ranges = [0] * 360
        self.pose = Pose(0,0,0)
        self.found = {"fire_hydrant":False, "green_box":False, "mail_box":False, "number_5":False}
        self.goal_estimates = {"fire_hydrant":None, "green_box":None, "mail_box":None, "number_5":None}
        self.object_seen = False
        self.exploring = True
        self.green_mask = None
        self.display_image = None
        # self.amcl_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.amcl_callback)
        self.amcl_sub = rospy.Subscriber('/odom',Odometry,self.amcl_callback)
        #self.laser_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        #self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        #self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.bridge = cv_bridge.CvBridge()
        self.ac = actionlib.SimpleActionClient("move_base",MoveBaseAction) # action client communicates with the move_base server
        
    def initialisePose(self):
        posepub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped, queue_size=1)
        initialpose = PoseWithCovarianceStamped()
        initialpose.header.frame_id = "map"
        initialpose.header.stamp = rospy.Time.now()
        initialpose.pose.pose.position.x = -1
        initialpose.pose.pose.position.y = 1
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
        # self.initialise_waypoints()
        self.robot_movement()
    
    def robot_movement(self):
        while not rospy.is_shutdown() and not all(x==True for x in self.found.values()):
            for wp in self.map.waypoints:

                if self.object_seen:
                    self.beacon_to_object()

                self.move_to_waypoint(wp)

    def beacon_to_object(self):
        for key,value in self.found.iteritems():
            if self.found[key] == False and self.goal_estimates[key] != None:
                self.move_to_waypoint(self.goal_estimates[key])
                if(self.ac.get_state() == GoalStatus.SUCCEEDED):
                    print "I have found {}".format(key)
                    self.found[key] = True
                else:
                    print "failed to reach {}".format(key)
        self.exploring = True

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #processing_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        
        

        (h, w) = depth_image.shape[:2]

        image_resized = cv2.resize(depth_image, (w/4, h/4))

        output = np.zeros( (h/4, w/4) )
        

        intermediate = self.bridge.cv2_to_imgmsg(self.green_mask, encoding="passthrough")
        depth_mask = self.bridge.imgmsg_to_cv2(intermediate, desired_encoding="32FC1")

        if self.object_seen:
            output = cv2.bitwise_and(image_resized, depth_mask)
        
        M = cv2.moments(output)

        if M['m00'] > 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(self.display_image,(cX,cY), 5, (0,0,255), -1)
            if self.object_seen and self.exploring and not self.found["green_box"]:
                
                err = cX- w/8
                proportional_z = -float(err) / 100
                depth = image_resized[cY][cX]
                print "x,y,theta: {},{},{}".format(self.pose.x,self.pose.y,self.pose.theta)
                print "proportional_z: {}".format(proportional_z)
                print "cX {} cY {}".format(cX,cY)
                print "depth {}".format(depth)
                
                if not isnan(depth):
                    
                    x = self.pose.x + (depth * sin(self.pose.theta + proportional_z))
                    y = self.pose.y + (depth * cos(self.pose.theta + proportional_z))
                    self.goal_estimates["green_box"] = Point(x, y, 0)
                    print "current location {}, {}, theta: {}".format(self.pose.x, self.pose.y, self.pose.theta)
                    print "estimated location {} , {}, heading: {}".format(x, y, self.pose.theta + proportional_z)
                    self.exploring = False
                    self.cancel_pub.publish(GoalID())


        cv2.imshow("window", self.display_image)
        cv2.imshow("depth", image_resized)
        cv2.imshow("depth masked", output)
        cv2.imshow("green mask", self.green_mask)
        cv2.waitKey(3)
        

    def rgb_callback(self, msg):
        """ Callback method for the image recognition
        
        returns: 
            nothing
        arguments:
            msg: scan data used to populate list of ranges from /camera/rgb/image_raw topic"""
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        self.display_image = cv2.resize(image, (w/4, h/4))
        hsv_image = cv2.cvtColor(self.display_image, cv2.COLOR_BGR2HSV)

        lower_hsv_green = np.array([40, 200, 20], dtype="uint8")
        upper_hsv_green = np.array([70, 255, 255], dtype="uint8")

        self.green_mask = cv2.inRange(hsv_image, lower_hsv_green, upper_hsv_green)

        
        # Contour detection starts from the bottom of the image, so we rotate the image 90 degrees counter clockwise
        # To ensure that the first beacon detected is always the left most beacon.
        rotated_mask = cv2.rotate(self.green_mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
        _, contours, hierarchy = cv2.findContours(rotated_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        if contours and not self.found["green_box"] and not self.object_seen:
            # self.cancel_pub.publish(GoalID())
            self.object_seen = True


    def move_to_waypoint(self, wp):
        """Sends commands to move the robot to a desired goal location.
        
        arguments:
            wp: Point object that represents the goal location.
        returns:
            Boolean: True if destination is reached, False if 120 seconds pass without reaching the destination."""

        

        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
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
        self.ac.send_goal(goal)

        self.ac.wait_for_result(rospy.Duration(60))

        if(self.ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def amcl_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        # Make the range from 0 to +2pi to avoid negative value issues
        if yaw < 0:
            yaw += 2*pi
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

    def map_callback(self, msg):
        print('=' * 100)
        print(msg.info)
        print(len(msg.data))
        print('=' * 100)
        return

    def frontier_exploration(self):
        
        pass

    def sense(self):
        pass


if __name__ == '__main__':
    try:
        print('testing')
        robot = TurtleBot()
        robot.entry_function()
    except rospy.ROSInterruptException:
        pass