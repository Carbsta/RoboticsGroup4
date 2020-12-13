#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy

import numpy as np
import tf
import cv_bridge, cv2
import actionlib

from geometry_msgs.msg import Point, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from find_object_2d.msg import ObjectsStamped
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import cos, sin, pi, isnan, sqrt
from struct import unpack

"""
TODO:
Mess with planner settings
Check mailbox range finding.
Refactor and documentation - --write object transform function--
extra:
rescaling slam map for fast frontier exploration
"""

class Map:
    """Map class containing OccupancyGrid for frontier exploration and waypoints for waypoint navigation"""
    def __init__(self):
        self.waypoints = []
        self.grid = OccupancyGrid()
        self.occupied_thresh = 0.65
        self.free_thresh = 0.196
        self.bad_points = []
        self.map_sub = rospy.Subscriber('/slam_map', OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        self.grid = msg

    def euclidean_distance(self, x, y):
        """finds the euclidean distance between any two points in n dimensions"""
        return sqrt(sum([(a - b) ** 2 for a, b in zip(x, y)]))                    
    
    def add_waypoint(self, point):
        self.waypoints.append(point)
    
    def to_grid(self, point):
        gp = ( 
                int( (point[0] - self.grid.info.origin.position.x) / self.grid.info.resolution),
                int( ( point[1] - self.grid.info.origin.position.y ) / self.grid.info.resolution)
            )
        if gp[0] >= self.grid.info.width or gp[1] >= self.grid.info.height or gp[0] < 0 or gp[1] < 0:
            return None
        else:
            return gp

    def to_world(self, point):
        wp = (
                ( ( self.grid.info.origin.position.x + (point[0] * self.grid.info.resolution)) + self.grid.info.resolution / 2.0),
                ( ( self.grid.info.origin.position.y + (point[1] * self.grid.info.resolution)) + self.grid.info.resolution / 2.0)
            )

        if point[0] >= self.grid.info.width or point[0] < 0 or point[1] >= self.grid.info.height or point[1] < 0:
            return None
        else:
            return wp

    def to_index(self, gx, gy):
        return gy * self.grid.info.width + gx

    def from_index(self, index):
        y = int(index / self.grid.info.width)
        x = index - (y * self.grid.info.width)
        return (x,y)

class TurtleBot():
    def __init__(self):
        rospy.init_node('turtlebot', anonymous=True)
        self.rate = rospy.Rate(10)
        self.frontier_exp = False
        self.map = Map()
        self.initialise_waypoints()
        self.amcl_pose = PoseWithCovarianceStamped()
        self.stopping_distance = 0.2
        self.current_goal = (0,0)
        self.points = PointCloud2()
        self.ranges = [0] * 360

        self.found = {"fire_hydrant":False, "green_box":False, "mail_box":False, "number_5":False}
        self.goal_estimates = {"fire_hydrant":None, "green_box":None, "mail_box":None, "number_5":None}
        self.objectIds = [None,"number_5","mail_box","mail_box","mail_box"]
        self.object_seen = False

        self.ac = actionlib.SimpleActionClient("move_base",MoveBaseAction) # action client communicates with the move_base server
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.broadcaster = tf.TransformBroadcaster()
        self.listner = tf.TransformListener()

        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.object_sub = rospy.Subscriber('/objectsStamped', ObjectsStamped, self.object_callback)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.point_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, callback=self.points_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)

        self.bridge = cv_bridge.CvBridge()
    

    def initialise_waypoints(self):
        """creates waypoints for each room"""
        self.map.add_waypoint(Point(0, 0, 0)) # "centre"
        self.map.add_waypoint(Point(2, -3, 0)) # open room A
        self.map.add_waypoint(Point(3, 0, 0)) # open room B
        self.map.add_waypoint(Point(2, 2, 0)) # closed room A
        self.map.add_waypoint(Point(4, 4, 0)) # closed room B
        self.map.add_waypoint(Point(6, 3, 0)) # far corridor A
        self.map.add_waypoint(Point(5,-3,0)) # far corridor B
        self.map.add_waypoint(Point(-1, 2, 0)) # start corridor A
        self.map.add_waypoint(Point(-1, -2, 0)) # start corridor B

    def entry_function(self):
        self.robot_movement()
    
    def robot_movement(self):
        while not rospy.is_shutdown():
            num_points = len(self.map.waypoints)
            points_visited = 0
            while points_visited < (2*num_points)-1:
                for wp in self.map.waypoints:
                    self.move_to_waypoint(wp)

                    if self.spin_safe() and not self.object_seen:
                        self.spin()

                    if self.object_seen:
                        self.beacon_to_object()

                    if self.all_found():
                        print "done!"
                        return

                    points_visited += 1

            print "{} out of {} objects found in two tours, switching to frontier exploration".format(self.num_found(), len(self.found))
            
            while not self.frontier_exploration():
                if self.object_seen:
                    self.beacon_to_object()
                if self.all_found():
                    print "done!"
                    return
                self.rate.sleep()
                
            print "frontier exploration complete, {} out of {} objects found, switching to roaming".format(self.num_found(), len(self.found))

    def spin_safe(self):
        return not any(x <= 0.25 for x in self.ranges)

    def spin(self):
        print "spinning"
        current_angle = 0
        speed = 33 # degrees per second
        angular_speed = speed*2*pi/360
        angle = 360
        relative_angle = angle*2*pi/360

        t = Twist()
        t.angular.z = angular_speed

        t0 = rospy.Time.now().to_sec()
        while current_angle < relative_angle and not self.object_seen:
            self.twist_pub.publish(t)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1 - t0)

        t.angular.z = 0
        self.twist_pub.publish(t)

    def num_found(self):
        return sum(x==True for x in self.found.values())

    def all_found(self):
        return all(x==True for x in self.found.values())

    def beacon_to_object(self):
        for key,value in self.found.iteritems():
            if self.found[key] == False and self.goal_estimates[key] != None:
                print "moving to {} at {}".format(key, self.goal_estimates[key])
                self.move_to_waypoint(self.goal_estimates[key])
                distance = self.map.euclidean_distance((self.amcl_pose.pose.pose.position.x, self.amcl_pose.pose.pose.position.y), (self.goal_estimates[key].x, self.goal_estimates[key].y))
                if(distance <= 1.5):
                    print "I have found {}!".format(key)
                    self.found[key] = True
                else:
                    print "failed to reach {}".format(key)
        self.object_seen = False

    def object_callback(self, msg):
        if msg.objects.data and not self.object_seen:
            for i in range(0, len(msg.objects.data),12):
                object_id = msg.objects.data[i]
                object_frame_id = "object_" + str(int(object_id))
                key = self.objectIds[int(object_id)]
                if not self.found[key]:
                    self.range_object("/map", object_frame_id, msg.header.stamp, 0.75, key)
            if self.object_seen:
                self.cancel_pub.publish(GoalID())

    def range_object(self, map_frame, object_frame, time, dist_from, object_key):
        try:
            (trans, rot) = self.listner.lookupTransform("/map", object_frame, time)

            robot_x = self.amcl_pose.pose.pose.position.x
            robot_y = self.amcl_pose.pose.pose.position.y
            dist_between = self.map.euclidean_distance((robot_x,robot_y),(trans[0],trans[1]))
            dist_ratio = (dist_between - dist_from) / dist_between
            x = robot_x + dist_ratio * (trans[0] - robot_x)
            y = robot_y + dist_ratio * (trans[1] - robot_y)
            self.goal_estimates[object_key] = Point(x,y,0)
            self.object_seen = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def points_callback(self, msg):
        self.points = msg

    def rgb_callback(self, msg):
        if not self.object_seen:
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # colour slicing for the green box
            lower_hsv_green = np.array([40, 200, 20], dtype="uint8")
            upper_hsv_green = np.array([70, 255, 255], dtype="uint8")
            green_mask = cv2.inRange(hsv_image, lower_hsv_green, upper_hsv_green)

            # colour slicing for the fire hydrant
            lower_hsv_red = np.array([0,255, 35], dtype="uint8")
            upper_hsv_red = np.array([10,255,64], dtype="uint8")
            red_mask = cv2.inRange(hsv_image, lower_hsv_red, upper_hsv_red)

            if not self.found["green_box"]:
                self.find_object_by_mask("green_box", green_mask)
            if not self.found["fire_hydrant"]:
                self.find_object_by_mask("fire_hydrant", red_mask)
            
            if self.object_seen:
                self.cancel_pub.publish(GoalID())

    def find_object_by_mask(self, object_name, mask):

        if not self.found[object_name]:
            M = cv2.moments(mask)
            if M['m00'] > 0:
                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])

                arrayPosition = v*self.points.row_step + u*self.points.point_step
                arrayPosX = arrayPosition + self.points.fields[0].offset
                arrayPosY = arrayPosition + self.points.fields[1].offset
                arrayPosZ = arrayPosition + self.points.fields[2].offset

                [x] = unpack('f',self.points.data[arrayPosX:arrayPosX+4])
                [y] = unpack('f',self.points.data[arrayPosY:arrayPosY+4])
                [z] = unpack('f',self.points.data[arrayPosZ:arrayPosZ+4])

                if not isnan(x):

                    translation = (x, y, z)
                    rotation = (0.0, 0.0, 0.0, 1.0)
                    time = rospy.Time.now()
                    child = object_name
                    parent = self.points.header.frame_id

                    self.broadcaster.sendTransform(translation, rotation, time, child, parent)
                    
                    self.range_object("/map", child, time, 0.75, child)

    def move_to_waypoint(self, wp):
        """Sends commands to move the robot to a desired goal location.
        
        arguments:
            wp: Point object that represents the goal location.
        returns:
            Boolean: True if destination is reached, False if 120 seconds pass without reaching the destination."""

        self.current_goal = (wp.x, wp.y)

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

    def amcl_callback(self, msg):
        self.amcl_pose = msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        distance = self.map.euclidean_distance((x, y),self.current_goal)

        if distance < self.stopping_distance:
            self.cancel_pub.publish(GoalID())

    def scan_callback(self, msg):
        self.ranges = msg.ranges

    def frontier_exploration(self):
        frontier = self.calculate_frontier()
        if not frontier:
            return True
        prio_cell = self.highest_priority(frontier)
        world_point = self.map.to_world(self.map.from_index(prio_cell))
        if world_point == None:
            print "World point out of map"
            raise IndexError
        print "frontier goal {} {}".format(world_point[0], world_point[1])
        goal = Point(world_point[0], world_point[1], 0)
        self.move_to_waypoint(goal)
        if(self.ac.get_state() != GoalStatus.SUCCEEDED):
            self.map.bad_points.append(prio_cell)
        return False

    def calculate_frontier(self):
        frontier = []
        for x in range(self.map.grid.info.width * self.map.grid.info.height):
            if not x in self.map.bad_points:
                if self.map.grid.data[x] != -1 and self.map.grid.data[x] < 65:
                    borders = self.get_borders(x)
                    if -1 in borders:
                        frontier.append(x)
        return frontier

    def get_borders(self, x):
        borders = []
        if x:
            borders.append(self.map.grid.data[x-1])
        if x >= self.map.grid.info.width:
            borders.append(self.map.grid.data[x-self.map.grid.info.width])
        if x < (self.map.grid.info.width * self.map.grid.info.height) - self.map.grid.info.width:
            borders.append(self.map.grid.data[x+self.map.grid.info.width])
        if x != (self.map.grid.info.width * self.map.grid.info.height) - 1:
            borders.append(self.map.grid.data[x+1])

        return borders

    def highest_priority(self, frontier):
        prio_cell = frontier[0]
        max_prio = -1
        for cell in frontier:
            a_cell = self.get_borders(cell).count(-1)
            world_point = self.map.to_world(self.map.from_index(cell))
            if world_point == None:
                print "World point out of map"
                raise IndexError
            d_cell = self.map.euclidean_distance((self.amcl_pose.pose.pose.position.x, self.amcl_pose.pose.pose.position.y), world_point)
            p_cell = a_cell / d_cell
            if p_cell >= max_prio:
                prio_cell = cell
                max_prio = p_cell
        return prio_cell

if __name__ == '__main__':
    try:
        print('testing')
        robot = TurtleBot()
        robot.entry_function()
    except rospy.ROSInterruptException:
        pass