#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy

import numpy as np
import tf
import cv_bridge, cv2
import actionlib
import argparse

from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from find_object_2d.msg import ObjectsStamped
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi, isnan, sqrt
from struct import unpack

"""
TODO:
done!

future:
rescaling slam map for faster frontier exploration
"""

class Map:
    """Map class containing OccupancyGrid for frontier exploration and waypoints for waypoint navigation"""
    def __init__(self):
        self.waypoints = []
        self.grid = OccupancyGrid()
        self.attempted_points = []
        # occupancy grid is updated by subscriming to the renamed /slam_map topic published by gmapping
        self.map_sub = rospy.Subscriber('/slam_map', OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        self.grid = msg

    def euclidean_distance(self, x, y):
        """finds the euclidean distance between any two points in n dimensions"""
        return sqrt(sum([(a - b) ** 2 for a, b in zip(x, y)]))                    
    
    def add_waypoint(self, point):
        """adds a waypoint to the end of the waypoint queue"""
        self.waypoints.append(point)
    
    def to_grid(self, point):
        """converts from world space to grid coordinates"""
        gp = ( 
                int( (point[0] - self.grid.info.origin.position.x) / self.grid.info.resolution),
                int( ( point[1] - self.grid.info.origin.position.y ) / self.grid.info.resolution)
            )
        if gp[0] >= self.grid.info.width or gp[1] >= self.grid.info.height or gp[0] < 0 or gp[1] < 0:
            return None
        else:
            return gp

    def to_world(self, point):
        """converts from grid coordinates to world space"""
        wp = (
                ( ( self.grid.info.origin.position.x + (point[0] * self.grid.info.resolution)) + self.grid.info.resolution / 2.0),
                ( ( self.grid.info.origin.position.y + (point[1] * self.grid.info.resolution)) + self.grid.info.resolution / 2.0)
            )

        if point[0] >= self.grid.info.width or point[0] < 0 or point[1] >= self.grid.info.height or point[1] < 0:
            return None
        else:
            return wp

    def to_index(self, gx, gy):
        """returns row major order index from grid indices"""
        return gy * self.grid.info.width + gx

    def from_index(self, index):
        """returns grid inidices from row major order index"""
        y = int(index / self.grid.info.width)
        x = index - (y * self.grid.info.width)
        return (x,y)

class TurtleBot():
    """Robot Class, contains methods to navigate the map, and locate objects"""
    def __init__(self):
        rospy.init_node('turtlebot', anonymous=True)
        self.rate = rospy.Rate(10)
        self.frontier_exp = False
        self.map = Map()
        self.initialise_waypoints()
        self.amcl_pose = PoseWithCovarianceStamped()
        self.stopping_distance = 0.2 # Stopping distance for manual waypoint cancelling
        self.current_goal = (0,0)
        self.points = PointCloud2()
        self.ranges = [0] * 360

        """Information about the objects, inlcuding whether they have been found yet, where
        the robot thinks a waypoint to move near them should be located, a list to convert from
        IDs from find_object_2d to the names used by the robot, and a dictionary to record the image
        coordinates of the objects found using find_object_2d"""
        self.found = {"fire_hydrant":False, "green_box":False, "mail_box":False, "number_5":False}
        self.goal_estimates = {"fire_hydrant":None, "green_box":None, "mail_box":None, "number_5":None}
        self.objectIds = [None,"number_5","mail_box","mail_box","mail_box"]
        self.object_uvs = {"number_5":None, "mail_box":None}
        self.object_seen = False # controls if the robot should be exploring or moving to an object

        self.ac = actionlib.SimpleActionClient("move_base",MoveBaseAction) # action client communicates with the move_base server
        
        """Publishers:"""
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher('/image_out', Image, queue_size=1)

        """Communication with TF, broadcasting object locations to TF enables them to be displayed
        in RVIZ"""
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener(cache_time=rospy.Duration(10.0))

        """Subscribers:"""
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.object_sub = rospy.Subscriber('/objectsStamped', ObjectsStamped, self.object_callback)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.point_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.points_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)

        self.bridge = cv_bridge.CvBridge() # conversion between ROS msgs and OpenCV images
    
    def amcl_callback(self, msg):
        """Updates the robot's pose and stops robot when in range of current goal"""
        self.amcl_pose = msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        distance = self.map.euclidean_distance((x, y),self.current_goal)

        if distance < self.stopping_distance:
            self.cancel_pub.publish(GoalID())

    def object_callback(self, msg):
        """Recieves data from find_object_2d and calculates position on camera and in world space"""
        self.object_uvs = {"number_5":None, "mail_box":None} # used by rgb_callback to draw onto image_out
        if msg.objects.data:
            for i in range(0, len(msg.objects.data),12): # object data grouped in 12s
                object_id = msg.objects.data[i]
                object_frame_id = "object_" + str(int(object_id))
                # offsets used to access specific attributes of the object
                object_width = msg.objects.data[i+1]
                object_height = msg.objects.data[i+2]
                object_dx = msg.objects.data[i+9]
                object_dy = msg.objects.data[i+10]
                key = self.objectIds[int(object_id)]
                self.object_uvs[key] = (object_dx+(object_width/2), object_dy+(object_height/2))
                if not self.found[key] and not self.object_seen:
                    self.range_object("/map", object_frame_id, msg.header.stamp, 0.75, key)
                    if self.object_seen:
                        self.cancel_pub.publish(GoalID())

    def rgb_callback(self, msg):
        """Process the rgb image from the robot camera to produce image masks for the
        green box and fire hydrant, as well as modify the image with markers for each
        object and publish to /image_out"""
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

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
        
        # drawing markers for mail box and number 5
        for key,value in self.object_uvs.iteritems():
            if self.object_uvs[key] != None:
                cv2.circle(self.image,(int(self.object_uvs[key][0]),int(self.object_uvs[key][1])), 20, (0,0,255), -1)
        
        image_out = self.bridge.cv2_to_imgmsg(self.image,encoding='bgr8')
        self.image_pub.publish(image_out)

    def scan_callback(self, msg):
        self.ranges = msg.ranges

    def points_callback(self, msg):
        self.points = msg

    def initialise_waypoints(self):
        """creates waypoints for each room to maximise area seen by robot's camera"""
        self.map.add_waypoint(Point(0, 0, 0)) # "centre"
        self.map.add_waypoint(Point(2, -3, 0)) # open room A
        self.map.add_waypoint(Point(3, 0, 0)) # open room B
        self.map.add_waypoint(Point(2, 2, 0)) # closed room A
        self.map.add_waypoint(Point(4, 4, 0)) # closed room B
        self.map.add_waypoint(Point(6, 3, 0)) # far corridor A
        self.map.add_waypoint(Point(5,-3,0)) # far corridor B
        self.map.add_waypoint(Point(-1, 2, 0)) # start corridor A
        self.map.add_waypoint(Point(-1, -2, 0)) # start corridor B

    def robot_movement(self):
        """If frontier exploration enabled, explore until frontier empty and beacon to objects
        when seen. Otherwise tour the map via predefined waypoints, beaconing to objects
        until all have been found"""
        num_points = len(self.map.waypoints)
        tours = 0
        while not rospy.is_shutdown():

            if self.frontier_exp:
                self.move_to_waypoint(Point(0,0,0))
            while self.frontier_exp and not self.frontier_exploration():
                if self.object_seen:
                    self.beacon_to_object()
                    self.move_to_waypoint(Point(0,0,0))
                if self.all_found():
                    print "done!"
                    return
                self.rate.sleep()
            if self.frontier_exp:
                print "frontier exploration complete, {} out of {} objects found, switching to roaming".format(self.num_found(), len(self.found))
                self.frontier_exp = False

            points_visited = 0
            while points_visited < num_points-1:
                for wp in self.map.waypoints:
                    self.move_to_waypoint(wp)

                    # spinning maximises area seen by the camera for object detection
                    if self.spin_safe() and not self.object_seen:
                        self.spin()

                    if self.object_seen:
                        self.beacon_to_object()

                    if self.all_found():
                        print "done!"
                        return

                    points_visited += 1
            tours += 1
            print "{} out of {} objects found in {} tour(s)".format(self.num_found(), len(self.found), tours)

    def beacon_to_object(self):
        """Move to a point near a seen object, announcing success if the robot stops within 1.5 meters
        of the object"""
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

    def move_to_waypoint(self, wp, frontier=False):
        """Sends commands to move the robot to a desired goal location."""

        self.current_goal = (wp.x, wp.y)

        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            continue
              #rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = wp
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        #rospy.loginfo("Sending goal location ...")
        self.ac.send_goal(goal)

        distance = self.map.euclidean_distance((self.amcl_pose.pose.pose.position.x, self.amcl_pose.pose.pose.position.y),(wp.x, wp.y))
        speed = 0.1

        timeout = (distance / speed) if frontier else 60
        self.ac.wait_for_result(rospy.Duration(timeout))

    def spin_safe(self):
        """Ensures the robot has a large enough area to rotate 360 degrees in,
        with some margin for error"""
        return not any(x <= 0.25 for x in self.ranges)

    def spin(self):
        """Rotate 360 degrees to maximise area scanned by camera, exit early if an
        object is seen."""
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
            # open loop as uses speed and time to calculate distance rotated.
            current_angle = angular_speed*(t1 - t0) 

        t.angular.z = 0
        self.twist_pub.publish(t)

    def num_found(self):
        """Returns the number of objects found so far"""
        return sum(x==True for x in self.found.values())

    def all_found(self):
        """Returns true if all objects are found"""
        return all(x==True for x in self.found.values())

    def find_object_by_mask(self, object_name, mask):
        """Finds the centre pixel of an image mask and uses point cloud data
        to find the coresponding point cloud coordinates for the object and broadcasts
        this coordinate data to TF"""
        if not self.found[object_name]:
            M = cv2.moments(mask)
            if M['m00'] > 0:
                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])
                cv2.circle(self.image,(u,v), 20, (0,0,255), -1) # marks the image with a circle

                """PointCloud2 data is arranged as binary data in 1D array, with information
                describing the length of rows and points within the array, as well as the offset
                between fields. We use this infromation to calcualte "pointers" to index this data
                for the X, Y, and Z coordinates of the Point given by the image coordinates u,v
                Additionally as this data is stored as binary data, we need to read 4 bytes at a time
                and use struct.unpack to read this data as a floating point number"""

                arrayPosition = v*self.points.row_step + u*self.points.point_step
                arrayPosX = arrayPosition + self.points.fields[0].offset
                arrayPosY = arrayPosition + self.points.fields[1].offset
                arrayPosZ = arrayPosition + self.points.fields[2].offset

                [x] = unpack('f',self.points.data[arrayPosX:arrayPosX+4])
                [y] = unpack('f',self.points.data[arrayPosY:arrayPosY+4])
                [z] = unpack('f',self.points.data[arrayPosZ:arrayPosZ+4])

                if not isnan(x): # Discarding points that are out of depth range, aka nan values

                    translation = (x, y, z)
                    rotation = (0.0, 0.0, 0.0, 1.0)
                    time = rospy.Time.now()
                    child = object_name
                    parent = self.points.header.frame_id

                    self.broadcaster.sendTransform(translation, rotation, time, child, parent)
                    
                    if not self.object_seen:
                        self.range_object("/map", child, time, 0.75, child)

                        if self.object_seen:
                            self.cancel_pub.publish(GoalID())

    def range_object(self, map_frame, object_frame, time, dist_from, object_key):
        """Find a point near the desired object, using TF to transform into /map space"""
        try:
            # wait for the transform to be published
            self.listener.waitForTransform("/map",object_frame, time, rospy.Duration(1))
            # transform to world space (e.g from camera space)
            (trans, rot) = self.listener.lookupTransform("/map", object_frame, time)

            robot_x = self.amcl_pose.pose.pose.position.x
            robot_y = self.amcl_pose.pose.pose.position.y
            dist_between = self.map.euclidean_distance((robot_x,robot_y),(trans[0],trans[1]))
            dist_ratio = (dist_between - dist_from) / dist_between
            x = robot_x + dist_ratio * (trans[0] - robot_x)
            y = robot_y + dist_ratio * (trans[1] - robot_y)
            self.goal_estimates[object_key] = Point(x,y,0)
            self.object_seen = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error: {}".format(e))

    def frontier_exploration(self):
        """Implementation of frontier algorithm, using the highest priority grid space."""
        frontier = self.calculate_frontier()
        if not frontier:
            return True
        prio_cell = self.highest_priority(frontier)
        world_point = self.map.to_world(self.map.from_index(prio_cell))
        if world_point == None:
            print "World point out of map"
            raise IndexError
        # print "frontier goal {} {}".format(world_point[0], world_point[1])
        goal = Point(world_point[0], world_point[1], 0)
        self.move_to_waypoint(goal, True)
        """laser sensors may fail to update grid cells for all visited spaces for a variety of
        reasons, so we also keep track of any visited cells to exclude them from the frontier"""
        self.map.attempted_points.append(prio_cell)
        return False

    def calculate_frontier(self):
        """caluclates the frontier of the grid by checking if empty cells have unknown neighbours"""
        frontier = []
        for x in range(self.map.grid.info.width * self.map.grid.info.height):
            # cells the robot has visited but failed to be updated for whatever reason are excluded
            if not x in self.map.attempted_points:
                if self.map.grid.data[x] != -1 and self.map.grid.data[x] < 65:
                    borders = self.get_borders(x)
                    if -1 in borders:
                        frontier.append(x)
        return frontier

    def get_borders(self, x):
        """returns a list of the non diagonal neighbours of a cell"""
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
        """Returns the cell of the highest priority, as definied by the cell with the highest
        number of unknown non diagonal neigbours over the distance between the robot and the cell"""
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
        robot = TurtleBot()
        robot.frontier_exp = False # disabled by default for speed
        robot.robot_movement()
    except rospy.ROSInterruptException:
        pass