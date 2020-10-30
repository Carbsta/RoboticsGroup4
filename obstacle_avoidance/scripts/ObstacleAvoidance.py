#!/usr/bin/env python
import rospy
import random 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
from math import pi, sqrt
from random import uniform

class Pose:
    """Simple Pose Object

    Contains the values set by odom_callback"""
    def __init__(self,theta,x,y):
        self.theta = theta
        self.x = x
        self.y = y

class TurtlebotDriving:
    """Robot Class
    
    Contains methods for both RANDOM WALK and OBSTACLE AVOIDANCE.
    Publishes to /cmd_vel
    Subscribes to /odom with the method odom_callback
    Subscribes to /scan with the method scan_callback"""
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pose = Pose(0,0,0)
        self.ranges = [0] * 360
        self.front_average = 1
        self.left_average = 1
        self.right_average = 1
        self.distance = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('odom', Odometry,self.odom_callback)
        self.sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)

    def odom_callback(self, msg):
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
    
    def scan_callback(self, msg):
        # Get the ranges from the scan msg
        self.ranges = msg.ranges
        # Get a 30 degree cone reading from front left and right
        self.front_list = self.ranges[0:15] + self.ranges[345:360]
        self.front_average = sum(self.front_list) / len(self.front_list)
        self.left_list = self.ranges[75:105]
        self.left_average = sum(self.left_list) / len(self.left_list)
        self.right_list = self.ranges[255:285]
        self.right_average = sum(self.right_list) / len(self.right_list)
        print 'fr {} left {} right {}'.format(self.front_average, self.left_average, self.right_average)

    def robot_movement(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.random_walk()

    def obstacle_avoidance(self):
        if self.front_average <= 0.25 or self.left_average <= 0.25 or self.right_average <= 0.25:
            print 'Avoiding'
            # Stop forward movement
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            self.pub.publish(move_cmd)
            # While there is nothing in front 1m rotate
            while self.front_average < 1.0:
                move_cmd.angular.z = -pi/8
                self.pub.publish(move_cmd)

    def wall_following(self):
        print 'following'
        while not rospy.is_shutdown:

            self.random_walk()

    def random_walk(self):
        while not rospy.is_shutdown():
        
            move_cmd = Twist()
            move_cmd.linear.x = 0.15

            self.distance = 0

            x1 = self.pose.x
            y1 = self.pose.y
            while self.distance < 3:
                self.pub.publish(move_cmd)
                self.rate.sleep()
                x2 = self.pose.x
                y2 = self.pose.y
                self.distance += abs(x2 - x1) + abs(y2 - y1)
                x1 = self.pose.x
                y1 = self.pose.y
                self.obstacle_avoidance()

            target_theta = uniform(0, 2*pi)
            self.turn_to_theta(target_theta)

    def turn_to_theta(self, target_theta):
        """Turns the robot to the desired orientation via the shortest arc.
        
        returns: nothing
        arguments:
            target_theta: the desired orientation of the robot in the world coordinates"""

        print "current theta {}".format(self.pose.theta)
        print "turning to {}".format(target_theta)

        # calculate the change in orientation for both clockwise and anti clockwise rotation
        if self.pose.theta < target_theta:
            print "current less than target"
            anticlockwise = target_theta - self.pose.theta
            clockwise = 2*pi - target_theta + self.pose.theta
        else:
            print "target less than current"
            anticlockwise = 2*pi - self.pose.theta + target_theta
            clockwise = self.pose.theta - target_theta

        print "anticlockwise {}".format(anticlockwise)
        print "clockwise {}".format(clockwise)

        #python lists can be indexed by boolean values to choose between rotating clockwise and anti clockwise
        turnvelocities = [-pi/16, pi/16]
        move_cmd = Twist()
        move_cmd.angular.z = turnvelocities[anticlockwise < clockwise]

        total_turned = 0
        targets = [clockwise, anticlockwise]
        total_to_turn = targets[anticlockwise < clockwise]

        print "total radians to turn {}".format(total_to_turn)
        t0 = self.pose.theta

        while total_turned < total_to_turn:
            self.pub.publish(move_cmd)
            self.rate.sleep()
            t1 = self.pose.theta
            # when changing from values either side of 0 / 2pi we need to subtract 2pi to get the relative difference
            if (t0 > pi+pi/2) and (t1 < pi/2):
                diff = abs(t0 - t1 - 2*pi)
            elif (t0 < pi/2) and (t1 > pi+pi/2):
                diff = abs(t1 - t0 - 2*pi)
            else:
                diff = abs(t1 - t0)

            total_turned += diff
            t0 = self.pose.theta

        print "current theta {}".format(self.pose.theta)
        print "total radians turned {}\n".format(total_turned)
    
if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.robot_movement()
    except rospy.ROSInterruptException:
        pass
