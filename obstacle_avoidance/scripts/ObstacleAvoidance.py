#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
from math import pi
import random

class Pose:
    """Simple Pose Object

    Contains the values set by odom_callback"""
    def __init__(self,theta,x,y):
        self.theta = theta
        self.x = x
        self.y = y

class TurtlebotDriving:
    """Robot Class
    
    Contains methods for both open loop and closed loop control.
    Publishes to /cmd_vel
    Subscribes to /odom with the method odom_callback"""
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(30)
        self.pose = Pose(0,0,0)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('odom',Odometry,self.odom_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.random_walking = True
        self.rotate_direction = 0
        self.obstacle_interval = 0.5

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
        front_list = msg.ranges[0:14] + msg.ranges[345:359]
        front_average = sum(front_list) / len(front_list)
        left_list = msg.ranges[15:44]
        left_average = sum(left_list) / len(left_list)
        right_list = msg.ranges[315:344]
        right_average = sum(right_list) / len(right_list)
        print('front {}, right {}, left {}'.format(front_average, right_average, left_average))
        if front_average < self.obstacle_interval or left_average < self.obstacle_interval or right_average < self.obstacle_interval:
            if self.random_walking:
                self.random_walking = False
                if left_average < self.obstacle_interval:
                    self.rotate_direction = 1
                elif right_average < self.obstacle_interval:
                    self.rotate_direction = 0
                else:
                    self.rotate_direction = random.uniform(0, 1)
                print('stop here')
        else:
            self.random_walking = True

    def robot_movement(self):
        while not rospy.is_shutdown():
            if self.random_walking:
                self.random_walk()
            else:
                self.obstacle_avoidance()

    def random_walk(self):
        print('random walking')
        move_cmd = Twist()
        speed = 0.15
        move_cmd.linear.x = speed
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t < rospy.Duration(1).to_sec():
            self.pub.publish(move_cmd)
            self.rate.sleep()

    def obstacle_avoidance(self):
        print('avoiding')
        self.pub.publish(Twist())
        rotate_cmd = Twist()
        speed = pi/4 if self.rotate_direction == 0 else -pi/4
        print('rotating to {} with speed {}'.format(self.rotate_direction, speed))
        rotate_cmd.angular.z = speed
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t < rospy.Duration(1).to_sec():
            self.pub.publish(rotate_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.robot_movement()
    except rospy.ROSInterruptException:
        pass