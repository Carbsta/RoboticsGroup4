#!/usr/bin/env python
import rospy
import random 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
from math import pi, sqrt

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
        self.rate = rospy.Rate(10)
        self.pose = Pose(0,0,0)
        self.distance = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('odom',Odometry,self.odom_callback)

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

    def robot_movement(self):
        while not rospy.is_shutdown:

            random_walk()

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
            # rotate by angle alpha
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = pi/32
            i = pi*random.random()
            while(self.pose.theta < i):
                self.pub.publish(move_cmd)
                self.rate.sleep()
            print 'My orientation is {}'.format(self.pose.theta)
            
            


    def obstacle_avoidance(self):
        return

    def wall_following(self):
        return

    

if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.random_walk()
    except rospy.ROSInterruptException:
        pass
