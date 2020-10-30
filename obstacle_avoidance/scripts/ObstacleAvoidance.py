#!/usr/bin/env python
import rospy
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

            self.random_walk()

    def random_walk(self):
        while not rospy.is_shutdown():
        
            move_cmd = Twist()
            move_cmd.linear.x = 0.15

            self.distance = 0

            x1 = self.pose.x
            y1 = self.pose.y
            while self.distance < 1:
                self.pub.publish(move_cmd)
                self.rate.sleep()
                x2 = self.pose.x
                y2 = self.pose.y
                self.distance += abs(x2 - x1) + abs(y2 - y1)
                x1 = self.pose.x
                y1 = self.pose.y

            target_theta = uniform(0, 2*pi)
            self.turn_to_theta(target_theta)
            


    def obstacle_avoidance(self):
        return

    def wall_following(self):
        return

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
        robot.random_walk()
    except rospy.ROSInterruptException:
        pass