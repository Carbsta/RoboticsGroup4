#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
from math import pi

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

    def random_walk(self):
        # Drive move a distance d
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        self.distance = 0

        time = rospy.Time.now().to_sec()
        oldpose_x = self.pose.x
        oldpose_y = self.pose.y
        while rospy.Time.now().to_sec() - time < rospy.Duration(2).to_sec():
            self.pub.publish(move_cmd)
            self.rate.sleep()
            x = abs(self.pose.x - oldpose_x)
            y = abs(self.pose.y - oldpose_y)
            oldpose_x = self.pose.x
            oldpose_y = self.pose.y
            self.distance += x+y
            self.obstacle_avoidance()
        print 'I have moved {} m'.format(self.distance)
        print 'I am at {} {}'.format(self.pose.x, self.pose.y)

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

    
if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.robot_movement()
    except rospy.ROSInterruptException:
        pass