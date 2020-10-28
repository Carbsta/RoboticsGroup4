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
    
    Contains methods for both open loop and closed loop control.
    Publishes to /cmd_vel
    Subscribes to /odom with the method odom_callback"""
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pose = Pose(0,0,0)
        self.ranges = [0] * 360
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
        print 'test'
        self.ranges = msg.ranges
        dis_left = self.ranges[90]

    def robot_movement(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.random_walk()

    def random_walk(self):
        # drive move a distance d
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
        print 'avoiding'
        if self.ranges[0] <= 0.25 or self.ranges[270]<= 0.25 or self.ranges[90]<= 0.25:
            # stop forward movement
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            self.pub.publish(move_cmd)
            while self.ranges[0] > 1.0:
                # while there is nothing in front 1 meter rotate
                move_cmd.angular.z = pi/8
                self.pub.publish(move_cmd)

        print 'Range {} '.format(self.ranges[0])

    def wall_following(self):
        print 'following'

    

if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.robot_movement()
    except rospy.ROSInterruptException:
        pass