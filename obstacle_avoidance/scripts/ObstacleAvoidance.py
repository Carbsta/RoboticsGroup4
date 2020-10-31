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
    Subscribes to /scan with the method scan_callback
    
    member variables:
        rate: refresh rate in Hz of actions, used to set intervals of sleep
        pose: robot pose object
        ranges: list of 360 degrees of range lasers
        range_cones: ranges broken down into average values across a small number of cones
        range_flags: boolean values that are true if the value of a cone is less than obstacle_range
        distance: distance travelled by the robot in a straight line
        obstacle_range: parameter to control how close the robot can get to obstacles
        pub: publisher channel to post movement commands to
        odom_sub: subscriber channel to recieve odometry data
        laser_sub: subscriber channel to recieve scan data"""
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pose = Pose(0,0,0)
        self.ranges = [0] * 360
        self.range_cones = [0] * 6
        self.range_flags = [False] * 6
        self.distance = 0
        self.obstacle_range = 0.5
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry,self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)

    def odom_callback(self, msg):
        """Callback method to update internal pose of the robot.

        returns: 
            nothing
        arguments:
            msg: pose data supplied by the /odom topic
        """
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
        """Callback method to update internal list of range cones and flags

        returns: 
            nothing
        arguments:
            msg: scan data used to populate list of ranges from /scan topic"""
        self.ranges = msg.ranges

        front_centre_left = self.ranges[0:14] # 15 degree cone
        self.range_cones[0] = sum(front_centre_left) / len(front_centre_left)

        front_left = self.ranges[15:29]
        self.range_cones[1] = sum(front_left) / len(front_left)

        front_left_bumper = self.ranges[30:60] # 30 degree cone
        self.range_cones[2] = sum(front_left_bumper) / len(front_left_bumper)

        front_right_bumper = self.ranges[300:330]
        self.range_cones[3] = sum(front_right_bumper) / len(front_right_bumper)

        front_right = self.ranges[330:344]
        self.range_cones[4] = sum(front_right) / len(front_right)

        front_centre_right = self.ranges[345:359]
        self.range_cones[5] = sum(front_centre_right) / len(front_centre_right)

        self.range_flags = [x <= self.obstacle_range for x in self.range_cones]
        
    def robot_movement(self):
        """Robot Movement Control
        
        Entry function from main, calls random_walk() continuously
        Can be extended to run wall following behaviour
        
        returns: 
            nothing
        arguments: 
            none"""
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.random_walk()

    def obstacle_avoidance(self):
        """Obstacle Avoidance
        
        Checks if range flags have been set, and picks a direction for the robot to turn.
        Turns the robot until the relevant range flags are no longer set.
        
        returns: 
            nothing
        arguments: 
            none"""
        if self.range_flags[0] or self.range_flags[5] or self.range_flags[1] or self.range_flags[4]:
            print 'AVOIDING FRONT!'

            if self.range_cones[0] < self.range_cones[5]:
                print 'Turning Right'
                turnright = True
                
            else:
                print 'Turning Left'
                turnright = False

            while self.range_flags[0] or self.range_flags[5] or self.range_flags[1] or self.range_flags[4]:
                if turnright:
                    target_theta = (self.pose.theta - pi/16)
                    self.turn_to_theta(target_theta)
                else:
                    target_theta = (self.pose.theta + pi/16)
                    self.turn_to_theta(target_theta)

            print "Avoiding Complete."
        
        if self.range_flags[2]:
            print 'AVOIDING FRONT LEFT!'

            while self.range_flags[2]:
                target_theta = (self.pose.theta - pi/16)
                self.turn_to_theta(target_theta)

            print "Avoiding Complete."
        
        if self.range_flags[3]:
            print 'AVOIDING FRONT RIGHT!'

            while self.range_flags[3]:
                target_theta = (self.pose.theta + pi/16)
                self.turn_to_theta(target_theta)

            print "Avoiding Complete."

    def wall_following(self):
        """future wall following"""
        print 'following'

    def random_walk(self):
        """Moves robot forward for three meters before turning a random direction.
        
        returns:
            nothing
        arguments:
            none"""
        while not rospy.is_shutdown():
        
            move_cmd = Twist()
            move_cmd.linear.x = 0.15

            self.distance = 0

            x1 = self.pose.x
            y1 = self.pose.y
            while self.distance < 3:
                self.obstacle_avoidance()
                self.pub.publish(move_cmd)
                self.rate.sleep()
                x2 = self.pose.x
                y2 = self.pose.y
                self.distance += abs(x2 - x1) + abs(y2 - y1)
                x1 = self.pose.x
                y1 = self.pose.y

            print "RANDOM TURN!"
            target_theta = uniform(0, 2*pi)
            self.turn_to_theta(target_theta)

    def turn_to_theta(self, target_theta):
        """Turns the robot to the desired orientation via the shortest arc.
        
        returns: 
            nothing
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
