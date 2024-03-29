#!/usr/bin/env python
import rospy
import random 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import tf
from math import pi, sqrt
from random import uniform
import cv_bridge, cv2
import numpy as np

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
    Subscribes to /camera/rgb/image_raw with the method image_callback
    
    member variables:
        rate: refresh rate in Hz of actions, used to set intervals of sleep
        pose: robot pose object
        ranges: list of 360 degrees of range lasers
        range_cones: ranges broken down into average values across a small number of cones
        range_flags: boolean values that are true if the value of a cone is less than obstacle_range
        distance: distance travelled by the robot in a straight line
        obstacle_range: parameter to control how close the robot can get to obstacles
        err: Error from beacon to calculate heading via proportional control
        proportional_z: the desired angular velocity of the robotic to adjust its heading to face a beacon target
        pillar_found: flag used to override random walk behaviour while beaconing
        beaconing_disabled: flag used to override beaconing while avoding obstacles
        beacon_cooldown: time between the end of the obstacle avoidance routine and re-enabling of beaconing
        beacon_cooldown_start: used to track the time at the start of the beaconing cooldown
        bridge: openCV bridge
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
        self.obstacle_range = 0.35
        self.err = 0
        self.proportional_z = 0
        self.pillar_found = False
        self.beaconing_disabled = False
        self.beacon_cooldown = 2
        self.beacon_cooldown_start = 0
        self.bridge = cv_bridge.CvBridge()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry,self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

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

    def image_callback(self, msg):
        """ Callback method for the image recognition
        
        returns: 
            nothing
        arguments:
            msg: scan data used to populate list of ranges from /camera/rgb/image_raw topic"""
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        image_resized = cv2.resize(image, (w/4,h/4))
        hsv_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

        lower_hsv_green = np.array([40, 40, 40], dtype="uint8")
        upper_hsv_green = np.array([70, 255, 255], dtype="uint8")

        mask = cv2.inRange(hsv_image, lower_hsv_green, upper_hsv_green)

        # Contour detection starts from the bottom of the image, so we rotate the image 90 degrees counter clockwise
        # To ensure that the first beacon detected is always the left most beacon.
        rotated_mask = cv2.rotate(mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
        _, contours, hierarchy = cv2.findContours(rotated_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # this draws a new mask by drawing the first contour as a white fill on a black image, always drawing the leftmost pillar
        base = np.zeros( (w/4, h/4) )
        single_mask_rotated = cv2.drawContours(base, contours, 0, (255),-1)

        # We rotate this new mask back for proportional control
        single_mask = cv2.rotate(single_mask_rotated, cv2.ROTATE_90_CLOCKWISE)

        M = cv2.moments(single_mask)

        if M['m00'] > 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(image_resized,(cX,cY), 5, (0,0,255), -1)
            # Beaconing is disabled at the *end* of the obstacle avoidance routine to avoid triggering avoidance again immediately
            if self.beaconing_disabled:
                self.proportional_z = 0
                if rospy.Time.now().to_sec() - self.beacon_cooldown_start >= self.beacon_cooldown:
                    self.beaconing_disabled = False
                    print "BEACONING ENABLED\n"
            else:
                self.err = cX- w/8
                self.pillar_found = True
                self.proportional_z = -float(self.err) / 100
        else:
            self.pillar_found = False
            self.proportional_z = 0

        
        rotated_output = cv2.rotate(image_resized, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.drawContours(rotated_output, contours, -1, (0,255,0), 3)
        output = cv2.rotate(rotated_output, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow("single mask", single_mask)
        cv2.imshow("window", output)

        cv2.waitKey(3)
        
    def robot_movement(self):
        """Robot Movement Control
        
        Entry function from main, calls random_walk() continuously
        Can be extended to run wall following behaviour.
        Currently just starts random walk method which is the current top level routine.
        
        returns: 
            nothing
        arguments: 
            none"""
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

            self.beacon_cooldown_start = rospy.Time.now().to_sec()
            self.beaconing_disabled = True
            print "Avoiding Complete."
            print "BEACONING DISABLED\n"
        
        if self.range_flags[2]:
            print 'AVOIDING FRONT LEFT!'

            while self.range_flags[2]:
                target_theta = (self.pose.theta - pi/16)
                self.turn_to_theta(target_theta)

            self.beacon_cooldown_start = rospy.Time.now().to_sec()
            self.beaconing_disabled = True
            print "Avoiding Complete."
            print "BEACONING DISABLED\n"
        
        if self.range_flags[3]:
            print 'AVOIDING FRONT RIGHT!'

            while self.range_flags[3]:
                target_theta = (self.pose.theta + pi/16)
                self.turn_to_theta(target_theta)

            self.beacon_cooldown_start = rospy.Time.now().to_sec()   
            self.beaconing_disabled = True
            print "Avoiding Complete."
            print "BEACONING DISABLED\n"

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
            while not rospy.is_shutdown():
                self.obstacle_avoidance()
                move_cmd.angular.z = self.proportional_z
                self.pub.publish(move_cmd)
                self.rate.sleep()
                x2 = self.pose.x
                y2 = self.pose.y
                self.distance += abs(x2 - x1) + abs(y2 - y1)
                x1 = self.pose.x
                y1 = self.pose.y
                if self.distance >= 3 and not self.pillar_found: # keep going forward if beaconing
                    break

            print "RANDOM TURN!"
            target_theta = uniform(0, 2*pi)
            self.turn_to_theta(target_theta, True)

    def turn_to_theta(self, target_theta, breakout=False):
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

        #breakout and pillar_found used in conjunction to decide if a turn should be ended early by beaconing
        while total_turned < total_to_turn and not (breakout and self.pillar_found):
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