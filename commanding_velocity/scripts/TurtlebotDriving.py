#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from math import pi
import matplotlib.pyplot as plt

class Pose:
    """Simple Pose Object

    Contains the values set by odom_callback"""
    def __init__(self,theta,x,y):
        self.theta = theta
        self.x = x
        self.y = y

class Visualisation:
    """Visualisation Class"""

    def __init__(self, x, y):
        self.robotPositionX = x
        self.robotPositionY = y

    def visualise(self):
        plt.plot(self.robotPositionX, self.robotPositionY)
        plt.show()


class TurtlebotDriving:
    """Robot Class
    
    Contains methods for both open loop and closed loop control.
    Publishes to /cmd_vel
    Subscribes to /odom with the method odom_callback"""
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(30)
        self.pose = Pose(0,0,0)
        self.distance = 0
        self.rotation = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('odom',Odometry,self.odom_callback)
        self.tolerance = -0.01 # tolerated error when turning to avoid missing turns
        self.turnThetas = [(pi/2)+self.tolerance, pi+self.tolerance,
                         (pi+(pi/2))+self.tolerance, (2*pi)+self.tolerance]
        self.robotPositionX = []
        self.robotPositionY = []

    def open_loop(self):
        while not rospy.is_shutdown():

            self.rate.sleep()

            for _ in range(0,4):
                # drive move a distance d
                move_cmd = Twist()
                move_cmd.linear.x = 0.1
                self.distance = 0

                time = rospy.Time.now().to_sec()
                oldpose_x = self.pose.x
                oldpose_y = self.pose.y
                while rospy.Time.now().to_sec() - time < rospy.Duration(10).to_sec():
                    self.pub.publish( move_cmd )
                    self.rate.sleep()
                    x = abs(self.pose.x - oldpose_x)
                    y = abs(self.pose.y - oldpose_y)
                    oldpose_x = self.pose.x
                    oldpose_y = self.pose.y
                    self.distance += x+y
                    self.robotPositionX.append(self.pose.x)
                    self.robotPositionY.append(self.pose.y)
                print 'I have moved {} m'.format(self.distance)
                print 'I am at {} {}'.format(self.pose.x, self.pose.y)

                # rotate by angle alpha
                move_cmd.linear.x = 0.0
                # rotate 90 degrees over two seconds
                move_cmd.angular.z = (pi/2)/2
                time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - time < rospy.Duration(2).to_sec():
                    self.pub.publish(move_cmd)
                    self.rate.sleep()
                print 'My orientation is {}'.format(self.pose.theta)

            # stop
            self.pub.publish(Twist())
            self.plot_trajectory()
            break

    def closed_loop(self):
        while not rospy.is_shutdown():

            for i in range(0,4):
                # drive move a distance d
                move_cmd = Twist()
                move_cmd.linear.x = 0.15

                self.distance = 0

                oldpose_x = self.pose.x
                oldpose_y = self.pose.y
                while self.distance < 1:
                    self.pub.publish( move_cmd )
                    self.rate.sleep()
                    x = abs(self.pose.x - oldpose_x)
                    y = abs(self.pose.y - oldpose_y)
                    oldpose_x = self.pose.x
                    oldpose_y = self.pose.y
                    self.distance += x+y
                    self.robotPositionX.append(self.pose.x)
                    self.robotPositionY.append(self.pose.y)
                print 'I have moved {} m'.format(self.distance)
                print 'I am at {} {}'.format(self.pose.x, self.pose.y)

                # rotate by angle alpha
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = pi/32
                while self.pose.theta < self.turnThetas[i]:
                    self.pub.publish(move_cmd)
                    self.rate.sleep()
                print 'My orientation is {}'.format(self.pose.theta)
            
            #stop
            self.pub.publish(Twist())
            # visualise
            self.plot_trajectory()
            break

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

    def plot_trajectory(self):
        visualisation = Visualisation(self.robotPositionX, self.robotPositionY)
        visualisation.visualise()

if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        #robot.open_loop()
        robot.closed_loop()
    except rospy.ROSInterruptException:
        pass