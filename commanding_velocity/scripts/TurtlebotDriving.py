#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from math import pi

class Pose:
    def __init__(self,theta,x,y):
        self.theta = theta
        self.x = x
        self.y = y

class TurtlebotDriving:
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pose = Pose(0,0,0)
        self.distance = 0
        self.rotation = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('odom',Odometry,self.odom_callback)

    def atTargetTheta(self, theta, targetTheta):
        return round(theta,2) == round(targetTheta,2)


    def open_loop(self):
        while not rospy.is_shutdown():

            for x in range(0,4):
                # drive move a distance d
                move_cmd = Twist()
                move_cmd.linear.x = 0.15
                self.distance = 0

                time = rospy.Time.now()
                oldpose_x = self.pose.x
                oldpose_y = self.pose.y
                while rospy.Time.now() - time < rospy.Duration(10):
                    self.pub.publish( move_cmd )
                    self.rate.sleep()
                    x = abs(self.pose.x - oldpose_x)
                    y = abs(self.pose.y - oldpose_y)
                    oldpose_x = self.pose.x
                    oldpose_y = self.pose.y
                    self.distance += x+y
                    print 'I have moved {} m'.format(self.distance)
                    print 'I am at {} {}'.format(self.pose.x, self.pose.y)


                # rotate by angle alpha
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 1.57/2
                time = rospy.Time.now()
                while rospy.Time.now() - time < rospy.Duration(2):
                    self.pub.publish( move_cmd )
                    self.rate.sleep()

            
            #stop
            self.pub.publish(Twist())
            break

    def closed_loop(self):
        while not rospy.is_shutdown():

            for x in range(0,4):
                # drive move a distance d
                move_cmd = Twist()
                move_cmd.linear.x = 0.15

                self.distance = 0

                time = rospy.Time.now()

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
                    print 'I have moved {} m'.format(self.distance)
                    print 'I am at {} {}'.format(self.pose.x, self.pose.y)


                # rotate by angle alpha
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 1.57/2
                time = rospy.Time.now()
                i = pi/2
                if self.pose.theta + i > pi:
                    targetTheta = (self.pose.theta + i) - 2*pi
                else:
                    targetTheta = self.pose.theta + i
                while not self.atTargetTheta(self.pose.theta, targetTheta):
                    self.pub.publish( move_cmd )
                    self.rate.sleep()
                    print 'My orientation is {}'.format(self.pose.theta)
            
            #stop
            self.pub.publish(Twist())
            break

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

#def plot_trajectory():


if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.closed_loop()
    except rospy.ROSInterruptException:
        pass
        