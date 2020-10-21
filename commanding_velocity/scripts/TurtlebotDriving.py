#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from math import pi
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class Pose:
    """Simple Pose Object
    
    Contains the values set by odom_callback"""
    def __init__(self,theta,x,y):
        self.theta = theta
        self.x = x
        self.y = y

class Visualisation:
    """Placeholder Visualisation Class"""
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.xdata, self.ydata = [], []
        self.ln, = plt.plot([],[], 'ro')
        self.ani = FuncAnimation(self.fig, self.update, frames=np.linspace(0, 2*np.pi, 128),
                    init_func=self.init, blit=True)
        plt.show()

    def init(self):
        self.ax.set_xlim(0, 2*np.pi)
        self.ax.set_ylim(-1, 1)
        return self.ln,

    def update(self,frame):
        self.xdata.append(frame)
        self.ydata.append(np.sin(frame))
        self.ln.set_data(self.xdata, self.ydata)
        return self.ln,


class TurtlebotDriving:
    """Robot Class
    
    Contains methods for both open loop and closed loop control.
    Publishes to /cmd_vel
    Subscribes to /odom with the method odom_callback"""
    def __init__(self):
        rospy.init_node('turtlebot_driving', anonymous=True)
        self.rate = rospy.Rate(20)
        self.pose = Pose(0,0,0)
        self.distance = 0
        self.rotation = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('odom',Odometry,self.odom_callback)
        self.tolerance = -0.05 # tolerated error when turning
        self.turnThetas = [(pi/2)+self.tolerance, pi+self.tolerance,
                         (pi+(pi/2))+self.tolerance, (2*pi)+self.tolerance]


    def open_loop(self):
        while not rospy.is_shutdown():

            for _ in range(0,4):
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
                #rotate 90 degrees over two seconds
                move_cmd.angular.z = (pi/2)/2
                time = rospy.Time.now()
                while rospy.Time.now() - time < rospy.Duration(2):
                    self.pub.publish( move_cmd )
                    self.rate.sleep()
                    print 'My orientation is {}'.format(self.pose.theta)

            
            #stop
            self.pub.publish(Twist())
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
                    print 'I have moved {} m'.format(self.distance)
                    print 'I am at {} {}'.format(self.pose.x, self.pose.y)


                # rotate by angle alpha
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = pi/8
                while self.pose.theta < self.turnThetas[i]:
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

        if yaw < 0:
            yaw += 2*pi
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y


if __name__ == '__main__':
    try:
        robot = TurtlebotDriving()
        robot.closed_loop()
        plot = Visualisation()
    except rospy.ROSInterruptException:
        pass
        