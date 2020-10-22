#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
import tf
import matplotlib.pyplot as plt

class Visualisation:
    def __init__(self, robot_path_x, robot_path_y):
        self.robot_path_x = robot_path_x
        self.robot_path_y = robot_path_y
    
    def show_map(self):
        plt.plot(self.robot_path_x, self.robot_path_y)
        plt.show()

class Pose:
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta

class TurtleBo:
    def __init__(self, initial_x, initial_y):
        rospy.init_node('robot_square_mover_closed', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pose = Pose(initial_x, initial_y, 0)
        self.distance = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.desired_pose = [Pose(1, 0, 0), Pose(1, 1, 0), Pose(0, 1, 0), Pose(0, 0, 0)]
        self.desired_theta = [pi/2, pi, -pi/2, 0]
        self.robot_path_x = []
        self.robot_path_y = []
    
    def close_loop_move(self):
        while not rospy.is_shutdown():
            for i in range(0, 4):
                move_cmd = Twist()
                speed = 0.15

                print('step: {}'.format(i))
                
                move_cmd.linear.x = speed
                distance_to_travel = self.count_distance_to_travel(i)
                while distance_to_travel > 0.02:
                    self.pub.publish(move_cmd)
                    distance_to_travel = self.count_distance_to_travel(i)
                    self.rate.sleep()
                    self.robot_path_x.append(self.pose.x)
                    self.robot_path_y.append(self.pose.y)
                    print('{}, move = x: {}, y: {}, dtt: {}'.format(i, self.pose.x, self.pose.y, distance_to_travel))
                
                self.pub.publish(Twist())

                move_cmd = Twist()
                move_cmd.angular.z = speed
                rotation_to_rotate = self.count_rotation_to_rotate(i)
                while rotation_to_rotate > 0.02:
                    self.pub.publish(move_cmd)
                    self.rate.sleep()
                    rotation_to_rotate = self.count_rotation_to_rotate(i)
                    print('{}, rotate = rotation: {}, rtt: {}'.format(i, self.pose.theta, rotation_to_rotate))
                
                self.pub.publish(Twist())
            
            print('final pose= x: {}, y: {}, theta: {}'.format(self.pose.x, self.pose.y, self.pose.theta))
            visualisation = Visualisation(self.robot_path_x, self.robot_path_y)
            visualisation.show_map()
            break

    def count_distance_to_travel(self, i):
        desired_pose = self.desired_pose[i]
        if i % 2 == 0:
            distance_to_travel = desired_pose.x - self.pose.x
        else:
            distance_to_travel = desired_pose.y - self.pose.y
        
        return distance_to_travel if i <= 1 else abs(distance_to_travel)
    
    def count_rotation_to_rotate(self, i):
        desired_theta = self.desired_theta[i]
        rotation_to_rotate = desired_theta - self.pose.theta
        return rotation_to_rotate if i <= 1 else abs(rotation_to_rotate)
        # return rotation_to_rotate if self.pose.theta <= 0 and rotation_to_rotate > 0 else 0

    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta = yaw

if __name__ == '__main__':
    try:
        robot = TurtleBo(0, 0)
        robot.close_loop_move()
    except rospy.ROSInterruptException:
        pass