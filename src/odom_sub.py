#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from math import sqrt
import matplotlib.pyplot as plt

global x,y, vel_x, vel_y, speed
x = y = vel_x = vel_y = speed = 0
def hectorodomsub(msg):
    hector_x.append(msg.pose.pose.position.x)
    hector_y.append(msg.pose.pose.position.y)
    hector_vel_x.append(-msg.twist.twist.linear.x)
    hector_vel_y.append(-msg.twist.twist.linear.y)
    hector_speed.append(sqrt((msg.twist.twist.linear.x**2)+(msg.twist.twist.linear.y**2)+(msg.twist.twist.linear.z**2)))
    gt_x.append(x)
    gt_y.append(y)
    gt_vel_x.append(vel_x)
    gt_vel_y.append(vel_y)
    gt_speed.append(speed)


def groundtruthsub(msg):
    global x,y,vel_x, vel_y, speed
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    speed = sqrt((msg.twist.twist.linear.x**2)+(msg.twist.twist.linear.y**2)+(msg.twist.twist.linear.z**2))


hector_x = []
hector_y = []

gt_x = []
gt_y = []

hector_vel_x = []
hector_vel_y = []
hector_speed = []

gt_vel_x = []
gt_vel_y = []
gt_speed = []

rospy.init_node("subodom",anonymous=True)
rospy.Subscriber('/uav1/odometry/odom_local', Odometry, hectorodomsub)
rospy.Subscriber('/uav1/ground_truth', Odometry, groundtruthsub)
while not rospy.is_shutdown():
    rospy.spin()

figure, axis = plt.subplots(1, 1)
axis.plot(hector_x, hector_y, color='r', label='Hector Position Odometry')
axis.plot(gt_x, gt_y, color='b', label='Ground Truth')
plt.show()

figure, axis = plt.subplots(1, 1)
axis.plot(hector_vel_x, color='r', label='Hector x velocity')
axis.plot(gt_vel_x, color='b', label='Ground Truth x velocity')
plt.show()

figure, axis = plt.subplots(1, 1)
axis.plot(hector_vel_y, color='r', label='Hector y velocity')
axis.plot(gt_vel_y, color='b', label='Ground Truth y velocity')
plt.show()

figure, axis = plt.subplots(1, 1)
axis.plot(hector_speed, color='r', label='Hector speed')
axis.plot(gt_speed, color='b', label='Ground Truth speed')
plt.show()

