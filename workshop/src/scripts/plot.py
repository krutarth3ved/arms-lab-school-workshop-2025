#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi
import numpy as np

plt.ion()

class CircleTrajectoryPlot:
    def __init__(self, topic_name='/odom'):
        self.x_vals = []
        self.y_vals = []
        self.robot_yaw = 0.0
        self.robot_pos = (0.0, 0.0)
        rospy.Subscriber(topic_name, Odometry, self.odom_callback)

    def odom_callback(self, data):
        pos = data.pose.pose.position
        orient = data.pose.pose.orientation
        self.robot_pos = (pos.x, pos.y)
        self.x_vals.append(pos.x)
        self.y_vals.append(pos.y)

        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.robot_yaw = yaw

    def plot_live(self):
        rate = rospy.Rate(10)  # 10 Hz
        radius = 1.0
        theta = np.linspace(0, 2 * np.pi, 300)
        circle_x = radius * np.cos(theta)
        circle_y = radius * np.sin(theta)

        while not rospy.is_shutdown():
            plt.clf()
            plt.plot(circle_x, circle_y, 'r--', label='Ideal Circle')
            plt.plot(self.x_vals, self.y_vals, 'b-', label='Robot Trajectory')

            # Plot current robot pose
            x, y = self.robot_pos
            plt.plot(x, y, 'bo')
            plt.quiver(x, y, np.cos(self.robot_yaw), np.sin(self.robot_yaw),
                       angles='xy', scale_units='xy', scale=1, color='b', width=0.005)

            plt.title("Live Circular Trajectory - TurtleBot")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.legend()
            plt.axis('equal')
            plt.grid(True)
            plt.pause(0.001)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('circle_trajectory_plotter')
    plotter = CircleTrajectoryPlot(topic_name='/odom')
    plotter.plot_live()