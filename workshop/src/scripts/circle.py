#!/usr/bin/env python3

import math
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

pi = np.pi

class PIDControl:
    def __init__(self, name, kv=1.0, ka=3.0, publisher_name='/cmd_vel', odom_name='/odom'):
        self.name = name
        self.kv = kv
        self.ka = ka
        self.vmax = 0.22
        self.wmax = pi / 2.0
        self.pub_cmd_vel = rospy.Publisher(publisher_name, Twist, queue_size=1)
        self.bot_location = Pose2D()
        self.bot_vel = Twist()
        rospy.Subscriber(odom_name, Odometry, self.callback_odom)

    def callback_odom(self, data):
        self.bot_location.x = data.pose.pose.position.x
        self.bot_location.y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.bot_location.theta = yaw if yaw >= 0 else yaw + 2 * pi

    def E_alpha_i(self, goal_pose, cur_pose):
        alpha_i = math.atan2(goal_pose.y - cur_pose.y, goal_pose.x - cur_pose.x)
        alpha_i = alpha_i if alpha_i >= 0 else alpha_i + 2 * pi
        error_alpha = alpha_i - cur_pose.theta
        # Normalize to [-pi, pi]
        error_alpha = (error_alpha + pi) % (2 * pi) - pi
        return error_alpha

    def Dist(self, goal_pose, cur_pose):
        return math.sqrt((goal_pose.x - cur_pose.x)**2 + (goal_pose.y - cur_pose.y)**2)

    def Velocity_tracking_law(self, goal_pose):
        e_alpha = self.E_alpha_i(goal_pose, self.bot_location)
        D_i = self.Dist(goal_pose, self.bot_location)
        
        V = self.kv * math.cos(e_alpha) * D_i
        W = self.ka * e_alpha

        # Saturation
        V = max(-self.vmax, min(self.vmax, V))
        W = max(-self.wmax, min(self.wmax, W))

        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = W
        return cmd

if __name__ == '__main__':
    rospy.init_node('PID_circle_tracker')
    pid_controller = PIDControl("circle_PID", publisher_name="/cmd_vel", odom_name="/odom")
    rate = rospy.Rate(20)

    radius = 2.0         # Radius of circle
    angular_speed = 0.2  # Radians per second
    theta = 0.0          # Initial angle

    while not rospy.is_shutdown():
        # Generate circular reference point
        x_ref = radius * math.cos(theta)
        y_ref = radius * math.sin(theta)
        goal = Pose2D(x=x_ref, y=y_ref, theta=0.0)

        cmd = pid_controller.Velocity_tracking_law(goal)
        pid_controller.pub_cmd_vel.publish(cmd)

        theta += angular_speed * (1.0 / 20.0)  # Increment angle over time
        if theta > 2 * pi:
            theta -= 2 * pi

        rate.sleep()