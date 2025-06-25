#!/usr/bin/env python3

# Modified from hector_ui/src/ui_hector_quad.py node from https://github.com/RAFALAMAO/hector-quadrotor-noetic
# Place it under hector_ui/src/ and build package again.

# Run any hector_quadrotor launch file that opens a gazebo world, loads backrgound nodes for hector quadrotor and spawns drone model in it.
# example: roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
# or example: roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
# Then make the node executable(chmod +x z_pid_gui.py) and run this node as:  rosrun hector_ui z_pid_gui.py

import rospy
import tkinter as tk
from tkinter import ttk
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class ZControlGUI:
    def __init__(self, master):
        self.master = master
        master.title("Z PID GUI")
        master.geometry("400x500")

        self.kp = tk.DoubleVar(value=1.0)
        self.kd = tk.DoubleVar(value=0.0)
        self.target_z = tk.DoubleVar(value=1.5)

        self.current_z = 0.0
        self.last_z = 0.0
        self.last_time = None  # <- Init as None

        rospy.init_node('z_pid_gui_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)

        ttk.Label(master, text="Kp (Z)").pack(pady=(10, 0))
        ttk.Scale(master, from_=0.0, to=5.0, orient='horizontal', variable=self.kp).pack(fill='x', padx=20)

        ttk.Label(master, text="Kd (Z)").pack(pady=(10, 0))
        ttk.Scale(master, from_=0.0, to=2.0, orient='horizontal', variable=self.kd).pack(fill='x', padx=20)

        ttk.Label(master, text="Target Z (meters)").pack(pady=(20, 5))
        ttk.Scale(master, from_=5.0, to=0.0, orient='vertical', length=300, variable=self.target_z).pack(pady=10)

        self.kp_str = tk.StringVar()
        self.kd_str = tk.StringVar()
        self.z_str = tk.StringVar()

        ttk.Label(master, textvariable=self.kp_str).pack()
        ttk.Label(master, textvariable=self.kd_str).pack()
        ttk.Label(master, textvariable=self.z_str).pack()

        self.master.after(100, self.control_loop)
        self.update_labels()

    def update_labels(self):
        self.kp_str.set(f"Kp: {self.kp.get():.2f}")
        self.kd_str.set(f"Kd: {self.kd.get():.2f}")
        self.z_str.set(f"Target Z: {self.target_z.get():.2f} m")
        self.master.after(100, self.update_labels)


    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z

    def control_loop(self):
        now = rospy.Time.now()
        if self.last_time is None:
            self.last_time = now
            self.last_z = self.current_z
            self.master.after(100, self.control_loop)
            return

        dt = (now - self.last_time).to_sec()
        dz = self.current_z - self.last_z
        vz_error = 0.0 if dt == 0 else dz / dt

        error = self.target_z.get() - self.current_z
        vz = self.kp.get() * error - self.kd.get() * vz_error

        twist = Twist()
        twist.linear.z = vz
        self.cmd_pub.publish(twist)

        self.last_z = self.current_z
        self.last_time = now

        self.master.after(100, self.control_loop)

if __name__ == '__main__':
    try:
        root = tk.Tk()
        gui = ZControlGUI(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
