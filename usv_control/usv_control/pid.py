#!/usr/bin/env python
'''
Using instances of the pypid Pid class to control yaw and velocity with added plotting and saving functions
'''
from datetime import datetime
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from usv_pid.pypid import Pid
import matplotlib.pyplot as plt
from math import pi

import csv

class PidControl(Node):
    def __init__(self, engaged=False, yaw_cntrl=True, vel_cntrl=True):
        super().__init__('pid_control')

        # Setup Yaw PID
        self.engaged = engaged
        self.yaw_cntrl = yaw_cntrl
        self.vel_cntrl = vel_cntrl
        self.Kp = 10.0
        self.Ki = 0.01
        self.Kd = 15.0
        self.ypid = Pid(self.Kp, self.Ki, self.Kd)
        self.ypid.set_setpoint(0.0)
        self.ypid.set_inputisangle(True, pi)
        self.ypid.set_derivfeedback(True)  # D term in feedback loop
        fc = 20  # cutoff freq in hz
        wc = fc * (2.0 * pi)  # cutoff freq. in rad/s
        self.ypid.set_derivfilter(1, wc)

        self.vpid = Pid(10.0, 10.0, 0.01)
        self.vpid.set_setpoint(0.0)
        self.vpid.set_maxIout(1.0)
        self.vpid.set_derivfeedback(True)  # D term in feedback loop
        self.vpid.set_derivfilter(1, wc)

        # Initialize control variables
        self.yaw = 0.0
        self.vel = 0.0
        self.desired_yaw = 0.0
        self.desired_vel = 0.0
        self.lasttime = None

        # For storing errors
        self.time_list = []
        self.yaw_error_list = []
        self.vel_error_list = []
        self.left_cmd_list = []
        self.right_cmd_list = []


        # Setup publishers
        self.left_publisher = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
        self.right_publisher = self.create_publisher(Float64, '/usv/right/thrust/cmd_thrust', 10)

        # Setup subscribers
        self.create_subscription(Odometry, '/usv/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.create_subscription(Imu, '/usv/imu/data', self.imu_callback, 50)

    def imu_callback(self, msg):
        euler = self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.yaw = euler[2]

    def twist_callback(self, msg):
        self.desired_yaw = msg.angular.z
        self.desired_vel = msg.linear.x
        self.ypid.set_setpoint(msg.angular.z)
        self.vpid.set_setpoint(msg.linear.x)

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now.to_msg().sec - self.lasttime.to_msg().sec
        self.lasttime = now

        # Yaw Control
        if self.yaw_cntrl:
            yout = self.ypid.execute(dt, self.yaw)
            torque = yout[0]
        else:
            torque = 0.0

        # Velocity Control
        if self.vel_cntrl:
            vout = self.vpid.execute(dt, msg.twist.twist.linear.x)
            thrust = vout[0]
        else:
            thrust = 0.0

        # Compute thrust for left and right motors
        self.left_cmd = Float64()
        self.right_cmd = Float64()
        self.left_cmd.data = (-1.0 * torque + thrust)
        self.right_cmd.data = (torque + thrust)

        # Publish thrust commands
        self.left_publisher.publish(self.left_cmd)
        self.right_publisher.publish(self.right_cmd)

        # Compute and store errors
        yaw_error = self.desired_yaw - self.yaw
        vel_error = self.desired_vel - msg.twist.twist.linear.x

        self.time_list.append(self.get_clock().now().nanoseconds / 1e9)
        self.yaw_error_list.append(yaw_error)
        self.vel_error_list.append(vel_error)

        self.left_cmd_list.append(self.left_cmd.data)
        self.right_cmd_list.append(self.right_cmd.data)

    def plot_errors(self):
        """
        Plot yaw and velocity errors over time.
        """
        plt.figure(figsize=(10, 6))

        # Plot yaw error
        plt.subplot(2, 1, 1)
        plt.plot(self.time_list, self.yaw_error_list, label="Yaw Error")
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw Error (rad)')
        plt.title('Yaw Error Over Time')
        plt.grid(True)

        # Plot velocity error
        plt.subplot(2, 1, 2)
        plt.plot(self.time_list, self.vel_error_list, label="Velocity Error", color="red")
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity Error (m/s)')
        plt.title('Velocity Error Over Time')
        plt.grid(True)

        # Show plot
        plt.tight_layout()
        plt.show()

    def save_yaw_error_csv(self, filename="yaw_error_pid.csv"):
        """
        Save yaw error vs time into a CSV file, with time starting from zero.
        """
        if not self.time_list:
            print("No data to save.")
            return

        # Get the first time value to set time to start from zero
        start_time = self.time_list[0]
        
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Yaw Error (rad)"])
            for t, yaw_error in zip(self.time_list, self.yaw_error_list):
                relative_time = t - start_time  # Make time start from zero
                writer.writerow([relative_time, yaw_error])
        print(f"Yaw error data saved to {filename}")

    def save_thrust_data_csv(self):
        """
        Save time, left_cmd, right_cmd, and total thrust (magnitude) into a CSV file.
        The filename will include the current date and time.
        """
        if not self.time_list:
            print("No data to save.")
            return

        # Get the current date and time for the filename
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"{current_time}_pid_thrust_data.csv"

        start_time = self.time_list[0]
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Left Command", "Right Command", "Total Thrust Magnitude"])
            for t, left_cmd, right_cmd in zip(self.time_list, self.left_cmd_list, self.right_cmd_list):
                relative_time = t - start_time  # Make time start from zero
                total_thrust = abs(left_cmd + right_cmd)
                writer.writerow([relative_time, left_cmd, right_cmd, total_thrust])
        print(f"Thrust data saved to {filename}")

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw).
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

def main(args=None):
    rclpy.init(args=args)
    pid_control = PidControl(True, True, True)

    try:
        rclpy.spin(pid_control)
    except KeyboardInterrupt:
        pass

    # After stopping, plot the error data
    #pid_control.plot_errors()

    # Save yaw error data to CSV
    pid_control.save_yaw_error_csv()
    pid_control.save_thrust_data_csv()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
