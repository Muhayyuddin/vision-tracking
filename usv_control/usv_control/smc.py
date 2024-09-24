#!/usr/bin/env python
'''
Using Sliding Mode Control (SMC) to control yaw and velocity for a USV with increased speed
'''
from datetime import datetime
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import csv

class SlidingModeControl(Node):
    def __init__(self, engaged=False, yaw_cntrl=True, vel_cntrl=True):
        super().__init__('sliding_mode_control')
        
        self.engaged = engaged
        self.yaw_cntrl = yaw_cntrl
        self.vel_cntrl = vel_cntrl

        # Sliding Mode Control Parameters for Yaw (Increased Speed)
        self.lambda_ = 2  # Increased sliding surface parameter for yaw
        self.k = 5          # Increased control gain for yaw
        self.boundary_layer = 0.5 # Reduced boundary layer thickness for faster response

        # Sliding Mode Control Parameters for Velocity (Surge) (Increased Speed)
        self.vel_lambda = 10  # Increased sliding surface parameter for velocity
        self.vel_k = 15         # Increased control gain for velocity
        self.vel_boundary_layer = 0.5  # Reduced boundary layer for faster velocity response

        # Initialize control commands
        self.left_cmd = Float64()
        self.right_cmd = Float64()

        # Setup publishers
        self.left_publisher = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
        self.right_publisher = self.create_publisher(Float64, '/usv/right/thrust/cmd_thrust', 10)

        # Setup subscribers
        self.create_subscription(Odometry, '/usv/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.create_subscription(Imu, '/usv/imu/data', self.imu_callback, 50)

        # Internal states
        self.yaw = 0.0
        self.vel = 0.0
        self.desired_yaw = 0.0
        self.desired_vel = 0.0

        # Lists for storing errors over time
        self.time_list = []
        self.yaw_error_list = []
        self.vel_error_list = []
        self.left_cmd_list = []
        self.right_cmd_list = []


        # Time tracking
        self.lasttime = None

    def imu_callback(self, msg):
        # Extract the yaw (Z-axis rotation) from the IMU quaternion
        euler = self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.yaw = euler[2]

    def twist_callback(self, msg):
        # Set desired yaw and velocity from incoming Twist messages
        self.desired_yaw = msg.angular.z
        self.desired_vel = msg.linear.x

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now.to_msg().sec - self.lasttime.to_msg().sec
        self.lasttime = now

        # Compute yaw and velocity errors
        yaw_error = self.desired_yaw - self.yaw
        vel_error = self.desired_vel - msg.twist.twist.linear.x

        # Store errors over time
        self.time_list.append(self.get_clock().now().nanoseconds / 1e9)
        self.yaw_error_list.append(yaw_error)
        self.vel_error_list.append(vel_error)

        # Sliding Mode Control for Yaw
        yaw_control = self.sliding_mode_control(yaw_error, self.lambda_, self.k, self.boundary_layer)

        # Sliding Mode Control for Velocity (Surge)
        vel_control = self.sliding_mode_control(vel_error, self.vel_lambda, self.vel_k, self.vel_boundary_layer)

        # Compute thrust for left and right motors based on yaw and velocity control
        # Corrected for forward motion: Apply the same vel_control to both thrusters
        self.left_cmd.data = -0.1 * yaw_control + vel_control
        self.right_cmd.data = yaw_control + vel_control
        print("appending thrust ",self.left_cmd, "  ",self.right_cmd)
        self.left_cmd_list.append(self.left_cmd.data)
        self.right_cmd_list.append(self.right_cmd.data)
        # Publish thrust commands
        if self.engaged:
            self.left_publisher.publish(self.left_cmd)
            self.right_publisher.publish(self.right_cmd)


    def sliding_mode_control(self, error, lambda_, k, boundary_layer):
        # Sliding surface
        s = lambda_ * error

        # Control law using a saturation function to handle chattering
        control_signal = k * self.sat(s / boundary_layer)
        return control_signal

    def sat(self, value):
        # Saturation function to avoid chattering (SMC technique)
        if value > 1.0:
            return 1.0
        elif value < -1.0:
            return -1.0
        return value

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

    def plot_errors(self):
        """
        Plot the yaw and velocity errors over time using matplotlib.
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

        # Display the plot
        plt.tight_layout()
        plt.show()

    def save_yaw_error_csv(self, filename="yaw_error_smc.csv"):
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
        filename = f"{current_time}_smc_thrust_data.csv"

        start_time = self.time_list[0]
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Left Command", "Right Command", "Total Thrust Magnitude"])
            for t, left_cmd, right_cmd in zip(self.time_list, self.left_cmd_list, self.right_cmd_list):
                relative_time = t - start_time  # Make time start from zero
                total_thrust = abs(left_cmd + right_cmd)
                writer.writerow([relative_time, left_cmd, right_cmd, total_thrust])
        print(f"Thrust data saved to {filename}")
   

def main(args=None):
    rclpy.init(args=args)
    sliding_mode_control = SlidingModeControl(True, True, True)

    try:
        rclpy.spin(sliding_mode_control)
    except KeyboardInterrupt:
        pass

    # After stopping, plot the error data
    #sliding_mode_control.plot_errors()

    # Save yaw error data to CSV
    sliding_mode_control.save_yaw_error_csv()
    sliding_mode_control.save_thrust_data_csv()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
