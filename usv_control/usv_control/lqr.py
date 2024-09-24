#!/usr/bin/env python
from datetime import datetime
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt
import csv

class LQRControl(Node):
    def __init__(self, engaged=False, yaw_cntrl=True, vel_cntrl=True):
        super().__init__('lqr_control')
        self.engaged = engaged
        self.yaw_cntrl = yaw_cntrl
        self.vel_cntrl = vel_cntrl

        # USV parameters
        self.m = 800.0  # Mass in kg
        self.I_zz = 2027.475  # Moment of inertia in kg·m^2
        self.l = 1.55  # Distance between thrusters in meters
        self.u_clip = 4.0  # Surge velocity clip in m/s
        self.u_dot_clip = 2.0  # Surge acceleration clip in m/s^2
        self.r_clip = np.deg2rad(8.0)  # Yaw rate clip in rad/s
        self.r_dot_clip = np.deg2rad(30.0)  # Yaw acceleration clip in rad/s^2
        self.k = 0.1  # Thrust coefficient
        self.C_psi_err = np.pi / 4  # Yaw error coefficient

        # LQR Matrices (To be tuned)
        self.Q_yaw = np.diag([10.0, 1.0])  # State cost matrix for yaw
        self.R_yaw = np.array([[0.1]])      # Control cost matrix for yaw
        self.Q_vel = np.diag([10.0])        # State cost matrix for velocity
        self.R_vel = np.array([[0.1]])      # Control cost matrix for velocity

        # State vectors
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.vel = 0.0

        # Desired values
        self.desired_yaw = 0.0
        self.desired_vel = 0.0

        # Lists for storing errors over time
        self.time_list = []
        self.yaw_error_list = []
        self.vel_error_list = []
        self.left_cmd_list = []
        self.right_cmd_list = []

        
        # Initialize some bits as none - for now
        self.lasttime = None
        self.left_cmd = Float64()
        self.right_cmd = Float64()

        # Setup publisher
        self.left_publisher = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
        self.right_publisher = self.create_publisher(Float64, '/usv/right/thrust/cmd_thrust', 10)

        # Setup subscribers
        self.create_subscription(Odometry, '/usv/odom', self.odom_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.twist_callback, 10)
        self.create_subscription(Imu, '/usv/imu/data', self.imu_callback, 50)

    def imu_callback(self, msg):
        euler = self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.yaw = euler[2]

    def twist_callback(self, msg):
        self.desired_yaw = msg.angular.z
        self.desired_vel = msg.linear.x

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now.to_msg().sec - self.lasttime.to_msg().sec
        self.lasttime = now

        # Get current yaw rate and velocity from odom message
        self.yaw_rate = msg.twist.twist.angular.z
        self.vel = msg.twist.twist.linear.x

        # Clip velocity and yaw rate
        self.vel = np.clip(self.vel, -self.u_clip, self.u_clip)
        self.yaw_rate = np.clip(self.yaw_rate, -self.r_clip, self.r_clip)

        # Calculate errors
        yaw_error = self.desired_yaw - self.yaw
        vel_error = self.desired_vel - self.vel
        
        # Store errors over time
        self.time_list.append(self.get_clock().now().nanoseconds / 1e9)
        self.yaw_error_list.append(yaw_error)
        self.vel_error_list.append(vel_error)

        # Yaw Control
        if self.yaw_cntrl:
            yaw_state = np.array([yaw_error, self.yaw_rate])
            torque = self.lqr_control(yaw_state, self.Q_yaw, self.R_yaw)
        else:
            torque = 0.0

        # Velocity Control
        if self.vel_cntrl:
            vel_state = np.array([vel_error])
            thrust = self.lqr_control(vel_state, self.Q_vel, self.R_vel)
        else:
            thrust = 0.0

        # Apply thrust and torque to the left and right thrusters
        self.left_cmd.data = (-1.0 * torque + thrust)
        self.right_cmd.data = (torque + thrust)
        self.left_cmd_list.append(self.left_cmd.data)
        self.right_cmd_list.append(self.right_cmd.data)
       
        self.left_publisher.publish(self.left_cmd)
        self.right_publisher.publish(self.right_cmd)

 


    def lqr_control(self, state, Q, R):
        """
        LQR control: Solves the continuous-time Algebraic Riccati Equation (ARE) and
        computes the optimal gain K.
        """
        A = np.array([[0, 1], [0, 0]]) if len(state) == 2 else np.array([[0]])
        B = np.array([[0], [1]]) if len(state) == 2 else np.array([[1]])
        
        # Solve the Riccati equation for P using scipy's solve_continuous_are
        P = solve_continuous_are(A, B, Q, R)
        # Compute the LQR gain matrix K
        K = np.linalg.inv(R) @ B.T @ P
        # Calculate the optimal control input
        control_input = K @ state
        return control_input[0]

    def euler_from_quaternion(self, x, y, z, w):
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
    def save_yaw_error_csv(self, filename="yaw_error_lqr.csv"):
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
        filename = f"{current_time}_lqr_thrust_data.csv"

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
    lqr_control = LQRControl(True, True, True)

    try:
        rclpy.spin(lqr_control)
    except KeyboardInterrupt:
        pass

    # After stopping, plot the error data
    #lqr_control.plot_errors()

    # Save yaw error data to CSV
    lqr_control.save_yaw_error_csv()
    lqr_control.save_thrust_data_csv()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
