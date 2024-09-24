#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import csv
from datetime import datetime

class FrameListener(Node):

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('usv2dpose')

        # Subscription and publisher setup
        self.usv_pose = self.create_subscription(TFMessage, '/usv/pose_static', self.odometry, 10)
        self.odom_pub = self.create_publisher(Odometry, '/usv/odom', 10)
        self.index = 8 # Adjust index based on your setup
        self.br = TransformBroadcaster(self)
        self.odom = Odometry()

        # Initialize CSV file and start time
        self.start_time = None
        self.csv_filename = f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_usv_position_data.csv"
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Relative Time (s)", "Position X", "Position Y"])

    def odometry(self, msg):
        t = TransformStamped()
        print(msg.transforms)
        t.header.stamp = msg.transforms[self.index].header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'usv'

        # Set position and orientation data from the message
        t.transform.translation.x = msg.transforms[self.index].transform.translation.x
        t.transform.translation.y = msg.transforms[self.index].transform.translation.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = msg.transforms[self.index].transform.rotation.x
        t.transform.rotation.y = msg.transforms[self.index].transform.rotation.y
        t.transform.rotation.z = msg.transforms[self.index].transform.rotation.z
        t.transform.rotation.w = msg.transforms[self.index].transform.rotation.w

        # Send the transformation
        self.br.sendTransform(t)

        # Set the odometry message data
        self.odom.header.stamp = msg.transforms[self.index].header.stamp
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose.position.x = msg.transforms[self.index].transform.translation.x
        self.odom.pose.pose.position.y = msg.transforms[self.index].transform.translation.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = msg.transforms[self.index].transform.rotation.x
        self.odom.pose.pose.orientation.y = msg.transforms[self.index].transform.rotation.y
        self.odom.pose.pose.orientation.z = msg.transforms[self.index].transform.rotation.z
        self.odom.pose.pose.orientation.w = msg.transforms[self.index].transform.rotation.w
        self.odom.child_frame_id = "usv"

        # Publish the odometry data
        self.odom_pub.publish(self.odom)
        print(f"Odom Position X: {self.odom.pose.pose.position.x}, Odom Position Y: {self.odom.pose.pose.position.y}")


        # Save position data to CSV
        self.save_position_to_csv(self.odom.pose.pose.position.x, 
                                 self.odom.pose.pose.position.y,
                                  msg.transforms[self.index].header.stamp)

    def save_position_to_csv(self, x, y, stamp):
        """
        Save the position (x, y) and relative time to a CSV file.
        """
        if self.start_time is None:
            self.start_time = stamp.sec + stamp.nanosec * 1e-9

        # Calculate the relative time
        current_time = stamp.sec + stamp.nanosec * 1e-9
        relative_time = current_time - self.start_time

        # Save data to CSV
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([relative_time, x, y])

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
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
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    frame_listener_node = FrameListener()

    # Spin the node so the callback function is called
    try:
        rclpy.spin(frame_listener_node)
    except KeyboardInterrupt:
        pass

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
