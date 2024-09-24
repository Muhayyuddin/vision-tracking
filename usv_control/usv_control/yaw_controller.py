#!/usr/bin/env python
'''
Example of using the pypid module for heading control of a USV.
'''
# Python
import imp
import sys
from math import pi
from turtle import left
import rclpy
from rclpy.node import Node
# ROS
#from dynamic_reconfigure.server import Server
#from kingfisher_control.cfg import YawDynamicConfig
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
#from kingfisher_msgs.msg import Drive
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from usv_msgs.msg import PidControlDiagnose
# BSB
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
 
from usv_pid.pypid import Pid


class Heading_Control(Node):
    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('heading_control')
        self.Kp=55.0
        self.Ki=0.005
        self.Kd=55.0
        self.yaw=0.0
        self.pid = Pid(self.Kp,self.Ki,self.Kd)
        self.pid.set_setpoint(-pi/2)
        self.pid.set_inputisangle(True,pi)
        self.pid.set_derivfeedback(True)  # D term in feedback look
        fc = 50;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.pid.set_derivfilter(1,wc)
        self.yaw_cmd = Float64()
        self.debugmsg = PidControlDiagnose()
        self.lasttime = None
        # For diagnosing/tuning PID
        self.left_publisher = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
        self.right_publisher = self.create_publisher(Float64,'/usv/right/thrust/cmd_thrust', 10)
        self.timer_period = 0.25  # seconds
        self.timer = self.create_timer(self.timer_period, self.callback)
        self.pubdebug = self.create_publisher(PidControlDiagnose,"pid_debug",10)
        # Setup subscribers
        self.create_subscription(Imu,'/usv/imu/data',self.imu_callback,50)
        self.create_subscription(Twist,"cmd_vel",self.twist_calback,10)

        
    def set_setpoint(self,msg):
        self.pid.set_setpoint(msg.data)
        
    def twist_calback(self,msg):
        self.pid.set_setpoint(msg.angular.z)

    def imu_callback(self,msg):

        euler = self.euler_from_quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        self.yaw = euler[2]

    def callback(self):

        now = self.get_clock().now().to_msg().sec

        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now-self.lasttime
        self.lasttime = now
        print("dt: %f"%dt)
        print("yaw value is : %f"%self.yaw)

        out = self.pid.execute(dt,self.yaw)
        torque = out[0]
        left = Float64()
        right = Float64()

        left.data = -1*torque
        right.data = torque
        self.yaw_cmd = torque
        self.left_publisher.publish(left)
        self.right_publisher.publish(right)

        if not (self.pubdebug is None):
            self.debugmsg.pid = out[0]
            self.debugmsg.p = out[1]
            self.debugmsg.i = out[2]
            self.debugmsg.d = out[3]
            self.debugmsg.error = out[4]
            self.debugmsg.setpoint = out[5]
            self.debugmsg.derivative= out[6]
            self.debugmsg.integrals = out[7]
            self.pubdebug.publish(self.debugmsg)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
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
        
        return roll_x, pitch_y, yaw_z # in radians
    

def main(args=None):    
    rclpy.init(args=args)
    
    heading_control=Heading_Control()

    try:
        rclpy.spin(heading_control)
    except KeyboardInterrupt:
        pass
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()