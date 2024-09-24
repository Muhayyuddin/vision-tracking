#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

import sys
from math import pi

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
# From this package
from usv_pid.pypid import Pid


class USV_PID_Control(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """
    super().__init__('usv_pid_control')
    # Setup Yaw Pid
    self.ypid = Pid(0.0, 0.0, 0.0)
    self.ypid.set_setpoint(0.0)
    # self.pid.set_inputisangle(True,pi)
    self.ypid.set_derivfeedback(True)  # D term in feedback look
    fc = 20;  # cutoff freq in hz
    wc = fc*(2.0*pi)  # cutoff freq. in rad/s
    self.ypid.set_derivfilter(1, wc)
    self.ypid.set_maxIout(1.0)
    # Setup Velocity Pid
    self.vpid = Pid(0.0, 0.0, 0.0)
    self.vpid.set_setpoint(0.0)
    self.vpid.set_maxIout(1.0)
    # self.pid.set_inputisangle(True,pi)
    self.vpid.set_derivfeedback(True)  # D term in feedback look
    fc = 20;  # cutoff freq in hz
    wc = fc*(2.0*pi)  # cutoff freq. in rad/s
    self.vpid.set_derivfilter(1, wc)

    # Initialize some bits as none - for now
    self.drivemsg = None
    self.publisher = None
    self.lasttime = None
    # For diagnosing/tuning PID
    self.vpubdebug = None
    self.ypubdebug = None

    # Setup outbound messages
    self.left_cmd = Float64()
    self.right_cmd = Float64()
    
    # Setup publisher
    self.left_publisher = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
    self.right_publisher = self.create_publisher(Float64,'/usv/right/thrust/cmd_thrust', 10)
    self.timer_period = 0.1  # seconds
    self.timer = self.create_timer(self.timer_period, self.callback)

    # Setup subscribers
    self.s1 = self.create_subscription(Odometry,'/usv/odom',self.odom_callback,10)
    self.s2 = self.create_subscription(Twist,"/cmd_vel",self.twist_callback,10)
    self.dt = 0.0
    self.dyaw = 0.0
    self.dx = 0.0
    print('all set')

  def callback(self):
    # print("dt: %.6f"%dt)
    if self.dt < 1.0e-6:
      self.get_logger().info('USV Control dt too small "%f"'%self.dt)
      return
          
    yout = self.ypid.execute(self.dt,self.dyaw)
    torque = yout[0]

    # Velocity control
    vout = self.vpid.execute(self.dt,self.dx)
    thrust = vout[0]

    # I believe drive messages are scaled to -1.0 to 1.0
    # Scale so that no one output saturates
    '''
    mag = abs(torque)+abs(thrust)
    if mag > 1.0:

    torque = torque/mag
    thrust = thrust/mag
    '''

    # rospy.loginfo('Torque: %.3f, Thrust: %.3f'%(torque,thrust))
    '''
    self.drivemsg.left=-1*torque + thrust
    self.drivemsg.right=torque + thrust
    self.publisher.publish(self.drivemsg)
    '''
    self.left_cmd.data = (-1.0*torque + thrust)
    self.right_cmd.data = (torque + thrust)
    print(self.left_cmd.data )
    print(self.right_cmd.data )
    self.left_publisher.publish(self.left_cmd)
    self.right_publisher.publish(self.right_cmd)

  def twist_callback(self, msg):
    self.ypid.set_setpoint(msg.angular.z)
    self.vpid.set_setpoint(msg.linear.x)

  def odom_callback(self,msg):
    # Yaw Control
    print("in odom callback")
    self.dyaw = msg.twist.twist.angular.z # measured rate (process variable)
    now = self.get_clock().now()
    if self.lasttime is None:
      self.lasttime = now
      return
    self.dt = now.to_msg().sec-self.lasttime.to_msg().sec
    self.lasttime = now
    self.dx = msg.twist.twist.linear.x

  
 
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  # ROS Parameters
  
  yawKp = 40.0
  yawKd = 0.0
  yawKi = 0.0

  velKp = 60.0
  velKd = 0.0
  velKi = 20.0

  # Create the node
  pid_control = USV_PID_Control()
  # Set initial gains from parameters
  pid_control.ypid.Kp = yawKp
  pid_control.ypid.Kd = yawKd
  pid_control.ypid.Ki = yawKi
  pid_control.vpid.Kp = velKp
  pid_control.vpid.Kd = velKd
  pid_control.vpid.Ki = velKi


  try:
    rclpy.spin(pid_control)
  except KeyboardInterrupt:
    pass
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

    
 