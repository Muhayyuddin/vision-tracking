#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import pi
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

# From this package
from usv_pid.pypid import Pid


class PositionControl(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """
    super().__init__('position_control')
   
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

    # Setup subscribers
    self.s1 = self.create_subscription(Odometry,'/usv/odom',self.odom_callback,10)
    #self.s2 = self.create_subscription(Twist,"cmd_vel",self.twist_callback,10)
    self.create_subscription(Float64,"set_position",self.set_setpoint,10)

    print('all set')


  def set_setpoint(self, msg):
    self.vpid.set_setpoint(msg.data)

  def odom_callback(self,msg):
    # Yaw Control
    now = self.get_clock().now()
    if self.lasttime is None:
      self.lasttime = now
      return
    dt = now.to_msg().sec-self.lasttime.to_msg().sec
    self.lasttime = now
    # print("dt: %.6f"%dt)
    if dt < 1.0e-6:
      self.get_logger().info('USV Control dt too small "%f"'%dt)
      return

    # Velocity control
    dx = msg.pose.pose.position.x
    vout = self.vpid.execute(dt,dx)
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
    self.left_cmd.data = (thrust)
    self.right_cmd.data = (thrust)
    print(self.left_cmd.data )
    print(self.right_cmd.data )
    self.left_publisher.publish(self.left_cmd)
    self.right_publisher.publish(self.right_cmd)

    if not (self.vpubdebug is None):
        self.vdebugmsg.PID = vout[0]
        self.vdebugmsg.P = vout[1]
        self.vdebugmsg.I = vout[2]
        self.vdebugmsg.D = vout[3]
        self.vdebugmsg.Error = vout[4]
        self.vdebugmsg.Setpoint = vout[5]
        self.vdebugmsg.Derivative= vout[6]
        self.vdebugmsg.Integral = vout[7]
        self.vpubdebug.publish(self.vdebugmsg)
 
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)

  velKp = 0.7
  velKd = 0.7
  velKi = 0.3
  # Create the node
  pid_position_control = PositionControl()
  # Set initial gains from parameters

  pid_position_control.vpid.Kp = velKp
  pid_position_control.vpid.Kd = velKd
  pid_position_control.vpid.Ki = velKi


  try:
    rclpy.spin(pid_position_control)
  except KeyboardInterrupt:
    pass
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

    
 