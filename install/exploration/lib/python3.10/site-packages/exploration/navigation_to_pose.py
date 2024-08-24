import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
import numpy as np
from pyquaternion import Quaternion
from rclpy.clock import Clock
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations

# Import Action Client library, action type and other relevant library
# !!!!

class actionServerControl(Node):

  def __init__(self) -> None:
    super().__init__('action_server_control')
    print('Create Action')
    # Create Action client to contol Nav2 stack
    # !!!!
    self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
    self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
    self.action_client.wait_for_server()
    # Wait for server to be ready
    # !!!!
    self.clock = self.get_clock()
    print('Init completed')
#  self.move_to_position(2.0, 1.5, 0.0)

    self.get_logger().info('Goal Sended')
    self.odom_check = False
    self.base_check = False

    self.odom_wrt_map = Transform()
    self.base_wrt_odom = Transform()
    self.pos = [0.0, 0.0, 0.0]
    self.ang = [0.0, 0.0, 0.0]


  def map_callback(self, msg):
    pass
    #  print(msg.info.width)
    #  print(msg.info.height)         


  def tf_callback(self, msg):

    if (msg.transforms[0].child_frame_id == "odom"):
        self.odom_wrt_map = msg.transforms[0].transform
        print("odom_wrt_map = ", [self.odom_wrt_map.translation.x, self.odom_wrt_map.translation.y, self.odom_wrt_map.translation.z])
        self.odom_check = True

    if (msg.transforms[0].child_frame_id == "base_footprint"):
        self.base_wrt_odom = msg.transforms[0].transform
        print("base_wrt_odom = ",  [self.base_wrt_odom.translation.x, self.base_wrt_odom.translation.y, self.base_wrt_odom.translation.z])
        self.base_check = True
    
    if self.odom_check and self.base_check:
        base_x = self.odom_wrt_map.translation.x + self.base_wrt_odom.translation.x
        base_y = self.odom_wrt_map.translation.y + self.base_wrt_odom.translation.y
        base_z = self.odom_wrt_map.translation.z + self.base_wrt_odom.translation.z

        base_quat_x = self.base_wrt_odom.rotation.x
        base_quat_y = self.base_wrt_odom.rotation.y
        base_quat_z = self.base_wrt_odom.rotation.z
        base_quat_w = self.base_wrt_odom.rotation.w
        self.pos = [base_x, base_y, base_z]
        self.ang = tf_transformations.euler_from_quaternion([base_quat_x, base_quat_y, base_quat_z, base_quat_w])
        print("quat = ", [base_quat_x, base_quat_y, base_quat_z, base_quat_w])
        print("base_pos = ", self.pos)
        print("base_angle = ", self.ang)


  def move_to_position(self,x=0.0,y=0.0, theta=0.0):
     """
        x = x position to reach
        y = y position to reach
        theta = angle expressed in radiants to reach

     """

     # Create Goal Message
     my_quaternion = Quaternion(axis=[0, 0, 1], angle=theta)
     qw, qx ,qy ,qz  = list(my_quaternion)

     goal_pose = NavigateToPose.Goal()
     #goal_pose.header.stamp = self._clock.now()
     goal_pose.pose.header.frame_id = 'map'
     goal_pose.pose.pose.position.x = x
     goal_pose.pose.pose.position.y = y
     
     goal_pose.pose.pose.orientation.x = qx
     goal_pose.pose.pose.orientation.y = qy
     goal_pose.pose.pose.orientation.z = qz
     goal_pose.pose.pose.orientation.w = qw

     # send goal and return
     self.action_client.send_goal_async(goal_pose)
     # !!!!
     return 


def main():
  # Class creation
  # !!!!
  rclpy.init()

  action_server = actionServerControl()

  rclpy.spin(action_server)

  
  # call class method to send command to action server
  # !!!!

  return

if __name__ == '__main__':
  main()