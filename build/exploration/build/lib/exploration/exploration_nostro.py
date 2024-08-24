import rclpy
from rclpy.node import Node




from geometry_msgs.msg import Transform, PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan


from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D




import tf_transformations
import numpy as np


SQUARE_SIZE = 10
SWEET_DISTANCE = 1.5
DISTANCE_WEIGHT = 1
DISTANCE_THRESHOLD = 0.1




def create_PoseStamped(nav: BasicNavigator, xx, yy, yaw):
   # Creates a Posestamped object given x, y and angle
   pose = PoseStamped()
   pose.header.frame_id = 'map'
   pose.header.stamp = nav.get_clock().now().to_msg()
   pose.pose.position.x = xx
   pose.pose.position.y = yy
   pose.pose.position.z = 0.0
   q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
   pose.pose.orientation.x = q_x
   pose.pose.orientation.y = q_y
   pose.pose.orientation.z = q_z
   pose.pose.orientation.w = q_w


   return pose


def path_length(start, path):
   poses = path.poses
   length = 0
   for pose in poses:
       length += np.sqrt((pose.pose.position.x - start.pose.position.x)**2 + (pose.pose.position.y - start.pose.position.y)**2)
       start = pose
   print(length)
   return length




class Exploration(Node):
   


   ## Node Initialization ##
   def __init__(self) -> None:
       super().__init__('exploration')
       self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
       self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
       self.tf_subscriber = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
       self.tf_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
       


       self.odom_wrt_map = Transform()
       self.base_wrt_odom = Transform()
       self.pos = [0.0, 0.0, 0.0]
       self.ang = [0.0, 0.0, 0.0]


       self.tf_odom_cb_check = False
       self.tf_base_cb_check = False
       self.map_cb_check = False
       self.odom_cb_check = False
       self.costmap_cb_check = False   
       self.map_completed= False
       


   ## Odometry Callback ##
   def odom_callback():
       pass
       
   ## TF Callback: for position and orientation of the robot ##
   def tf_callback(self, msg:TFMessage):
       if (msg.transforms[0].child_frame_id == "odom"):
           self.odom_wrt_map = msg.transforms[0].transform
           self.tf_odom_cb_check = True


       if (msg.transforms[0].child_frame_id == "base_footprint"):
           self.base_wrt_odom = msg.transforms[0].transform
           self.tf_base_cb_check = True
   
       if self.tf_odom_cb_check and self.tf_base_cb_check:
           base_x = self.odom_wrt_map.translation.x + self.base_wrt_odom.translation.x
           base_y = self.odom_wrt_map.translation.y + self.base_wrt_odom.translation.y
           base_z = self.odom_wrt_map.translation.z + self.base_wrt_odom.translation.z


           base_quat_x = self.base_wrt_odom.rotation.x
           base_quat_y = self.base_wrt_odom.rotation.y
           base_quat_z = self.base_wrt_odom.rotation.z
           base_quat_w = self.base_wrt_odom.rotation.w


           self.pos = [base_x, base_y, base_z]
           self.ang = tf_transformations.euler_from_quaternion([base_quat_x, base_quat_y, base_quat_z, base_quat_w])  
   


   ## Map Callback: for informations about the map ##
   def map_callback(self, msg: OccupancyGrid):
       
       self.resolution = msg.info.resolution
       self.originX = msg.info.origin.position.x
       self.originY = msg.info.origin.position.y
       self.width = msg.info.width
       self.height = msg.info.height
       self.data = msg.data
       self.map_cb_check = True


   def costmap_callback(self, msg: OccupancyGrid):
       self.costmap = PyCostmap2D(msg)
       self.costmap_cb_check = True


   def scan_callback(self, msg:LaserScan):
       self.scan_ranges = msg.ranges
       self.scan_cb_check = True




   ## So it's possible to execute a callback even if inside a while loop ##
   def wait_for_callback(self, topic):
       if topic == 'map':
           while not self.map_cb_check: 
               rclpy.spin_once(self)
       elif topic == 'odom':
           while not self.tf_odom_cb_check: 
               rclpy.spin_once(self)
       elif topic == 'base':
           while not self.tf_base_cb_check: 
               rclpy.spin_once(self)
       elif topic == 'costmap':
           while not self.costmap_cb_check: 
               rclpy.spin_once(self)
       elif topic == 'scan':
           while not self.scan_cb_check: 
               rclpy.spin_once(self)


       self.tf_odom_cb_check = False
       self.tf_base_cb_check = False
       self.map_cb_check = False
       self.costmap_cb_check = False    
       self.scan_cb_check = False      


   ## From 1D array we create a 2D array with map informations ##
   def create_map_2D(self, map_data, h, w):
       return np.reshape(map_data, (h, w))


   ## Dividing the map in bigger regions with properties 
   ## depending on the pixel informations inside 


   def create_square_map(self):
       self.map2D = self.create_map_2D(self.data, self.height, self.width) 
       # self.costmap2D = self.create_map_2D(self.costmap_data, self.height, self.width)
       
       self.square_size = SQUARE_SIZE #number of pixels
       self.square_map_h = (int)(self.height / self.square_size)
       self.square_map_w = (int)(self.width / self.square_size)


       idx_matrix = 1*self.resolution*SQUARE_SIZE*np.indices((self.square_map_h, self.square_map_w)) + 0.5*self.resolution*SQUARE_SIZE
       self.square_map_priority = np.zeros((self.square_map_h, self.square_map_w))
       self.square_map_position = np.zeros((self.square_map_h, self.square_map_w, 2))
       self.square_map_distance = np.zeros((self.square_map_h, self.square_map_w))
       self.square_map_unknown = np.zeros((self.square_map_h, self.square_map_w))
       








       
       for i in range(self.square_map_h):
           for j in range(self.square_map_w):
               # position of the center of the square for every square
               self.square_map_position[i,j] = np.array([idx_matrix[1][i,j] + self.originX , idx_matrix[0][i,j] + self.originY])
               # unknown value: value between 0 and 1 depending on how many pixel of the square are uknown over the total number of pixel
               self.square_map_unknown[i,j] = 1/(SQUARE_SIZE*SQUARE_SIZE) * (np.count_nonzero(self.map2D[i*SQUARE_SIZE: (i+1)*SQUARE_SIZE, j*SQUARE_SIZE: (j+1)*SQUARE_SIZE] == -1)) 
               #self.square_map_unknown[i,j] = -4*self.square_map_unknown[i,j]**2 + 4*self.square_map_unknown[i,j]
               # distance between the square and the robot
               self.square_map_distance[i,j] = np.sqrt((self.pos[0] - self.square_map_position[i,j][0])**2 + (self.pos[1] - self.square_map_position[i,j][1])**2)
               # priority: value between 0 and 1. The more the square is unknwon and closer to the "sweet_distance" the more its priority is high
               self.square_map_priority[i,j] = np.exp(-(DISTANCE_WEIGHT*(SWEET_DISTANCE - self.square_map_distance[i,j]))**2) * self.square_map_unknown[i,j]




   def check_collision(self,i):
        goal_vector = np.array([self.goal[0] - self.pos[0], self.goal[1] - self.pos[1]]) 
        goal_angle = np.arctan2(goal_vector[1], goal_vector[0]) 
        scan_angle = (goal_angle - self.ang[2]) * 180 /np.pi
        goal_distance = np.sqrt( goal_vector[0]**2 + goal_vector[1]**2)
        goal_cost = self.costmap.getCostXY(self.costmap.worldToMap(self.goal[0], self.goal[1])[0], self.costmap.worldToMap(self.goal[0], self.goal[1])[1])
               # map_xy = self.costmap.worldToMap(xx, yy)
       # cost = self.costmap.getCostXY(map_xy[0], map_xy[1])
        print(f"\tRobot to Goal Distance: {goal_distance:.3} Angle: {int(scan_angle) }, Lidar Distance: {self.scan_ranges[int(scan_angle)]:.3}, Goal Cost: {goal_cost}")
        
        if self.scan_ranges[int(scan_angle)] <= (goal_distance + 1) or goal_cost > 40:
            print("\tCollision Found, Change goal\n")
            return True #Collision Found
        print(f"\tCollision Free Goal, Accept goal: {i}\n")
        return False #Collision Free



           
   
   def find_goal(self):
       # indices sorted by max to min priority square
       sorted_indices = np.argsort(self.square_map_priority, axis=None)[::-1]
       sorted_indices_2d = np.array(np.unravel_index(sorted_indices, self.square_map_priority.shape)).T
       for i in range(len(sorted_indices_2d)-1):
          # set goal to max priority square position
          self.max_idx = [sorted_indices_2d[i][0], sorted_indices_2d[i][1]] 
          self.goal = self.square_map_position[self.max_idx[0], self.max_idx[1]]
          if  not self.check_collision(i): break
       if i == len(sorted_indices_2d):
           print("No Path Can be Found")     






   def reach_goal(self):
        goal_pose = create_PoseStamped(self.nav, self.goal[0], self.goal[1], 0)
        start_pose = create_PoseStamped(self.nav, self.pos[0], self.pos[1], self.ang[2])
        path = self.nav.getPath(start_pose, goal_pose)
        intermediate_pose = path.poses[int(len(path.poses)-1)]


        
        x_pose = intermediate_pose.pose.position.x
        y_pose = intermediate_pose.pose.position.y




        # xx = 0.846
        # yy = 0.29
        # map_xy = self.costmap.worldToMap(xx, yy)
        # cost = self.costmap.getCostXY(map_xy[0], map_xy[1])
        # print("COOOOOOOOOOOOOST = ", cost)


        # print(f"goal = [{x_pose},{y_pose}], cost = {self.costmap.getCostIdx(self.max_idx[0] * self.width + self.max_idx[1])}")




        self.nav.goToPose(create_PoseStamped(self.nav, x_pose, y_pose, 0))
        i = 0
        while not self.nav.isTaskComplete():
            if i == 0: print(f"reaching goal pose : [{self.goal[0]:.2}, {self.goal[1]:.2}]")
            i += 1

       
   def start_exploration(self):
       # Main exploration function
       self.get_logger().info(f'Starting exploration:')
       self.nav = BasicNavigator()




       while rclpy.ok() and not self.map_completed:


           self.get_logger().info("\n\n\t\t######## STARTING EXPLORATION ########")
           self.wait_for_callback("map")
           self.wait_for_callback("costmap")
           self.create_square_map()
           self.find_goal()
           self.get_logger().info(f"Goal: [{self.goal[0]:.2},{self.goal[1]:.2}]")
           self.reach_goal()
           # self.get_logger().info(f"Goal Reched")


def main(args=None):
   rclpy.init(args=args)
   explorer = Exploration()
   explorer.start_exploration()
   explorer.destroy_node()
   rclpy.shutdown()


       
if __name__ == "__main__":
   main()