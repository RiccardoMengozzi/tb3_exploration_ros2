import rclpy
from rclpy.node import Node
from rclpy.duration import Duration



from geometry_msgs.msg import Transform, PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan


from nav2_simple_commander.robot_navigator import BasicNavigator

import tf_transformations
import numpy as np
import time

SQUARE_SIZE = 5
SWEET_DISTANCE = 0.8
DISTANCE_WEIGHT = 1
DISTANCE_THRESHOLD = 0.1

OBSTACLE_LOWER_THRESHOLD = 70
OBSTACLE_UPPER_THRESHOLD = 100
OBSTACLE_PERCENTAGE_THRESHOLD = 0.1 / SQUARE_SIZE

UNKNOWN_LOWER_THRESHOLD = 40
UNKNOWN_UPPER_THRESHOLD = 60
UNKNOWN_PERCENTAGE_THRESHOLD = 0.3

CHECK_POSE_TIME = 5
STEP_LENGTH = 30 # length of step of divided path
PATH_DIV_THRESH = 0.05

BACKTRACKING_STEP = 3
MAX_GOAL_TIME = 40


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
        self.tf_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.wall_map_publisher = self.create_publisher(OccupancyGrid, '/wall_map', 10)

        self.odom_wrt_map = Transform()
        self.base_wrt_odom = Transform()
        self.pos = [0.0, 0.0, 0.0]
        self.ang = [0.0, 0.0, 0.0]
        self.goal_history = []

        self.map_cb_check = False  
        self.map_completed= False
        self.pose_cb_check = False
        self.odom_cb_check = False
        self.base_cb_check = False
        self.divide_path = False


    ## TF Callback: for position and orientation of the robot ##
    def tf_callback(self, msg:TFMessage):
        if (msg.transforms[0].child_frame_id == "odom"):
            self.odom_data = msg.transforms[0]
            self.odom_x = msg.transforms[0].transform.translation.x
            self.odom_y = msg.transforms[0].transform.translation.y
            q_x = msg.transforms[0].transform.rotation.x
            q_y = msg.transforms[0].transform.rotation.y
            q_z = msg.transforms[0].transform.rotation.z
            q_w = msg.transforms[0].transform.rotation.w
            self.odom_yaw = tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]


            self.odom_cb_check = True


        if (msg.transforms[0].child_frame_id == "base_footprint"):
            self.base_data = msg.transforms[0]
            self.base_x = msg.transforms[0].transform.translation.x
            self.base_y = msg.transforms[0].transform.translation.y
            q_x = msg.transforms[0].transform.rotation.x
            q_y = msg.transforms[0].transform.rotation.y
            q_z = msg.transforms[0].transform.rotation.z
            q_w = msg.transforms[0].transform.rotation.w
            self.base_yaw = tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]

            self.base_cb_check = True

    def getPose(self):
        self.wait_for_callback("odom")
        self.wait_for_callback("base") 
        self.x = self.odom_x+self.base_x*np.cos(self.odom_yaw)-self.base_y*np.sin(self.odom_yaw)
        self.y = self.odom_y+self.base_y*np.cos(self.odom_yaw)+self.base_x*np.sin(self.odom_yaw)
        self.yaw = self.base_yaw+self.odom_yaw
        self.pos = [self.x, self.y, 0]
        self.ang = [0, 0, self.yaw]
        self.pose_cb_check = True


    ## Map Callback: for informations about the map ##
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = msg.data
        self.map_cb_check = True


    def scan_callback(self, msg:LaserScan):
        self.scan_ranges = msg.ranges
        self.angle_increment = msg.angle_increment *180 / np.pi
 
        self.scan_cb_check = True
        



    ## So it's possible to execute a callback even if inside a while loop ##
    def wait_for_callback(self, topic):
        if topic == 'map':
            while not self.map_cb_check: 
                rclpy.spin_once(self)
        elif topic == 'base':
            while not self.base_cb_check: 
                rclpy.spin_once(self)
        elif topic == 'odom':
            while not self.odom_cb_check: 
                rclpy.spin_once(self)

        elif topic == 'scan':
            while not self.scan_cb_check: 
                rclpy.spin_once(self)

        self.odom_cb_check = False
        self.base_cb_check = False
        self.map_cb_check = False  
        self.scan_cb_check = False      


    ## From 1D array we create a 2D array with map informations ##
    def create_map_2D(self, map_data, h, w):
        return np.reshape(map_data, (h, w))


    ## Dividing the map in bigger regions with properties 
    ## depending on the pixel informations inside 


    def create_square_map(self):
        self.map2D = self.create_map_2D(self.data, self.height, self.width) 
        self.square_size = SQUARE_SIZE #number of pixels
        self.square_map_h = (int)(self.height / self.square_size)
        self.square_map_w = (int)(self.width / self.square_size)

        # centerH = (int)((self.height - self.square_map_h * self.square_size) / 2)
        # centerW = (int)((self.width - self.square_map_w * self.square_size) / 2)

        idx_matrix = 1*self.resolution*SQUARE_SIZE*np.indices((self.square_map_h, self.square_map_w)) + 0.5*self.resolution*SQUARE_SIZE

        self.square_map_priority = np.zeros((self.square_map_h, self.square_map_w))
        self.square_map_position = np.zeros((self.square_map_h, self.square_map_w, 2))
        self.square_map_distance = np.zeros((self.square_map_h, self.square_map_w))
        self.square_map_unknown = np.zeros((self.square_map_h, self.square_map_w))
        self.square_map_obstacle = np.zeros((self.square_map_h, self.square_map_w))

        for i in range(self.square_map_h):
            for j in range(self.square_map_w):
                square_pixels = self.map2D[i*SQUARE_SIZE: (i+1)*SQUARE_SIZE, j*SQUARE_SIZE: (j+1)*SQUARE_SIZE]

                # position of the center of the square for every square
                self.square_map_position[i,j] = np.array([idx_matrix[1][i,j] + self.originX , idx_matrix[0][i,j] + self.originY])
                # unknown value: value between 0 and 1 depending on how many pixel of the square are uknown over the total number of pixel
                # self.square_map_unknown[i,j] = 1/(SQUARE_SIZE*SQUARE_SIZE) * (np.count_nonzero(square_pixels == -1)) 
                self.square_map_unknown[i,j] = (1 / (SQUARE_SIZE ** 2)) * (np.sum((square_pixels < UNKNOWN_UPPER_THRESHOLD) & (square_pixels >= UNKNOWN_LOWER_THRESHOLD)))
                if self.square_map_unknown[i,j] < UNKNOWN_PERCENTAGE_THRESHOLD: self.square_map_unknown[i,j] = 0.0
                #self.square_map_unknown[i,j] = -4*self.square_map_unknown[i,j]**2 + 4*self.square_map_unknown[i,j]
                
                # if True, square is an obstacle
                self.square_map_obstacle[i, j] = (1 / (SQUARE_SIZE ** 2)) * (np.sum((square_pixels < OBSTACLE_UPPER_THRESHOLD) & (square_pixels >= OBSTACLE_LOWER_THRESHOLD))) >= OBSTACLE_PERCENTAGE_THRESHOLD
                # distance between the square and the robot
                self.square_map_distance[i,j] = np.sqrt((self.pos[0] - self.square_map_position[i,j][0])**2 + (self.pos[1] - self.square_map_position[i,j][1])**2)

                # priority: value between 0 and 1. The more the square is unknwon and closer to the "sweet_distance" the more its priority is high
                self.square_map_priority[i,j] = np.exp(-(DISTANCE_WEIGHT*(SWEET_DISTANCE - self.square_map_distance[i,j]))**2) * self.square_map_unknown[i,j]
                # print(f"[{i},{j}] position = {self.square_map_position[i,j]} \t square_pixels = {square_pixels}")
                # print(f"[{i},{j}] position = {self.square_map_position[i,j]} \t obstacle value = {(1 / (SQUARE_SIZE ** 2)) * (np.sum((square_pixels <= OBSTACLE_UPPER_THRESHOLD) & (square_pixels >= OBSTACLE_LOWER_THRESHOLD))) } \t obstacle = {self.square_map_obstacle[i,j]}")
        
        # wall_map_msg = OccupancyGrid()
    def publish_wall_map(self):

        wall_map_1D = self.square_map_obstacle.reshape(1,self.square_map_obstacle.shape[0] * self.square_map_obstacle.shape[1])
        wall_map_msg = self.map_data
        wall_map_msg.data = (wall_map_1D.squeeze().astype(np.int8) * 100).tolist()
        wall_map_msg.info.resolution = SQUARE_SIZE * self.resolution
        wall_map_msg.info.origin.position.x = self.originX
        wall_map_msg.info.origin.position.y = self.originY
        wall_map_msg.info.width = self.square_map_obstacle.shape[1]
        wall_map_msg.info.height = self.square_map_obstacle.shape[0]
        self.wall_map_publisher.publish(wall_map_msg)





    def check_goal(self,i,first_check):
        goal_vector = np.array([self.goal[0] - self.pos[0], self.goal[1] - self.pos[1]]) 
        self.goal_angle = np.arctan2(goal_vector[1], goal_vector[0]) 
        scan_angle = ((self.goal_angle - self.ang[2]) * 180 / np.pi) % 360           
        goal_distance = np.sqrt( goal_vector[0]**2 + goal_vector[1]**2)
        # i want to avoid that outer square are published
        if self.max_idx[0] == 0 or self.max_idx[0] == self.square_map_w or self.max_idx[1] == 0 or self.max_idx[1] == self.square_map_h:
            return True
        # i want to avoid that square too close or behind a wall are published 
        if first_check:
            if self.scan_ranges[int(scan_angle / self.angle_increment)] <= (goal_distance + 0.5):
                return True

        # i want to avoid that square mapped as obstacle are published 
        if self.square_map_obstacle[self.max_idx[0], self.max_idx[1]]:
            return True                         
        if len(self.goal_history) > 0:
            if self.goal[0] == self.goal_history[0][0] and self.goal[1] == self.goal_history[0][1]:
                return True
        
        print(f"\tRobot to Goal Distance: {goal_distance:.3} Angle: {int(scan_angle) }, Lidar Distance: {self.scan_ranges[int(scan_angle)]:.3}, Robot Position: {self.pos}")
        print(f"\tLevel of unkwnown: {self.square_map_unknown[self.max_idx[0], self.max_idx[1]]}")
        print(f"\tCollision Free Goal, Accept goal: {i}\n")

        self.new_goal_found = True
        return False # A good goal is found
            

    def find_goal(self):
        self.new_goal_found = False
        self.wait_for_callback("scan")

        sorted_indices = np.argsort(self.square_map_priority, axis=None)[::-1]
        sorted_indices_2d = np.array(np.unravel_index(sorted_indices, self.square_map_priority.shape)).T

        for i in range(len(sorted_indices_2d)-1):
            # set goal to max priority square position
            self.max_idx = [sorted_indices_2d[i][0], sorted_indices_2d[i][1]] 
            self.goal = self.square_map_position[self.max_idx[0], self.max_idx[1]]

            if not self.check_goal(i, first_check = True):
                self.goal_history.insert(0, self.goal)
                # print(f"map at goal: {self.goal}, with index: {self.max_idx}: \n", self.map2D[self.max_idx[0]*SQUARE_SIZE: (self.max_idx[0]+1)*SQUARE_SIZE, self.max_idx[1]*SQUARE_SIZE: (self.max_idx[1]+1)*SQUARE_SIZE])
                break
            elif self.check_goal(i, first_check = True) and not self.check_goal(i, first_check = False):
                self.goal_history.insert(0, self.goal)
                # print(f"map at goal: {self.goal}, with index: {self.max_idx}: \n", self.map2D[self.max_idx[0]*SQUARE_SIZE: (self.max_idx[0]+1)*SQUARE_SIZE, self.max_idx[1]*SQUARE_SIZE: (self.max_idx[1]+1)*SQUARE_SIZE])
                break
        
        if not self.new_goal_found:
            print(f"No Goal out of {i} can be Found")  




    def reach_goal(self):
        goal_pose = create_PoseStamped(self.nav, self.goal[0], self.goal[1], self.goal_angle)
        self.nav.goToPose(goal_pose)
        position_check = self.pos
        self.new_goal_found = False
        time_expired = False

        i = 1
        start_time = time.time()
        while not self.nav.isTaskComplete():


            # if self.square_map_obstacle[idx]:
            #     print("Goal canceled: obstacle found")
            #     self.nav.cancelTask()
            #     self.goal_history.pop(0)


            # implementare che se è fermo più di 10 secondi allora spezzettiamo la path ( si puo scoprire il tempo dalla feedback )
            
            nav_time = time.time()
            if nav_time - start_time > MAX_GOAL_TIME:
               
                print("MAX TIME EXPIRED")
                self.nav.cancelTask()
                self.goal_history.pop(0)
                time_expired = True
                break
     
            if nav_time - start_time > CHECK_POSE_TIME * i:
                i += 1
                self.getPose()
                if np.sqrt((self.pos[0] - position_check[0])**2 + (self.pos[1] - position_check[1])**2) < PATH_DIV_THRESH:
                    print("Goal can't be reached... trying dividing path....")
                    self.divide_path = True
                    self.nav.cancelTask()
                position_check = self.pos



        if self.divide_path:

            start_pose = create_PoseStamped(self.nav, self.pos[0], self.pos[1], self.ang[2])
            path = self.nav.getPath(start_pose, goal_pose)
            step_length = STEP_LENGTH
            while len(path.poses) < step_length:
                step_length -= 1
            intermediate_poses = path.poses[step_length :: step_length]
            intermediate_poses.append(goal_pose)

            start_time1 = time.time()
            for i,pose in enumerate(intermediate_poses):

                if time_expired: 
                    time_expired = False
                    break

                nav_time1 = time.time()
                if nav_time1 - start_time1 > MAX_GOAL_TIME * 2:
                    print("MAX TIME EXPIRED")
                    break

                self.getPose()
                xx = np.round(pose.pose.position.x, 3)
                yy = np.round(pose.pose.position.y, 3)
                goal_vector = np.array([xx - self.pos[0], yy - self.pos[1]]) 
                goal_angle = np.arctan2(goal_vector[1], goal_vector[0]) 

                self.nav.goToPose(create_PoseStamped(self.nav, xx, yy, goal_angle))
                print(f"reaching intermediate pose {i}/{len(intermediate_poses)}: [{pose.pose.position.x:.2}, {pose.pose.position.y:.2}] of goal: [{self.goal[0]:.2}, {self.goal[1]:.2}]")

                start_time2 = time.time()
                while not self.nav.isTaskComplete():
                    nav_time2 = time.time()
                    if nav_time2 - start_time2 > MAX_GOAL_TIME / 2:
                        print("MAX TIME EXPIRED")
                        self.goal_history.pop(0)
                        self.nav.cancelTask()
                        time_expired = True
                        break
            self.divide_path = False

        if time_expired: 
            time_expired = False
            self.backtracking()   

    def backtracking(self):


        print("BACKTRACKING")
        self.getPose()
        self.wait_for_callback("scan")
        # search for closer previous goal
        min_dist = 100000000
        min_idx = 0
        closer_goal = [0.0, 0.0]
        for i, goal in enumerate(self.goal_history):

            goal_vector = np.array([goal[0] - self.pos[0], goal[1] - self.pos[1]]) 
            goal_angle = np.arctan2(goal_vector[1], goal_vector[0]) 
            scan_angle = ((goal_angle - self.ang[2]) * 180 / np.pi) % 360           
            goal_distance = np.sqrt( goal_vector[0]**2 + goal_vector[1]**2)

            # print(f"min dist = {min_dist}, robot position = {self.pos[0:1]}, goal = {goal}, goal distance = {goal_distance} ")


            if not (self.scan_ranges[int(scan_angle)] <= (goal_distance)) and goal_distance < min_dist:
                # print("goal is lower than min_dist")
                min_dist = goal_distance
                min_idx = i
                closer_goal = goal

        self.goal_history = self.goal_history[min_idx:]
        self.goal = closer_goal
        self.reach_goal()







        # step = BACKTRACKING_STEP
        # #we set the previous goal as present goal
        # while step > len(self.goal_history):
        #     step -= 1
        # self.goal = self.goal_history[step - 1]
        # self.goal_history = self.goal_history[step - 1:]
        # self.reach_goal()




        





     
    def start_exploration(self):
        # Main exploration function
        self.get_logger().info("\n\n\t\t######## STARTING EXPLORATION ########")
        self.nav = BasicNavigator()




        while rclpy.ok() and not self.map_completed:
            self.get_logger().info("\n\n\t\t## Finding Goal ##")          
            self.wait_for_callback("map")
            self.getPose()
            self.create_square_map()
            self.publish_wall_map()
            self.find_goal()
            # print("goal_history = ", np.array(self.goal_history))
            if self.new_goal_found:
                self.get_logger().info(f"Goal: [{self.goal[0]:.2},{self.goal[1]:.2}]")
                self.reach_goal()
            else:
                # if goal is not found we backtrack through previous goals
                # first we cancel the goal in which we are blocked
                self.goal_history.pop(0)
                self.backtracking()


            # self.get_logger().info(f"Goal Reched")


def main(args=None):
 rclpy.init(args=args)
 explorer = Exploration()
 explorer.start_exploration()
 explorer.destroy_node()
 rclpy.shutdown()


     
if __name__ == "__main__":
 main()