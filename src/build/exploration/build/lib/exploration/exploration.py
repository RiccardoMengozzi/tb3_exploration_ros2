import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf_transformations
import numpy as np
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


SQUARE_SIZE = 5
SWEET_DISTANCE = 0.8
DISTANCE_WEIGHT = 1
DISTANCE_THRESHOLD = 0.1

OBSTACLE_LOWER_THRESHOLD = 80
OBSTACLE_UPPER_THRESHOLD = 100
OBSTACLE_PERCENTAGE_THRESHOLD = 0.1 / SQUARE_SIZE

UNKNOWN_LOWER_THRESHOLD = 10
UNKNOWN_UPPER_THRESHOLD = 70
UNKNOWN_PERCENTAGE_THRESHOLD = 0.4

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


class Exploration(Node):
 


    ## Node Initialization ##
    def __init__(self) -> None:
        super().__init__('exploration')

        callback_group = MutuallyExclusiveCallbackGroup()
        main_execution_group = MutuallyExclusiveCallbackGroup()
        self.nav = BasicNavigator() 

        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10, callback_group=callback_group)
        self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10, callback_group=callback_group)
        self.wall_map_publisher = self.create_publisher(OccupancyGrid, '/wall_map', 10, callback_group=callback_group)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10, callback_group=callback_group)
        self.common_points_pub = self.create_publisher(MarkerArray, '/common_points', 10, callback_group=callback_group)

        self.odom_cb_check = False
        self.base_cb_check = False
        self.pose_cb_check = False
        self.map_cb_check = False
        self.map_generated_check = False
        self.costmap_cb_check = False

        

        self.goal_history = []

        self.call_timer = self.create_timer(1, self.explore, callback_group=main_execution_group)

    def explore(self):

        # Main exploration function
        if self.map_cb_check:
            self.get_logger().info("\n\n\t ## Finding Goal ##")
            self.create_square_map()
            self.publish_wall_map()
            if self.costmap_cb_check:
                self.find_goal()
                # print("GOAL HISTORY = ", self.goal_history)
                if self.new_goal_found:
                    self.get_logger().info(f"Goal: [{self.goal[0]:.2},{self.goal[1]:.2}]")
                    self.reach_goal()


    ## TF Callback: for position and orientation of the robot ##
    def tf_callback(self, msg:TFMessage):
        # print("tf")
        if (msg.transforms[0].child_frame_id == "odom"):
            # print("odom")
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
            # print("base")

            self.base_data = msg.transforms[0]
            self.base_x = msg.transforms[0].transform.translation.x
            self.base_y = msg.transforms[0].transform.translation.y
            q_x = msg.transforms[0].transform.rotation.x
            q_y = msg.transforms[0].transform.rotation.y
            q_z = msg.transforms[0].transform.rotation.z
            q_w = msg.transforms[0].transform.rotation.w
            self.base_yaw = tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
            self.base_cb_check = True

        if self.base_cb_check and self.odom_cb_check:
            self.base_cb_check = False
            self.odom_cb_check = False

            self.x = self.odom_x+self.base_x*np.cos(self.odom_yaw)-self.base_y*np.sin(self.odom_yaw)
            self.y = self.odom_y+self.base_y*np.cos(self.odom_yaw)+self.base_x*np.sin(self.odom_yaw)
            self.yaw = self.base_yaw+self.odom_yaw
            self.pos = [self.x, self.y, 0]
            self.ang = [0, 0, self.yaw]
            self.pose_cb_check = True



    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap_data = msg.data
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin_x = msg.info.origin.position.x
        self.costmap_origin_y = msg.info.origin.position.y
        self.costmap_cb_check = True


    ## Map Callback: for informations about the map ##
    def map_callback(self, msg: OccupancyGrid):
        # print("map")
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = msg.data

        self.map_cb_check = True

    def position_to_index(self, position):
        """
        Convert a position (x, y) to its index in a 2D matrix.

        Args:
        - matrix: The 2D matrix
        - position: The position (x, y)

        Returns:
        - index: The index of the position in the matrix
        """

        x, y = position
        j = (int)((x - self.originX) / (SQUARE_SIZE * self.resolution))
        i = (int)((y - self.originY) / (SQUARE_SIZE * self.resolution))
        # Convert coordinates to index
        index = (i,j)

        return index



    def create_map_2D(self, map_data, h, w):
        return np.reshape(map_data, (h, w))


    def create_square_map(self):
        # print("creating")
        self.map2D = self.create_map_2D(self.data, self.height, self.width) 
        self.square_size = SQUARE_SIZE #number of pixels
        self.square_map_h = (int)(self.height / self.square_size)
        self.square_map_w = (int)(self.width / self.square_size)

        self.square_map_priority = np.zeros((self.square_map_h, self.square_map_w))
        self.square_map_position = np.zeros((self.square_map_h, self.square_map_w, 2))
        self.square_map_distance = np.zeros((self.square_map_h, self.square_map_w))
        self.square_map_unknown = np.zeros((self.square_map_h, self.square_map_w))
        self.square_map_obstacle = np.zeros((self.square_map_h, self.square_map_w))




        idx_matrix = 1*self.resolution*SQUARE_SIZE*np.indices((self.square_map_h, self.square_map_w)) + 0.5*self.resolution*SQUARE_SIZE

        for i in range(self.square_map_h):
            for j in range(self.square_map_w):
                square_pixels = self.map2D[i*SQUARE_SIZE: (i+1)*SQUARE_SIZE, j*SQUARE_SIZE: (j+1)*SQUARE_SIZE]

                self.square_map_position[i,j] = np.array([idx_matrix[1][i,j] + self.originX , idx_matrix[0][i,j] + self.originY])
                
                self.square_map_unknown[i,j] = (1 / (SQUARE_SIZE ** 2)) * (np.sum((square_pixels < UNKNOWN_UPPER_THRESHOLD) & (square_pixels >= UNKNOWN_LOWER_THRESHOLD)))
                if self.square_map_unknown[i,j] < UNKNOWN_PERCENTAGE_THRESHOLD: self.square_map_unknown[i,j] = 0.0
                
                self.square_map_obstacle[i, j] = (1 / (SQUARE_SIZE ** 2)) * (np.sum((square_pixels < OBSTACLE_UPPER_THRESHOLD) & (square_pixels >= OBSTACLE_LOWER_THRESHOLD))) >= OBSTACLE_PERCENTAGE_THRESHOLD
                
                self.square_map_distance[i,j] = np.sqrt((self.pos[0] - self.square_map_position[i,j][0])**2 + (self.pos[1] - self.square_map_position[i,j][1])**2)

                self.square_map_priority[i,j] = np.exp(-(DISTANCE_WEIGHT*(SWEET_DISTANCE - self.square_map_distance[i,j]))**2) * self.square_map_unknown[i,j]
                # print(f"[{i},{j}] position = {self.square_map_position[i,j]} \t square_pixels = {square_pixels}")
                # print(f"[{i},{j}] position = {self.square_map_position[i,j]} \t obstacle value = {(1 / (SQUARE_SIZE ** 2)) * (np.sum((square_pixels <= OBSTACLE_UPPER_THRESHOLD) & (square_pixels >= OBSTACLE_LOWER_THRESHOLD))) } \t obstacle = {self.square_map_obstacle[i,j]}")
                
                min_index = np.argmin(self.square_map_distance)
                row, column = np.unravel_index(min_index, self.square_map_distance.shape)
                self.robot_idxs = [row , column]

    def publish_wall_map(self):
        # print("publishing")

        wall_map_1D = self.square_map_obstacle.reshape(1,self.square_map_obstacle.shape[0] * self.square_map_obstacle.shape[1])
        wall_map_msg = self.map_data
        wall_map_msg.data = (wall_map_1D.squeeze().astype(np.int8) * 100).tolist()
        wall_map_msg.info.resolution = SQUARE_SIZE * self.resolution
        wall_map_msg.info.origin.position.x = self.originX
        wall_map_msg.info.origin.position.y = self.originY
        wall_map_msg.info.width = self.square_map_obstacle.shape[1]
        wall_map_msg.info.height = self.square_map_obstacle.shape[0]
        self.wall_map_publisher.publish(wall_map_msg)
        self.map_generated_check = True



    def publish_marker_array(self, common_points:list, goal):
        # Create a MarkerArray message
        marker_array_msg = MarkerArray()
        common_points.insert(0, goal)
        for i,point in enumerate(common_points):
            # Example Marker message
            marker_msg = Marker()
            marker_msg.header.frame_id = "map"
            marker_msg.header.stamp = self.get_clock().now().to_msg()
            marker_msg.ns = "common_point_" + str(i) 
            marker_msg.id = i
            marker_msg.type = Marker.SPHERE
            marker_msg.action = Marker.ADD
            marker_msg.pose.position.x = point[0]
            marker_msg.pose.position.y = point[1]
            marker_msg.pose.position.z = 0.0
            marker_msg.scale.x = 0.1
            marker_msg.scale.y = 0.1
            marker_msg.scale.z = 0.1
            if i == 0:
                marker_msg.color.r = 1.0
                marker_msg.color.g = 0.0
                marker_msg.color.b = 0.0
            else:
                marker_msg.color.r = 0.0
                marker_msg.color.g = 0.0
                marker_msg.color.b = 1.0
            marker_msg.color.a = 1.0
            marker_msg.lifetime = Duration(sec=5)
            # Add the Marker message to the MarkerArray
            marker_array_msg.markers.append(marker_msg)

        # Publish the MarkerArray message
        self.common_points_pub.publish(marker_array_msg)



    def create_visibility_map(self, idxs):
        rows, cols = self.square_map_obstacle.shape

        visibility_map = np.ones((rows, cols), dtype=bool)

        for row in range(rows):
            for col in range(cols):
                if self.square_map_obstacle[row][col]:
                    # If the cell is an obstacle, mark it
                    visibility_map[row][col] = False
                else:
                    # If the cell is not an obstacle, check if there is a line of sight from the robot
                    line_points = self.bresenham_line(idxs[1], idxs[0], col, row)
                    if any(self.square_map_obstacle[y][x] for x, y in line_points[1:]):
                        visibility_map[row][col] = False

        return visibility_map




    def check_obstacle_in_between(self, start_idxs, goal_idxs):

        start_row, start_col = start_idxs[0], start_idxs[1]
        end_row, end_col = goal_idxs[0], goal_idxs[1]

        # Use Bresenham's line algorithm to get all points between start and end positions
        line_points = self.bresenham_line(start_col, start_row, end_col, end_row)

        # Check if any point on the line has an obstacle
        for col, row in line_points[1:]:
            if self.square_map_obstacle[row][col]:
                return True  # Obstacle found
        return False  # No obstacles found

    def bresenham_line(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        points = []

        while True:
            points.append((x0, y0))

            if x0 == x1 and y0 == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

    def find_closest_point(self, points, goal_point):
        closest_point = None
        min_distance = float('inf')

        for point in points:
            distance = np.sqrt((point[0] - goal_point[0])**2 + (point[1] - goal_point[1])**2)
            if distance < min_distance and distance > 0.01:
                min_distance = distance
                closest_point = point

        return closest_point

    def find_goal(self):
        self.new_goal_found = False

        sorted_indices = np.argsort(self.square_map_priority, axis=None)[::-1]
        sorted_indices_2d = np.array(np.unravel_index(sorted_indices, self.square_map_priority.shape)).T

        for i in range(len(sorted_indices_2d)-1):
            # set goal to max priority square position
            self.goal_idxs = [sorted_indices_2d[i][0], sorted_indices_2d[i][1]] 
            self.goal = self.square_map_position[self.goal_idxs[0], self.goal_idxs[1]]

            if not self.check_goal(i):
                self.goal_history.insert(0, self.goal)
                # print(f"map at goal: {self.goal}, with index: {self.goal_idxs}: \n", self.map2D[self.goal_idxs[0]*SQUARE_SIZE: (self.goal_idxs[0]+1)*SQUARE_SIZE, self.goal_idxs[1]*SQUARE_SIZE: (self.goal_idxs[1]+1)*SQUARE_SIZE])
                break

        if not self.new_goal_found:
            expired = False
            print(f"No Goal out of {i} can be Found")  
            print(self.square_map_position[self.robot_idxs[0], self.robot_idxs[1]])
            robot_visibility_map = self.create_visibility_map(self.robot_idxs)
            common_points = []
            for idxs in sorted_indices_2d:
                if not self.square_map_obstacle[idxs[0], idxs[1]]:
                    if not self.getCostXY(self.square_map_position[idxs[0], idxs[1]]) > 50:
                        if self.square_map_priority[idxs[0], idxs[1]] > 0:
                            # set goal to max priority square position
                            self.goal_idxs = [idxs[0], idxs[1]] 
                            self.goal = self.square_map_position[idxs[0], idxs[1]]
                            goal_visibility_map = self.create_visibility_map(self.goal_idxs)
                            common_squares = np.logical_and(robot_visibility_map, goal_visibility_map)

                            # vis = np.where(robot_visibility_map)
                            # vis_points = []
                            # for i, j in zip(*vis):
                            #     vis_points.append(self.square_map_position[i,j])
                            # self.publish_marker_array(vis_points, self.goal)

                            true_indices = np.where(common_squares)
                            cm = []
                            # Iterate through True values
                            for i, j in zip(*true_indices):
                                if not self.getCostXY(self.square_map_position[i, j]) > 50 and not self.square_map_obstacle[i,j]:
                                    # print("GOAL = ", self.square_map_position[self.goal_idxs[0], self.goal_idxs[1]])
                                    # print("COMMON POINT = ", self.square_map_position[i, j])
                                    cm.append(self.square_map_position[i,j])
                            if len(cm) > 0:
                                common_points = cm
                                self.publish_marker_array(cm, self.goal)
                                break
            if len(common_points) > 0:
                mid_goal = self.find_closest_point(common_points, self.goal)
                # print("mid_goal = ", mid_goal)
            
                goal_pose = create_PoseStamped(self.nav, self.goal[0], self.goal[1], 0)
                mid_pose = create_PoseStamped(self.nav, mid_goal[0], mid_goal[1],0)
                poses = []
                poses.append(mid_pose)
                poses.append(goal_pose)
                self.goal_history.insert(0, mid_goal)
                self.goal_history.insert(0, self.goal)
                start_time = time.time()
                self.nav.goThroughPoses(poses)
                while not self.nav.isTaskComplete():
                    nav_time = time.time()
                    if nav_time - start_time > MAX_GOAL_TIME / 2:
                        print("MAX TIME EXPIRED")
                        expired = True
                        self.nav.cancelTask()
                        break
                if expired:
                    expired = False
                    while len(poses) > 0:
                        pose = poses.pop(0)
                        self.nav.goToPose(pose)
                        start_time = time.time()
                        while not self.nav.isTaskComplete():
                            nav_time = time.time()
                            if nav_time - start_time > MAX_GOAL_TIME:
                                print("MAX TIME EXPIRED")
                                self.nav.cancelTask()
                                break
                self.goal_history.pop(0)
                self.goal_history.pop(0)
            else:
                self.backtracking()


    def backtracking(self):
        self.goal_history.pop(0)
        found = False
        print("BACKTRACKING")
        # search for closer previous goal
        min_dist = float("inf")
        min_idx = 0
        closer_goal = [0.0, 0.0]
        for i, goal in enumerate(self.goal_history):

            goal_vector = np.array([goal[0] - self.pos[0], goal[1] - self.pos[1]])         
            goal_distance = np.sqrt( goal_vector[0]**2 + goal_vector[1]**2)
            robot_idxs = self.position_to_index(self.pos[0:2])
            goal_idxs = self.position_to_index(goal)
            # print("pos = ", self.pos[0:2])
            # print(np.array(self.goal_history))
            # print(f"history length = {len(self.goal_history)}")
            # print(f"robot position = {self.pos[0:1]}, goal = {goal}, robot_idx = {robot_idxs}, goal_idx = {goal_idxs}")


            if not self.check_obstacle_in_between(robot_idxs, goal_idxs) and goal_distance < min_dist:
                # print(f"goal = {self.goal}, goal_idx = {self.goal_idxs}, goal_distance = {goal_distance}, min_dist = {min_dist}")
                found = True
                min_dist = goal_distance
                min_idx = i
                closer_goal = goal

        if found:
            found = False
            self.goal_history = self.goal_history[min_idx:]
            self.goal = closer_goal
            print("FOUND CLOSEST GOAL: ", self.goal)
            self.reach_goal()
        else:
            self.goal = self.goal_history.pop(0)
            print("NOT FOUND CLOSEST GOAL, GOING TO THE PREVIOUS ONE: ", self.goal)
            self.reach_goal()



    def check_goal(self,i):

            if self.getCostXY(self.square_map_position[self.goal_idxs[0], self.goal_idxs[1]]) > 50:
                return True
            # i want to avoid that outer square are published
            if self.goal_idxs[0] == 0 or self.goal_idxs[0] == self.square_map_w or self.goal_idxs[1] == 0 or self.goal_idxs[1] == self.square_map_h:
                return True

            # i want to avoid that square too close or behind a wall are published 
            if self.check_obstacle_in_between(self.robot_idxs, self.goal_idxs):
                # print(f"Goal:{i} unacceptable, obstacle in between")
                return True
                    
            if len(self.goal_history) > 0:
                if self.goal[0] == self.goal_history[0][0] and self.goal[1] == self.goal_history[0][1]:
                    return True
                
            if self.square_map_priority[self.goal_idxs[0], self.goal_idxs[1]] == 0:
                return True
            
            print(f"\tCollision Free Goal, Accept goal: {i}\n")

            self.new_goal_found = True
            return False # A good goal is found
    

    def getCostXY(self, position):
    # Calculate the index of the position in the costmap
        map_index_x = int((position[0] - self.costmap_origin_x) / self.costmap_resolution)
        map_index_y = int((position[1] - self.costmap_origin_y) / self.costmap_resolution)
        
        # Check if the index is within the bounds of the costmap
        if 0 <= map_index_x < self.costmap_width and 0 <= map_index_y < self.costmap_height:
            # Calculate the index in the flattened costmap array
            costmap_index = map_index_y * self.costmap_width + map_index_x
            
            # Get the cost value at the position
            cost = self.costmap_data[costmap_index]
            return cost
        else:
            print("INDEX OF COSTMAP OUT OF RANGE")

    def reach_goal(self):

            goal_vector = np.array([self.goal[0] - self.pos[0], self.goal[1] - self.pos[1]]) 
            self.goal_angle = np.arctan2(goal_vector[1], goal_vector[0]) 
            goal_pose = create_PoseStamped(self.nav, self.goal[0], self.goal[1], self.goal_angle)
            self.nav.goToPose(goal_pose)
            expired = False
            self.new_goal_found = False
            i = 1
            start_time = time.time()
            while not self.nav.isTaskComplete():
                nav_time = time.time()
                if nav_time - start_time > MAX_GOAL_TIME:
                
                    print("MAX TIME EXPIRED")
                    self.nav.cancelTask()
                    expired = True
                    break
            if expired:
                expired = False
                self.backtracking()
                # if nav_time - start_time > CHECK_POSE_TIME * i:
                #     i += 1
                #     self.getPose()
                #     if np.sqrt((self.pos[0] - position_check[0])**2 + (self.pos[1] - position_check[1])**2) < PATH_DIV_THRESH:
                #         print("Goal can't be reached... trying dividing path....")
                #         self.divide_path = True
                #         self.nav.cancelTask()
                #     position_check = self.pos



def main(args=None):
    rclpy.init(args=args)
    explorer = Exploration()
    executor = MultiThreadedExecutor()
    executor.add_node(explorer)
    executor.spin()
    explorer.destroy_node()
    rclpy.shutdown()


     
if __name__ == "__main__":
 main()