import math
import numpy as np
from nav_msgs.msg import OccupancyGrid , Odometry
import time

# Sanitizer parameters
LAMP_POWER = 100e-6
ENERGY_TARGET = 10e-3
ROBOT_RADIUS = 0.1
DELTA_T = 0.5

SQUARE_SIZE = 4
OBSTACLE_LOWER_THRESHOLD = 60
OBSTACLE_UPPER_THRESHOLD = 100
OBSTACLE_PERCENTAGE_THRESHOLD = 0.1/SQUARE_SIZE



import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
import os
import tf_transformations
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool

class Sanitizer(Node):

    def __init__(self, map_path):
        super().__init__('Energy_Updater')
        self.map_cli = self.create_client(LoadMap, 'map_server/load_map')
        self.get_logger().info(f'waiting for map_load service')
        while not self.map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.map_req = LoadMap.Request()
        self.house_map = self.send_map_request(map_path=map_path)
        self.get_logger().info(f'Map Loaded Correctly')
        self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.energy_map_pub = self.create_publisher(OccupancyGrid, 'energy_map', 10)
        self.visibility_map_pub = self.create_publisher(OccupancyGrid, 'visibility_map', 10)
        self.wall_map_publisher = self.create_publisher(OccupancyGrid, '/wall_map', 10)
        self.distance_map_pub = self.create_publisher(OccupancyGrid, '/distance_map', 10)
        self.lamp_sub = self.create_subscription(Bool, '/lamp',self.lamp_callback, 10)
        self.odom_cb_check = False
        self.base_cb_check = False
        self.pose_cb_check = False
        self.map_generated_check= False 
        self.lamp_on = False
        while not self.pose_cb_check:
            rclpy.spin_once(self)
        self.map_generation(self.house_map)
        self.energy_timer = self.create_timer(DELTA_T ,self.update_energy_map)

    def lamp_callback(self, msg: Bool):
        self.lamp_on = msg.data


    def send_map_request(self, map_path):
        # send_request function for self.map_cli service client
        self.map_req.map_url = map_path
        self.map_future = self.map_cli.call_async(self.map_req)
        rclpy.spin_until_future_complete(self, self.map_future)
        response = self.map_future.result().map
        return response

    def map_generation(self, msg:OccupancyGrid):
        self.map_data = msg
        resolution = msg.info.resolution
        originX = msg.info.origin.position.x
        originY = msg.info.origin.position.y
        x_width = msg.info.width
        y_height = msg.info.height
        data = msg.data
        map_2D = np.reshape(data, (y_height, x_width))
        self.create_square_map(map_2D, originX, originY, resolution)
              
    def create_square_map(self, map2D, originX, originY, resolution):
        y_height, x_width = map2D.shape
        y_square_map_h = (int)(y_height / SQUARE_SIZE)
        center_y = (-y_square_map_h*SQUARE_SIZE + y_height)/2 
        x_square_map_w = (int)(x_width / SQUARE_SIZE)
        center_x = (-x_square_map_w*SQUARE_SIZE + x_width)/2 
        idx_matrix = resolution*SQUARE_SIZE*np.indices((y_square_map_h, x_square_map_w )) 

        self.square_map_position = np.zeros((y_square_map_h, x_square_map_w, 2))
        self.square_map_distance = np.zeros((y_square_map_h, x_square_map_w))
        self.square_map_obstacle = np.zeros((y_square_map_h, x_square_map_w))
        self.square_map_obstacle_in_between = np.zeros((y_square_map_h, x_square_map_w))
        self.square_map_obstacle_in_between_color = np.zeros((y_square_map_h, x_square_map_w))
        self.square_map_energy= np.zeros((y_square_map_h, x_square_map_w))
        self.square_map_energy_color = np.zeros((y_square_map_h, x_square_map_w)) -127

        self.square_map_position[:,:,0] = idx_matrix[1] + originX +  0.5*resolution*SQUARE_SIZE + center_x * resolution
        self.square_map_position[:,:,1] = idx_matrix[0] + originY +  0.5*resolution*SQUARE_SIZE + center_y * resolution
        self.square_map_distance[:,:] = np.sqrt((self.pos[0] - self.square_map_position[:,:,0])**2 + (self.pos[1] - self.square_map_position[:,:,1])**2)

        for i in range (y_square_map_h):
            for j in range(x_square_map_w):
                square_pixels = map2D[int(center_y) +i*SQUARE_SIZE: int(center_y)+(i+1)*SQUARE_SIZE, int(center_x)+j*SQUARE_SIZE: int(center_x)+(j+1)*SQUARE_SIZE]
                #square_pixels = map2D[ i*SQUARE_SIZE:(i+1)*SQUARE_SIZE, j*SQUARE_SIZE: (j+1)*SQUARE_SIZE]
                self.square_map_obstacle[i, j] = (np.sum(square_pixels >= OBSTACLE_LOWER_THRESHOLD) / square_pixels.size) >= OBSTACLE_PERCENTAGE_THRESHOLD 
                if self.square_map_obstacle[i,j]: self.square_map_energy_color[i,j] = 100
        self.publish_wall_map(originX, originY, resolution, center_x, center_y)
           
    def publish_wall_map(self,originX, originY, resolution, center_x , center_y):
        self.map_originX = originX + center_x * resolution
        self.map_originY = originY + center_y * resolution
        self.map_resolution = SQUARE_SIZE * resolution

        wall_map_1D = self.square_map_obstacle.reshape(1,self.square_map_obstacle.shape[0] * self.square_map_obstacle.shape[1])
        wall_map_msg = self.map_data
        wall_map_msg.data = (wall_map_1D.squeeze().astype(np.int8) * 100).tolist()
        wall_map_msg.info.resolution = self.map_resolution
        wall_map_msg.info.origin.position.x = self.map_originX
        wall_map_msg.info.origin.position.y = self.map_originY
        wall_map_msg.info.width = self.square_map_obstacle.shape[1]
        wall_map_msg.info.height = self.square_map_obstacle.shape[0]
        self.wall_map_publisher.publish(wall_map_msg)
        self.map_generated_check = True


    def publish_energy_map(self):
        energy_map_1D = self.square_map_energy_color.reshape(1,self.square_map_energy_color.shape[0] * self.square_map_energy_color.shape[1])
        energy_map_msg = self.map_data
        energy_map_msg.data = (energy_map_1D.squeeze().astype(np.int8)).tolist()
        energy_map_msg.info.resolution = self.map_resolution
        energy_map_msg.info.origin.position.x = self.map_originX
        energy_map_msg.info.origin.position.y = self.map_originY
        energy_map_msg.info.width = self.square_map_energy_color.shape[1]
        energy_map_msg.info.height = self.square_map_energy_color.shape[0]
        self.energy_map_pub.publish(energy_map_msg)

    def publish_visibility_map(self):
        visibility_map_1D = self.square_map_obstacle_in_between_color.reshape(1,self.square_map_obstacle_in_between_color.shape[0] * self.square_map_obstacle_in_between_color.shape[1])
        visibility_map_msg = self.map_data
        visibility_map_msg.data = (visibility_map_1D.squeeze().astype(np.int8)).tolist()
        visibility_map_msg.info.resolution = self.map_resolution
        visibility_map_msg.info.origin.position.x = self.map_originX
        visibility_map_msg.info.origin.position.y = self.map_originY
        visibility_map_msg.info.width = self.square_map_obstacle_in_between_color.shape[1]
        visibility_map_msg.info.height = self.square_map_obstacle_in_between_color.shape[0]
        self.visibility_map_pub.publish(visibility_map_msg)

    def publish_ditance_map(self):
        distance_map_1D = self.square_map_distance.reshape(1,self.square_map_distance.shape[0] * self.square_map_distance.shape[1])
        distance_map_msg = self.map_data
        distance_map_msg.data = (distance_map_1D.squeeze().astype(np.int8)).tolist()
        distance_map_msg.info.resolution = self.map_resolution
        distance_map_msg.info.width = self.square_map_distance.shape[1]
        distance_map_msg.info.height = self.square_map_distance.shape[0]
        self.distance_map_pub.publish(distance_map_msg)



    def tf_callback(self, msg:TFMessage):
        
        if (msg.transforms[0].child_frame_id == "odom"):
            self.odom_x = msg.transforms[0].transform.translation.x
            self.odom_y = msg.transforms[0].transform.translation.y
            q_x = msg.transforms[0].transform.rotation.x
            q_y = msg.transforms[0].transform.rotation.y
            q_z = msg.transforms[0].transform.rotation.z
            q_w = msg.transforms[0].transform.rotation.w
            self.odom_yaw = tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
            self.odom_cb_check = True

        if (msg.transforms[0].child_frame_id == "base_footprint"):
            self.base_x = msg.transforms[0].transform.translation.x
            self.base_y = msg.transforms[0].transform.translation.y
            q_x = msg.transforms[0].transform.rotation.x
            q_y = msg.transforms[0].transform.rotation.y
            q_z = msg.transforms[0].transform.rotation.z
            q_w = msg.transforms[0].transform.rotation.w
            self.base_yaw = tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
            self.base_cb_check = True
        
        if self.base_cb_check  and self.odom_cb_check :
            x = self.odom_x + self.base_x * np.cos(self.odom_yaw) - self.base_y* np.sin(self.odom_yaw)
            y = self.odom_y + self.base_y * np.cos( self.odom_yaw) + self.base_x* np.sin(self.odom_yaw)
            yaw = self.base_yaw + self.odom_yaw
            self.pos = [x, y, 0]
            self.ang = [0, 0, yaw]
            self.pose_cb_check = True
            if self.map_generated_check :
                min_index = np.argmin(self.square_map_distance)
                row, column = np.unravel_index(min_index, self.square_map_distance.shape)
                self.robot_cell_position = [row , column]
                

    def update_energy_map(self):
        self.check_obstacle_in_between()

        y_square_map_h, x_square_map_w = self.square_map_energy.shape
        self.square_map_distance[:,:] = self.square_map_distance[:,:] = np.sqrt((self.pos[0] - self.square_map_position[:,:,0])**2 + (self.pos[1] - self.square_map_position[:,:,1])**2)
        if self.lamp_on:  
            for i in range (y_square_map_h):
                for j in range(x_square_map_w):
                    #self.check_obstacle_in_between(i,j, self.robot_cell_position[0], self.robot_cell_position[1])
                    if self.square_map_distance[i,j] > ROBOT_RADIUS and (self.square_map_obstacle_in_between[i,j] == False):
                        self.square_map_energy[i, j] += LAMP_POWER*DELTA_T/((self.square_map_distance[i, j])**2 )  
                        self.square_map_energy_color[i,j] = (int) (self.square_map_energy[i,j]/ENERGY_TARGET*255-127 )
                        if self.square_map_energy_color[i,j] >= -1 and self.square_map_energy_color[i,j]<= 100: self.square_map_energy_color[i,j] = -2
                        if self.square_map_energy_color[i,j] > 255-128: self.square_map_energy_color[i,j] = 255-128
                    
        self.publish_energy_map()
        self.publish_visibility_map()
        self.publish_ditance_map()

    


    def check_obstacle_in_between(self):
        rows, cols = len(self.square_map_obstacle), len(self.square_map_obstacle[0])
        robot_row, robot_col = self.robot_cell_position

        obstacle_matrix = np.zeros((rows, cols), dtype=bool)

        for row in range(rows):
            for col in range(cols):
                if self.square_map_obstacle[row][col]:
                    # If the cell is an obstacle, mark it
                    obstacle_matrix[row][col] = True
                else:
                    # If the cell is not an obstacle, check if there is a line of sight from the robot
                    line_points = self.bresenham_line(robot_col, robot_row, col, row)
                    if any(self.square_map_obstacle[y][x] for x, y in line_points[1:]):
                        obstacle_matrix[row][col] = True
                if obstacle_matrix [row][col] : self.square_map_obstacle_in_between_color[row, col] = 0
                if self.square_map_obstacle[row, col] : self.square_map_obstacle_in_between_color[row, col] = 100
                if not obstacle_matrix [row][col]: self.square_map_obstacle_in_between_color[row, col] = 101
                #if self.square_map_distance[row, col] < ROBOT_RADIUS and (not self.square_map_obstacle[row,col]): self.square_map_obstacle_in_between_color[row, col] = 0

        self.square_map_obstacle_in_between = obstacle_matrix


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
    
            

    

def main(args=None):
    
    map_path = os.path.abspath('src/sanification/files/big_house_map.yaml')
    rclpy.init(args=args)
    sanitizer = Sanitizer(map_path)
    rclpy.spin(sanitizer)
    sanitizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
