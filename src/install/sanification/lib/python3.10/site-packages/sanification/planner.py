


import sys
import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import os
from std_msgs.msg import Bool
from geometry_msgs.msg import  PoseStamped
import tf_transformations


room_coordinates = np.array([   [[-6.7, 5.6],   [-4.75,1.53]],
                                [[-4.15, 5.5],  [0.31, 0.5]],
                                [[0.76, 5.51],  [2.64, 1.54]],
                                [[3.13, 5.45],  [7.86, 0.44]],
                                [[-6.75, 1.2],  [-4.85, -3.26]],
                                [[-4.39, 0.1],  [5.27, -3.31]],
                                [[5.67, 0.2],   [7.96, -4.57]],
                                [[-6.65, -3.65],[5.25 , -7.25]],
                                [[5.25, -4.9],   [7.90, -7.25]]])



WALL_DISTANCE = 0.2


class Planner(Node):

    def __init__(self, map_path, file_path):
        super().__init__('Planner')
        self.file_path= file_path
        self.map_cli = self.create_client(LoadMap, 'map_server/load_map')
        self.room_map_publisher = self.create_publisher(OccupancyGrid, '/room_map', 10)
        self.nav_on_pub = self.create_publisher(Bool, '/navigate',10)
        self.lamp_on_pub = self.create_publisher(Bool, '/lamp',10)
        lamp_on = Bool()
        lamp_on.data = False
        self.lamp_on_pub.publish(lamp_on)
        nav_on = Bool()
        nav_on.data = False
        self.nav_on_pub.publish(nav_on)
        self.amcl_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.robot_pose_check = False
        self.navigator = BasicNavigator()
        self.get_logger().info(f'waiting for map_load service')
        while not self.map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.map_req = LoadMap.Request()
        self.house_map = self.send_map_request(map_path=map_path)
        self.get_logger().info(f'Map Loaded Correctly')
        self.map_generation(self.house_map)
        while not self.robot_pose_check:
            rclpy.spin_once(self)
        self.reach_room()
        


    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])[2]
        self.robot_pose = self.create_PoseStamped(self.navigator, x, y, yaw)
        self.robot_pose_check = True

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
        self.create_room_map(map_2D, originX, originY, resolution)
              
    def create_room_map(self, map2D, originX, originY, resolution):
        y_height, x_width = map2D.shape
        idx_matrix = resolution*np.indices((y_height, x_width )) 
        self.room_map_position = np.zeros((y_height, x_width, 2))
        self.room_map = np.zeros((y_height, x_width)) +50
        self.room_map_position[:,:,0] = idx_matrix[1] + originX  + 0.5*resolution
        self.room_map_position[:,:,1] = idx_matrix[0] + originY  + 0.5*resolution

        point1 = room_coordinates[ROOM][0]
        point2 = room_coordinates[ROOM][1]
        
        for row in range(y_height):
            for col in range(x_width):
                if  min(point1[0], point2[0]) < self.room_map_position[row,col][0]  < max(point1[0], point2[0]):
                    if min(point1[1], point2[1]) < self.room_map_position[row,col][1]  < max(point1[1], point2[1]):
                        self.room_map[row, col] = 0

        if ROOM == 7:
            point1 = room_coordinates[ROOM+1][0]
            point2 = room_coordinates[ROOM+1][1]
            for row in range(y_height):
                for col in range(x_width):
                    if  min(point1[0], point2[0]) < self.room_map_position[row,col][0]  < max(point1[0], point2[0]):
                        if min(point1[1], point2[1]) < self.room_map_position[row,col][1]  < max(point1[1], point2[1]):
                            self.room_map[row, col] = 0


        self.publish_room_map() 

    def publish_room_map(self):
        room_map_msg = self.map_data
        room_map_1D = self.room_map.reshape(1,self.room_map.shape[0] * self.room_map.shape[1])
        room_map_msg.data = (room_map_1D.squeeze().astype(np.int8)).tolist()
        room_map_msg.info.width = self.room_map.shape[1]
        room_map_msg.info.height = self.room_map.shape[0]
        self.room_map_publisher.publish(room_map_msg)
    

    def reach_room (self):
        center = np.mean(room_coordinates[ROOM], axis=0)
        goal = self.create_PoseStamped(self.navigator, center[0], center[1], 0)
        goals = self.navigator.getPath(self.robot_pose, goal).poses[30::30]
        goals.append(goal)
        init_pos = [self.robot_pose.pose.position.x, self.robot_pose.pose.position.y] 
        for j, goal in enumerate(goals):

            xx = goal.pose.position.x
            yy = goal.pose.position.y
            goal_vector = np.array([xx - init_pos[0], yy - init_pos[1]]) 
            goal_angle = np.arctan2(goal_vector[1], goal_vector[0]) 
            rounded_goal = self.create_PoseStamped(self.navigator, np.round(xx,3), np.round(yy,3), goal_angle)
            self.navigator.goToPose(rounded_goal)
            init_pos = [xx, yy]
            i= 0
            while not self.navigator.isTaskComplete():
                if i == 0 and j == 0:
                    print(f"Reaching Room Number {ROOM}")
                    i += 1

        self.plan_path()
        lamp_on = Bool()
        lamp_on.data = True
        self.lamp_on_pub.publish(lamp_on)
            
    def plan_path(self):
        y_height,x_width = self.room_map.shape
        goals = []
        point1 = room_coordinates[ROOM][0]
        point2 = room_coordinates[ROOM][1]
        deltax = np.abs(point1[0] - point2[0])
        deltay = np.abs(point1[1] - point2[1])
        first_row_found = False
        first_col_found = False
                        
        distance = 1
        with open(self.file_path, "w") as file:
            if deltax > deltay:
                row = 0
                first_row = 0
                while row < y_height:       
                    # Determine the direction (left to right or right to left) based on the row index
                    direction = 1 if (row- first_row) % (2*distance) == 0 else -1
                    if direction == 1:
                        col = 0
                        while col < x_width:
                            
                            if (min(point1[0], point2[0]) +WALL_DISTANCE) < self.room_map_position[row, col][0] < (max(point1[0], point2[0]) - WALL_DISTANCE):
                                if (min(point1[1], point2[1]) + WALL_DISTANCE) < self.room_map_position[row, col][1] < (max(point1[1], point2[1]) - WALL_DISTANCE):
                                    goal = [self.room_map_position[row, col][0], self.room_map_position[row, col][1], 0]
                                    goals.append(goal)
                                    distance = 10
                                    if not first_row_found :
                                        first_row = row + 2*distance
                                        first_row_found = True  
                                    file.write(f"{goal[0]} {goal[1]} {goal[2]}\n")
                            col += distance        

                    else :
                        col = x_width-1
                        while col > 0:
                            if (min(point1[0], point2[0]) + WALL_DISTANCE) < self.room_map_position[row, col][0] < (max(point1[0], point2[0]) - WALL_DISTANCE):
                                if (min(point1[1], point2[1]) + WALL_DISTANCE) < self.room_map_position[row, col][1] < (max(point1[1], point2[1]) - WALL_DISTANCE):
                                    goal = [self.room_map_position[row, col][0], self.room_map_position[row, col][1], 0]
                                    goals.append(goal)
                                    distance =10 
                                    if not first_row_found :
                                        first_row = row + distance
                                        first_row_found = True
                                    file.write(f"{goal[0]} {goal[1]} {goal[2]}\n")
                            col -= distance
                            
                    row += distance
            else:
                col = 0
                first_col = 0
                while col < x_width:       
                    # Determine the direction (left to right or right to left) based on the row index
                    direction = 1 if (col- first_col) % (2*distance) == 0 else -1
                    if direction == -1:
                        row = 0
                        while row < y_height :
                            
                            if (min(point1[0], point2[0]) + WALL_DISTANCE) < self.room_map_position[row, col][0] < (max(point1[0], point2[0]) - WALL_DISTANCE):
                                if (min(point1[1], point2[1]) + WALL_DISTANCE) < self.room_map_position[row, col][1] < (max(point1[1], point2[1]) - WALL_DISTANCE):
                                    goal = [self.room_map_position[row, col][0], self.room_map_position[row, col][1], 0]
                                    goals.append(goal)
                                    distance = 10 
                                    if not first_col_found :
                                        first_col = col + 2*distance 
                                        first_col_found = True 
                                    file.write(f"{goal[0]} {goal[1]} {goal[2]}\n")
                            row += distance        

                    else :
                        row = y_height-1
                        while row > 0:
                            if (min(point1[0], point2[0]) + WALL_DISTANCE) < self.room_map_position[row, col][0] < (max(point1[0], point2[0]) - WALL_DISTANCE):
                                if (min(point1[1], point2[1]) + WALL_DISTANCE) < self.room_map_position[row, col][1] < (max(point1[1], point2[1]) - WALL_DISTANCE):
                                    goal = [self.room_map_position[row, col][0], self.room_map_position[row, col][1], 0]
                                    goals.append(goal)
                                    distance =10 
                                    if not first_col_found :
                                        first_col = col + distance
                                        first_col_found = True  
                                    file.write(f"{goal[0]} {goal[1]} {goal[2]}\n")
                            row -= distance
                            
                    col += distance






                # for row in range(0, y_height, distance):
                    
                #     # Determine the direction (left to right or right to left) based on the row index
                #     direction = 1 if row % (2*distance) == 0 else -1
                #     if direction == 1:
                #         for col in range(0, x_width, distance):
                            
                #             if (min(point1[0], point2[0]) +WALL_DISTANCE) < self.room_map_position[row, col][0] < (max(point1[0], point2[0]) - WALL_DISTANCE):
                #                 if (min(point1[1], point2[1]) + WALL_DISTANCE) < self.room_map_position[row, col][1] < (max(point1[1], point2[1]) - WALL_DISTANCE):
                #                     goal = [self.room_map_position[row, col][0], self.room_map_position[row, col][1], 0]
                #                     goals.append(goal)
                #                     distance = 10
                                    
                #                     file.write(f"{goal[0]} {goal[1]} {goal[2]}\n")
                #     else :
                #         for col in reversed(range(0, x_width, distance)):
                            
                #             if (min(point1[0], point2[0]) + WALL_DISTANCE) < self.room_map_position[row, col][0] < (max(point1[0], point2[0]) - WALL_DISTANCE):
                #                 if (min(point1[1], point2[1]) + WALL_DISTANCE) < self.room_map_position[row, col][1] < (max(point1[1], point2[1]) - WALL_DISTANCE):
                #                     goal = [self.room_map_position[row, col][0], self.room_map_position[row, col][1], 0]
                #                     goals.append(goal)
                #                     distance =10
                                    
                #                     file.write(f"{goal[0]} {goal[1]} {goal[2]}\n")
            # else:

            #     for col in range(0, x_width, distance):
                    
            #         # Determine the direction (left to right or right to left) based on the row index
            #         direction = 1 if col % (2*distance) == 0 else -1
            #         if direction == 1:
            #             for row in range(0, y_height, distance):
                            
            #                 if (min(point1[0], point2[0]) +WALL_DISTANCE) < self.room_map_position[row, col][0] < (max(point1[0], point2[0]) - WALL_DISTANCE):
            #                     if (min(point1[1], point2[1]) + WALL_DISTANCE) < self.room_map_position[row, col][1] < (max(point1[1], point2[1]) - WALL_DISTANCE):
            #                         goal = [self.room_map_position[row, col][0], self.room_map_position[row, col][1], 0]
            #                         goals.append(goal)
            #                         distance = 10
                                    
            #                         file.write(f"{goal[0]} {goal[1]} {goal[2]}\n")
            #         else :
            #             for row in reversed(range(0, y_height, distance)):
                            
            #                 if (min(point1[0], point2[0]) + WALL_DISTANCE) < self.room_map_position[row, col][0] < (max(point1[0], point2[0]) - WALL_DISTANCE):
            #                     if (min(point1[1], point2[1]) + WALL_DISTANCE) < self.room_map_position[row, col][1] < (max(point1[1], point2[1]) - WALL_DISTANCE):
            #                         goal = [self.room_map_position[row, col][0], self.room_map_position[row, col][1], 0]
            #                         goals.append(goal)
            #                         distance =10
                                    
            #                         file.write(f"{goal[0]} {goal[1]} {goal[2]}\n")

        
        
        nav_on = Bool()
        nav_on.data = True
        print("Navigation on", nav_on.data)
        self.nav_on_pub.publish(nav_on)
        


        

    def create_PoseStamped(self, nav: BasicNavigator, xx, yy, yaw):
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

    
        
            

    

def main(args=None):
    global ROOM
    if len(sys.argv) > 1:
        try:
            ROOM = int(sys.argv[1])
        except ValueError:
            print("Invalid argument for ROOM. Please provide an integer.")
            return

    print("room number", ROOM)
    map_path = os.path.abspath('src/sanification/files/big_house_map.yaml')
    file_path= os.path.abspath('src/sanification/files/goal_sequence.txt')
    rclpy.init(args=args)
    planner = Planner(map_path, file_path)

    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
