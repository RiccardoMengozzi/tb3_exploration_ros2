
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import threading

WATCH_ANGLE= 40 # i look for obstacle up to the angle value both on left and right
SIDE_WATCH_ANGLE = 40
DISTANCE_THRESHOLD = 0.4 # meters
WALL_THRESHOLD = 0.5

LINEAR_VEL = 0.2




class Exploration(Node):
    def __init__(self):
        super().__init__('exploration')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)   #create a publisher to the topic /cmd_vel
        self.scan_subscriber = self.create_subscription(LaserScan,'scan', self.scan_callback, 10)  #create a subscription to the topic /scan
        self.right_side_collisions = 0
        self.cb_check = False
        self.oriented = False

        

        self.scan_cb_check = False

    def wait_for_callback(self):
        if not self.cb_check:
            rclpy.spin_once(self)
        self.cb_check = False



    def scan_callback(self, msg):
        self.pointCloud = msg.ranges # save the ranges value detected by the laser
        self.scan_cb_check = True


    def check_distances(self):
        self.left_obstacle_distance =  [distance for distance in self.pointCloud[:WATCH_ANGLE] if distance>0 and distance< DISTANCE_THRESHOLD] # first "watch_angle" value are on the left side
        self.right_obstacle_distance = [distance for distance in self.pointCloud[359-WATCH_ANGLE:] if distance>0 and distance< DISTANCE_THRESHOLD ] #last "watch_angle" value are on the right side
        self.orientation_distances = [self.pointCloud[269 - (int)(SIDE_WATCH_ANGLE / 2)], self.pointCloud[269 + (int)(SIDE_WATCH_ANGLE / 2)]]


        self.left_obstacle_distance.append(float('inf'))
        self.right_obstacle_distance.append(float('inf'))

        

        right_collision_number = np.sum( np.array(self.right_obstacle_distance) <  WALL_THRESHOLD)
        left_collision_number  = np.sum( np.array(self.left_obstacle_distance) <  WALL_THRESHOLD) 
        


        if left_collision_number > 20:
            self.oriented = False
            print("WALL FRONT LEFT")
            self.orient_to_wall()
        elif right_collision_number > 20:
            self.oriented = False
            print("WALL FRONT RIGHT")    
            self.orient_to_wall()
        elif not self.oriented: 
            self.go_straight()
        else:
            self.follow_wall()

        # if left_obstacle_distance[0] != float('inf') or right_obstacle_distance[0]!= float('inf'):

        #     if min(left_obstacle_distance) <= min(right_obstacle_distance):
        #         self.turn_right()
        #     else :
        #         self.turn_left()
        # else:
        #     self.go_straight()
        


    def orient_to_wall(self):
        self.distance_1 = self.orientation_distances[0]
        self.distance_2 = self.orientation_distances[1]
    
        print("error = ",  self.distance_1 - self.distance_2)


        if np.abs(self.distance_1 - self.distance_2) > 0.01:
            print("orienting!")
            self.turn_left(0.0, 0.2)


        else:
            self.stop()
            self.oriented = True
            print("oriented")
            self.follow_wall()

    def follow_wall(self):
        print("following wall")
        
        if self.distance_1 - self.distance_2 > 0.001:
            print("Going to far!!")
            self.turn_left(LINEAR_VEL, 0.1)
        elif self.distance_1 - self.distance_2 < -0.001:
            print("Going to close!!!")
            self.turn_right(LINEAR_VEL, 0.1)
        else:
            self.go_straight()





    ################################# STATE ACTIONS #################################
       
    def go_straight(self):
        msg = Twist()
        msg.angular.z = 0.0 #rad/sec
        msg.linear.x  = LINEAR_VEL # m/sec
        self.get_logger().info('I Go straight')
        self.velocity_publisher.publish(msg)

    def stop(self):
        msg = Twist()
        msg.angular.z = 0.0 #rad/sec
        msg.linear.x  = 0.0 # m/sec
        # self.get_logger().info('Obstacle on Right: I Turn Left')
        self.velocity_publisher.publish(msg)

    def turn_left(self, vel, rot):
        msg = Twist()
        msg.angular.z = rot #rad/sec
        msg.linear.x  = vel # m/sec
        self.get_logger().info('I Turn Left')
        self.velocity_publisher.publish(msg)

    def turn_right(self, vel, rot):
        msg = Twist()
        msg.angular.z = -rot #rad/sec
        msg.linear.x  = vel # m/sec
        self.get_logger().info('I Turn Right')
        self.velocity_publisher.publish(msg)



    def start_exploration(self):
        # Main exploration function
        self.get_logger().info(f'Starting exploration:')
        while rclpy.ok():
            self.wait_for_callback()
            self.check_distances()

            # self.get_logger().info(f"Goal Reched")



def main(args = None):
   rclpy.init(args=args)
   explorer = Exploration()
   explorer.start_exploration()
   explorer.destroy_node()
   rclpy.shutdown()



if __name__ == '__main__':
    main()