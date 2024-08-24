import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import ClearEntireCostmap


LOCALIZATION_COVARIANCE_THRESHOLD = 1

WATCH_ANGLE = 60
DISTANCE_THRESHOLD = 0.5







class localizer(Node):

    def __init__(self) -> None:
        super().__init__('localizer')

        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, 10)
        # Create velocity publisher
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Add subscriber to laser scanner
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.laser_cb_check = False
        self.amcl_pose_sub_check = False

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        # Reads pose from /amcl_pose topic
        self.get_logger().info(f'reading amcl_pose...')
        self.amcl_pose = msg
        self.amcl_cov = msg.pose.covariance
        self.amcl_pose_sub_check = True

    def laser_callback(self, msg):
        pointCloud = msg.ranges # save the ranges value detected by the laser
        self.left_obstacle_distance =  [distance for distance in pointCloud[:WATCH_ANGLE] if distance>0 and distance< DISTANCE_THRESHOLD] # first "WATCH_ANGLE" value are on the left side
        self.right_obstacle_distance = [distance for distance in pointCloud[359-WATCH_ANGLE:] if distance>0 and distance< DISTANCE_THRESHOLD ] #last "WATCH_ANGLE" value are on the right side
        self.left_obstacle_distance.append(float('inf'))
        self.right_obstacle_distance.append(float('inf'))
        self.laser_cb_check = True


    def wait_for_callback(self, topic):
        # Waiting for subscriber to read data
        self.get_logger().info(f'waiting for {topic} callback')
        if topic == 'amcl_pose_sub':
            while not self.amcl_pose_sub_check: 
                rclpy.spin_once(self)
        elif topic == 'laser_sub':
           while not self.laser_cb_check: 
                rclpy.spin_once(self)

        self.amcl_pose_sub_check = False
        self.laser_cb_check = False


    def go_straight(self):
        msg = Twist()
        msg.angular.z = 0.0 #rad/sec
        msg.linear.x  = 0.3 # m/sec
        self.get_logger().info('No Obstacle: I Go straight')
        self.velocity_publisher.publish(msg)

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.2 #rad/sec
        msg.linear.x  = 0.0 # m/sec
        self.get_logger().info('Obstacle on Right: I Turn Left')
        self.velocity_publisher.publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.2 #rad/sec
        msg.linear.x  = 0.0 # m/sec
        self.get_logger().info('Obstacle on Left: I Turn Right')
        self.velocity_publisher.publish(msg)


    def stop(self):
        msg = Twist()
        msg.angular.z = 0.0 #rad/sec
        msg.linear.x  = 0.0 # m/sec
        self.get_logger().info('Robot Stoped')
        self.velocity_publisher.publish(msg)


    def check_collision(self):
        if self.left_obstacle_distance[0] != float('inf') or self.right_obstacle_distance[0]!= float('inf'):

            if min(self.left_obstacle_distance) <= min(self.right_obstacle_distance):
                self.turn_right()
            else :
                self.turn_left()
        else:
            self.go_straight()

    def send_request(self):
        # send_request function for self.amcl_cli service client
        self.future = self.amcl_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        return response
    
    def send_clear_costmap_request(self):
        # send_request function for self.clear_costmap_cli service client
        self.future = self.clear_costamp_cli.call_async(self.clear_costmap_req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        return response
    
    def localize(self):
        #Localization main function

        # This service initialize the amcl particles set (random)
        self.amcl_cli = self.create_client(Empty, 'reinitialize_global_localization')
        self.get_logger().info(f'localizing turtlebot...')
        while not self.amcl_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Empty.Request()
        self.empty_response = self.send_request()

        # Waiting for the first value of the position
        self.wait_for_callback('amcl_pose_sub')

        # Keep moving in the map untill the covariance gets low enough
        while rclpy.ok() and np.sum(self.amcl_cov)>LOCALIZATION_COVARIANCE_THRESHOLD:

            self.get_logger().info(f'covariance: {np.sum(self.amcl_cov)}')
            self.wait_for_callback('laser_sub')
            self.check_collision()      
            # At the end of every iteration read the position
            self.wait_for_callback('amcl_pose_sub')

        self.stop()
        self.get_logger().info(f'localization completed!')

        # This service clears the costmap since it gets messed up during localization
        self.clear_costamp_cli = self.create_client(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        #self.get_logger().info(f'localizing turtlebot...')
        while not self.clear_costamp_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.clear_costmap_req = ClearEntireCostmap.Request()
        self.empty_response = self.send_clear_costmap_request()



def main():
    rclpy.init()
    turtlebot_localizer = localizer()
    turtlebot_localizer.localize()
    turtlebot_localizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()