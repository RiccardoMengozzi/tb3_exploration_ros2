import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import os
import tf_transformations
import numpy as np
from std_msgs.msg import Bool


class GoalPublisherNode(Node):

    def __init__(self, file_path):
        super().__init__('goal_publisher_node')
        self.nav_on = False
        self.nav = BasicNavigator()
        self.nav_on_sub = self.create_subscription(Bool, '/navigate', self.nav_on_callback, 10)
        self.lamp_on_pub = self.create_publisher(Bool, '/lamp',10)
        self.file_path= file_path 
        self.get_logger().info(f"Waiting the Planner to write goals")
        while not self.nav_on:
            rclpy.spin_once(self)
            pass
        self.get_logger().info(f"Start Sanification of The room")        
        
        #self.timer = self.create_timer(100, self.read_goals_from_file)

    def nav_on_callback(self, msg: Bool):
        self.nav_on = msg.data
        if self.nav_on:
            lamp_on = Bool()
            lamp_on.data = True
            self.lamp_on_pub.publish(lamp_on)
            self.read_goals_from_file()
            


    def read_goals_from_file(self):
        self.goals = []
        print("self.file_path", self.file_path)
        try:
            with open(self.file_path, 'r') as file:
                for line in file:
                    x, y, theta = map(float, line.split())
                    goal = [x, y, theta]
                    self.goals.append(goal)
        except Exception as e:
            self.get_logger().error(f"Error reading goals from file: {str(e)}")
        print("Goals:", self.goals)
        for i in range(len(self.goals)-1):
            vector = [self.goals[i+1][0] - self.goals[i][0] , (self.goals[i+1][1] - self.goals[i][1])]
            angle = np.arctan2(vector[1],vector[0])
            self.goals[i][2] = angle
        self.publish_goal()


    def publish_goal(self):

        while len(self.goals) > 0:
            goal = self.goals.pop(0)
            goal_pose = create_PoseStamped(self.nav, goal[0], goal[1], goal[2])
            self.nav.goToPose(goal_pose)
            i = 0
            while not self.nav.isTaskComplete():
                if i == 0:
                    self.get_logger().info(f"Published goal: {goal_pose.pose.position.x:.2}, {goal_pose.pose.position.y:.2}")
                    i += 1

        self.get_logger().info(f"Room Sanification Completed")
        self.nav_on = False
        lamp_on = Bool()
        lamp_on.data = False
        self.lamp_on_pub.publish(lamp_on)



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

def main(args=None):
    
    script_dir = os.getcwd()
    print("script_dir", script_dir)

    # Assuming the script is somewhere in the project and you want to navigate to the project root
    # Navigate to the project root by going up directories (e.g., three levels up in this case)
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    print("project_root", project_root)

    # Relative path to the target file from the project root
    relative_path = "src/sanification/files/goal_sequence.txt"
    print("relative_path", relative_path)

    # Combine the project root and the relative path to form the absolute path
    file_path = os.path.join(project_root, relative_path)
    #file_path = os.path.join(os.getcwd(), file_path)
    print("file_path", file_path)
    rclpy.init(args=args)
    goal_publisher = GoalPublisherNode(file_path)
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
