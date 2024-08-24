import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityController(Node):

    def __init__(self):
        print("ciao3")
        super().__init__('velocity_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_callback()


    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VelocityController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()