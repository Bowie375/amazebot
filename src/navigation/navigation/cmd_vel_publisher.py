import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.get_logger().info('cmd_vel publisher node started...')
        self._cmd_vel_publisher = self.create_publisher(Twist, '/model/amazebot/cmd_vel', 10)
        self._cmd_vel_subscribr = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        print("[ INFO ] cmd_vel_publisher received message")
        self._cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()