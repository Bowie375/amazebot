import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from geometry_msgs.msg import (Pose, Point, Twist, 
                               PoseWithCovariance, 
                               TwistWithCovariance)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from rclpy.qos import (QoSProfile, DurabilityPolicy, 
                    ReliabilityPolicy, HistoryPolicy)


"""
    OdomTFPublisher class is used to publish the odom frame to the tf tree.
"""

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            depth=HistoryPolicy.UNKNOWN, 
            durability=DurabilityPolicy.VOLATILE)
        
        self.pose_publisher = self.create_publisher(TFMessage, '/tf', qos_profile)
        self.topic_publisher = self.create_publisher(Odometry, '/odom', qos_profile)

        # subscribe to /scan topic to get the current time
        self.clock_subscription = self.create_subscription(
            LaserScan, '/scan', self.publish_odom_tf, 10)
        
        self.get_logger().info("Odom TF Publisher has been started...")

    def publish_odom_tf(self, msg):
        # create a PoseStamped message with the current time and odom frame
        topic_msg = Odometry(
            header=Header(
                stamp=msg.header.stamp, 
                frame_id='odom'),
            child_frame_id='amazebot/odom',
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                ),
                covariance = [0.0] * 36
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0),
                ),
                covariance = [0.0] * 36
            )
        )

        pose_msg = TFMessage()
        pose_msg.transforms.append(
            TransformStamped(
                header=Header(
                    stamp=msg.header.stamp, 
                    frame_id='odom'),
                child_frame_id='amazebot/odom',
                transform=Transform(
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
        )

        self.pose_publisher.publish(pose_msg)
        self.topic_publisher.publish(topic_msg)

def main(args=None):
    rclpy.init(args=args)
    odom_tf_publisher = OdomTFPublisher()
    rclpy.spin(odom_tf_publisher)
    odom_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()