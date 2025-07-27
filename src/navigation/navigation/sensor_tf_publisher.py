import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3

from rclpy.qos import (QoSProfile, DurabilityPolicy, 
                    ReliabilityPolicy, HistoryPolicy)


"""
    SensorTFPublisher class is used to publish the sensor frame to the tf tree.
"""

class SensorTFPublisher(Node):
    def __init__(self):
        super().__init__('sensor_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            depth=HistoryPolicy.UNKNOWN, 
            durability=DurabilityPolicy.VOLATILE)
        
        self.pose_publisher = self.create_publisher(TFMessage, '/tf', qos_profile)

        # subscribe to /clock topic to get the current time
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.publish_sensor_tf, 10)
        
        self.get_logger().info("Sensor TF Publisher has been started...")

    def publish_sensor_tf(self, msg):
        # create a PoseStamped message with the current time and sensor frame
        pose_msg = TFMessage()

        pose_msg.transforms.append(
            TransformStamped(
                header=Header(
                    stamp=msg.clock, 
                    frame_id='amazebot/lidar_link'),
                child_frame_id='amazebot/lidar_link/lidar_sensor',
                transform=Transform(
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
        )

        ## publish imu tf
        pose_msg.transforms.append(
            TransformStamped(
                header=Header(
                    stamp=msg.clock, 
                    frame_id='amazebot/imu_link'),
                child_frame_id='amazebot/imu_link/imu_sensor',
                transform=Transform(
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
        )

        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_tf_publisher = SensorTFPublisher()
    rclpy.spin(sensor_tf_publisher)
    sensor_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()