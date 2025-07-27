import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    packg_name = 'navigation'
    robot_name = "amazebot.sdf"
    world_name = "amazeworld.sdf"
    rviz_name = "amazebot.rviz"

    pkg_navigation = get_package_share_directory(packg_name)
    robot_path = os.path.join(pkg_navigation, 'models', robot_name)
    world_path = os.path.join(pkg_navigation, 'worlds', world_name)
    rviz_path = os.path.join(pkg_navigation, 'rviz', rviz_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    with open(robot_path, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    start_gazebo_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', world_path],
        output='screen')

    # Bridge to forward tf and joint states to ros2
    gz_topic = '/model/amazebot'
    joint_state_gz_topic = '/world/amazeworld' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    start_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # Lidar Scan (Gazebo -> ROS2)
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            # Imu (Gazebo -> ROS2)
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
            ('lidar', '/scan'),
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time},
        ]
    )

    # Publish sensor TFs
    start_sensor_tf_publisher_cmd = Node(
        package=packg_name,
        executable='sensor_tf_publisher',
        name='sensor_tf_publisher_node',
    )

    # Publish odom TF
    start_odom_tf_publisher_cmd = Node(
        package=packg_name,
        executable='odom_tf_publisher',
        name='odom_tf_publisher_node',
    )

    # Publish /cmd_vel to /model/amazebot/cmd_vel
    start_cmd_vel_publisher_cmd = Node(
        package=packg_name,
        executable='cmd_vel_publisher',
        name='cmd_vel_publisher_node',
    )

    # Start Kimi
    start_kimi_cmd = Node(
        package=packg_name,
        executable='start_kimi',
        name='start_kimi_node',
    )

    # Start microphone
    #start_microphone_cmd = Node(
    #    package=packg_name,
    #    executable='start_microphone',
    #    name='microphone_node',
    #    emulate_tty=True,  # Enable PTY to allow stdin
    #    output='screen',
    #)

    # Launch rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        rviz_launch_arg,
        start_gazebo_cmd,
        start_bridge_cmd,
        start_robot_state_publisher_cmd,
        start_sensor_tf_publisher_cmd,
        start_odom_tf_publisher_cmd,
        start_cmd_vel_publisher_cmd,
        start_kimi_cmd,
        #start_microphone_cmd,
        #start_rviz_cmd
    ])