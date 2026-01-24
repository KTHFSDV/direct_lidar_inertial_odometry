#
#   Copyright (c)
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare('direct_lidar_inertial_odometry')

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    mapping = LaunchConfiguration('mapping', default='false')
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot')

    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/ouster/points')
    imu_topic = LaunchConfiguration('imu_topic', default='/ouster/imu')
    mission_topic = LaunchConfiguration('mission_topic', default='/state_machine/mission')

    accel_topic = LaunchConfiguration('accel_topic', default='/ros2can/recv/SBG_ECAN_MSG_IMU_ACCEL')
    gyro_topic = LaunchConfiguration('gyro_topic', default='/ros2can/recv/SBG_ECAN_MSG_IMU_GYRO')
    timestamp_topic = LaunchConfiguration('timestamp_topic', default='/ros2can/recv/SBG_ECAN_MSG_IMU_INFO')

    # Declare arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value=mapping,
        description='Enable DLIO mapping node'
    )
    declare_robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=robot_namespace,
        description='Namespace for mapping node'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )
    declare_mission_topic_arg = DeclareLaunchArgument(
        'mission_topic',
        default_value=mission_topic,
        description='Mission topic name'
    )

    declare_accel_topic_arg = DeclareLaunchArgument(
        'accel_topic',
        default_value=accel_topic,
        description='Accelerometer topic name'
    )

    declare_gyro_topic_arg = DeclareLaunchArgument(
        'gyro_topic',
        default_value=gyro_topic,
        description='Gyroscope topic name'
    )

    declare_timestamp_topic_arg = DeclareLaunchArgument(
        'timestamp_topic',
        default_value=timestamp_topic,
        description='Timestamp topic name'
    )

    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])

    # DLIO Odometry Node
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('mission', mission_topic),
            ('accel', accel_topic),
            ('gyro', gyro_topic),
            ('timestamp', timestamp_topic),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    # DLIO Mapping Node (conditional)
    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        namespace=robot_namespace,
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('~/keyframes', 'dlio/odom_node/pointcloud/keyframe'),
            ('~/map', 'dlio/map_node/map'),
        ],
        condition=IfCondition(mapping)
    )

    # RViz node (conditional)
    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlio.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_mapping_arg,
        declare_robot_namespace_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_mission_topic_arg,
        declare_accel_topic_arg,
        declare_gyro_topic_arg,
        declare_timestamp_topic_arg,
        dlio_odom_node,
        dlio_map_node,
        rviz_node
    ])
