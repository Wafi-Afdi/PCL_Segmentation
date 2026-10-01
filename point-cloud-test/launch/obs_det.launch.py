from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_cloud = DeclareLaunchArgument(
        'input_cloud_topic', default_value='/zed/zed_node/pointclouds/registered',
        description='Input point cloud topic')
    ekf_pose_topic = DeclareLaunchArgument(
        'ekf_pose_topic', default_value='/mavros/local_position/pose',
        description='poseetry topic')
    zed_pose_topic = DeclareLaunchArgument(
        'zed_pose_topic', default_value='/zed/zed_node/pose',
        description='zed pose topic')
    odom_topic = DeclareLaunchArgument(
            'odom_topic', default_value='/zed/zed_node/odom',
            description='odometry zed topic')
    output_cloud = DeclareLaunchArgument(
        'output_cloud_topic', default_value='/output_cloud',
        description='Output global point cloud topic')
    output_cluster = DeclareLaunchArgument(
        'output_cluster', default_value='/clusters',
        description='Output clusters of trees')
    output_cylinders = DeclareLaunchArgument(
        'output_cylinders', default_value='/cylinders',
        description='Output clusters of trees')
    output_ellipsoids = DeclareLaunchArgument(
        'output_ellipsoids', default_value='/ellipsoids',
        description='Output ellipsoids of point clouds')
    yaml_params_file = DeclareLaunchArgument(
        'yaml_params_file',default_value=[
        PathJoinSubstitution([
                FindPackageShare('point-cloud-test'),
                'config',
                'obstacle_det_node_params.yaml'
            ])
        ],
        description='Path to the YAML parameter file'
    )

    return LaunchDescription([
        input_cloud,
        zed_pose_topic,
        ekf_pose_topic,
        odom_topic,
        output_cloud,
        output_cluster,
        output_cylinders,
        output_ellipsoids,
        yaml_params_file,
        Node(
            package='point-cloud-test',
            executable='bb_pcl_proc_node',
            name='bb_pcl_proc_node',
            output='screen',
            remappings=[
                ('/input_cloud', LaunchConfiguration('input_cloud_topic')),
                ('/pose', LaunchConfiguration('zed_pose_topic')),
                ('/odom', LaunchConfiguration('odom_topic')),
                ('/output_cloud', LaunchConfiguration('output_cloud_topic')),
                ('/clusters', LaunchConfiguration('output_cluster')),
                ('/cylinders', LaunchConfiguration('output_cylinders')),
                ('/global/cylinders', '/global_cylinders'),
            ],
            parameters=[
                LaunchConfiguration('yaml_params_file'),
            ]
        ),
        Node(
            package='point-cloud-test',
            executable='obs_det_pcl_proc_node',
            name='obstacle_det_pcl_proc_node',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/input_cloud', LaunchConfiguration('input_cloud_topic')),
                ('/ekf_pose', LaunchConfiguration('ekf_pose_topic')),
                ('/zed/zed_node/pose', LaunchConfiguration('zed_pose_topic')),
                ('/odom', LaunchConfiguration('odom_topic')),
                ('/ellipsoids', LaunchConfiguration('output_ellipsoids')),
            ],
            parameters=[
                LaunchConfiguration('yaml_params_file'),
            ]
        ),
    ])