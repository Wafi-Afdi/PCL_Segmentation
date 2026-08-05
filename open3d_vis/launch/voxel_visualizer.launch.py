from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_cloud = DeclareLaunchArgument(
        'input_cloud_topic', default_value='/output_cloud',
        description='Input point cloud topic')
    input_clusters = DeclareLaunchArgument(
        'input_clusters_topic', default_value='/clusters',
        description='Input clusters topic')
    input_cylinders = DeclareLaunchArgument(
        'input_cylinders_topic', default_value='/cylinders',
        description='Input cylinders topic')
    input_odom = DeclareLaunchArgument(
        'input_odom_topic', default_value='/mavros/odometry/out',
        description='Odometry topic for drone pose')
    input_tracked_cylinders = DeclareLaunchArgument(
        'input_tracked_cylinders_topic', default_value='/global_cylinders',
        description='Global tracked cylinders topic')
    voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.1',
        description='Voxel grid leaf size in meters')
    show_clusters = DeclareLaunchArgument(
        'show_clusters', default_value='true',
        description='Show clusters or not')
    show_cylinders = DeclareLaunchArgument(
            'show_cylinders', default_value='true',
            description='Show cylinders or not')
    show_normals = DeclareLaunchArgument(
            'show_normals', default_value='false',
            description='Show normals or not')
    show_clouds = DeclareLaunchArgument(
            'show_clouds', default_value='true',
            description='Show clouds or not')
    show_tracked = DeclareLaunchArgument(
            'show_tracked', default_value='true',
            description='Show tracked or not')

    return LaunchDescription([
        input_cloud,
        input_clusters,
        input_cylinders,
        input_odom,
        input_tracked_cylinders,
        voxel_size,
        show_tracked,
        show_clouds,
        show_normals,
        show_clusters,
        show_cylinders,
        Node(
            package='open3d_vis',
            executable='voxel_visualizer',
            name='voxel_visualizer',
            output='screen',
            parameters=[{
                'voxel_size': LaunchConfiguration('voxel_size'),
                'show_cylinders': LaunchConfiguration('show_cylinders'),
                'show_clusters': LaunchConfiguration('show_clusters'),
                'show_clouds': LaunchConfiguration('show_clouds'),
                'show_normals': LaunchConfiguration('show_normals'),
                'show_tracked': LaunchConfiguration('show_tracked'),

            }],
            remappings=[
                ('/input_cloud', LaunchConfiguration('input_cloud_topic')),
                ('/clusters', LaunchConfiguration('input_clusters_topic')),
                ('/cylinders', LaunchConfiguration('input_cylinders_topic')),
                ('/odom', LaunchConfiguration('input_odom_topic')),
                ('/global_cylinders', LaunchConfiguration('input_tracked_cylinders_topic')),
            ],
        ),
    ])