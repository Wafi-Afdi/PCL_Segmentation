from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    yaml_params_file = DeclareLaunchArgument(
            'yaml_params_file',default_value=[
            PathJoinSubstitution([
                    FindPackageShare('open3d_vis'),
                    'config',
                    'open3d_vis_params.yaml'
                ])
            ],
            description='Path to the YAML parameter file'
        )

    return LaunchDescription([
        input_cloud,
        input_clusters,
        input_cylinders,
        input_odom,
        input_tracked_cylinders,
        yaml_params_file,
        Node(
            package='open3d_vis',
            executable='voxel_visualizer',
            name='voxel_visualizer',
            output='screen',
            parameters=[
                LaunchConfiguration('yaml_params_file'),
            ],
            remappings=[
                ('/input_cloud', LaunchConfiguration('input_cloud_topic')),
                ('/clusters', LaunchConfiguration('input_clusters_topic')),
                ('/cylinders', LaunchConfiguration('input_cylinders_topic')),
                ('/odom', LaunchConfiguration('input_odom_topic')),
                ('/global_cylinders', LaunchConfiguration('input_tracked_cylinders_topic')),
            ],
        ),
    ])