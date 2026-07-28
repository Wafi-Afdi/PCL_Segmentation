import threading

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2

from pcl_cstm_msg.msg import PointCloudArray
from pcl_cstm_msg.msg import VCylindersFit
from pcl_cstm_msg.msg import TrackedCylinderArray


CLUSTER_COLORS = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 1.0, 0.0],
    [1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
    [0.5, 0.0, 0.0],
    [0.0, 0.5, 0.0],
    [0.0, 0.0, 0.5],
    [0.5, 0.5, 0.0],
    [0.5, 0.0, 0.5],
    [0.0, 0.5, 0.5],
]


def _make_cylinder_wireframe(center_x, center_y, base_z, radius, height, color):
    mesh = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius, height=height, resolution=20)
    r = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, 0])
    mesh.rotate(r, center=[0.0, 0.0, 0.0])
    mesh.translate([center_x, center_y, base_z + height / 2.0])

    lines = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    lines.paint_uniform_color(color)
    return lines


def _make_drone_arrow(position, orientation_q):
    qw, qx, qy, qz = orientation_q
    r_align = o3d.geometry.get_rotation_matrix_from_axis_angle(
        [0.0, np.pi / 2.0, 0.0])
    r_odom = o3d.geometry.get_rotation_matrix_from_quaternion(
        [qw, qx, qy, qz])
    r = np.dot(r_odom, r_align)

    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.1,
        cone_radius=0.25,
        cylinder_height=0.5,
        cone_height=0.3,
        resolution=20)
    arrow.rotate(r, center=[0.0, 0.0, 0.0])
    arrow.translate([position.x, position.y, position.z])
    arrow.paint_uniform_color([1.0, 0.8, 0.0])
    return arrow


class VoxelVisualizer(Node):

    def __init__(self):
        super().__init__('voxel_visualizer')

        self.declare_parameter('voxel_size', 0.1)
        self.voxel_size = self.get_parameter('voxel_size').value

        self._lock = threading.Lock()
        self._latest_cloud = None
        self._latest_clusters = None
        self._latest_cylinders = None
        self._latest_tracked_cylinders = None
        self._latest_odom = None
        self._first_frame = True

        self._vis = o3d.visualization.Visualizer()
        self._vis.create_window(window_name='Point Cloud Voxels',
                                width=1024, height=768)

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        self._vis.add_geometry(axes)
        self._geometries = [axes]

        self._drone_arrow = None

        self._sub = self.create_subscription(
            PointCloud2,
            '/input_cloud',
            self._cloud_callback,
            qos_profile_sensor_data)

        self._sub_clusters = self.create_subscription(
            PointCloudArray,
            '/clusters',
            self._clusters_callback,
            qos_profile_sensor_data)

        self._sub_cylinders = self.create_subscription(
            VCylindersFit,
            '/cylinders',
            self._cylinders_callback,
            qos_profile_sensor_data)

        self._sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            qos_profile_services_default)

        self._sub_tracked_cylinders = self.create_subscription(
            TrackedCylinderArray,
            '/global_cylinders',
            self._tracked_cylinders_callback,
            qos_profile_sensor_data)

    def _cloud_callback(self, msg: PointCloud2):
        self.get_logger().info('First point cloud received!', once=True)

        points = np.array([
            (p[0], p[1], p[2])
            for p in point_cloud2.read_points(msg, field_names=['x', 'y', 'z'])
        ], dtype=np.float32)

        if points.shape[0] == 0:
            self.get_logger().warn('Received an empty point cloud!')
            return

        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points)

        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
            o3d_cloud, voxel_size=self.voxel_size)

        with self._lock:
            self._latest_cloud = voxel_grid

    def _clusters_callback(self, msg: PointCloudArray):
        if not msg.clouds:
            return

        self.get_logger().info(
            f'Received {len(msg.clouds)} clusters', throttle_duration_sec=2.0)

        cluster_voxels = []
        for i, cloud_msg in enumerate(msg.clouds):
            points = np.array([
                (p[0], p[1], p[2])
                for p in point_cloud2.read_points(
                    cloud_msg, field_names=['x', 'y', 'z'])
            ], dtype=np.float32)

            if points.shape[0] == 0:
                continue

            o3d_cloud = o3d.geometry.PointCloud()
            o3d_cloud.points = o3d.utility.Vector3dVector(points)

            color = CLUSTER_COLORS[i % len(CLUSTER_COLORS)]

            o3d_cloud.paint_uniform_color(color)

            voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(
                o3d_cloud, voxel_size=self.voxel_size)

            cluster_voxels.append(voxel)

        if cluster_voxels:
            with self._lock:
                self._latest_clusters = cluster_voxels

    def _cylinders_callback(self, msg: VCylindersFit):
        if not msg.cylinders:
            return

        wireframes = []
        for i, cyl in enumerate(msg.cylinders):
            if cyl.is_valid:
                color = CLUSTER_COLORS[i % len(CLUSTER_COLORS)]
                lines = _make_cylinder_wireframe(
                    center_x=cyl.pose.position.x,
                    center_y=cyl.pose.position.y,
                    base_z=cyl.pose.position.z,
                    radius=cyl.radius,
                    height=cyl.height,
                    color=color)
                wireframes.append(lines)
            else:
                continue

        if wireframes:
            with self._lock:
                self._latest_cylinders = wireframes

    def _tracked_cylinders_callback(self, msg: TrackedCylinderArray):
        if not msg.cylinders:
            return

        wireframes = []
        for tc in msg.cylinders:
            cyl = tc.cylinder
            lines = _make_cylinder_wireframe(
                center_x=cyl.pose.position.x,
                center_y=cyl.pose.position.y,
                base_z=cyl.pose.position.z,
                radius=cyl.radius,
                height=cyl.height,
                color=[0.9, 0.9, 0.9])
            wireframes.append(lines)

        if wireframes:
            with self._lock:
                self._latest_tracked_cylinders = wireframes

    def _odom_callback(self, msg: Odometry):
        with self._lock:
            self._latest_odom = msg

    def render_step(self):
        clusters = None
        cylinders = None
        tracked_cylinders = None
        cloud = None
        odom = None
        with self._lock:
            if self._latest_clusters is not None:
                clusters = self._latest_clusters
                self._latest_clusters = None
                cylinders = self._latest_cylinders
                self._latest_cylinders = None
            elif self._latest_cloud is not None:
                cloud = self._latest_cloud
                self._latest_cloud = None
            if self._latest_odom is not None:
                odom = self._latest_odom
            if self._latest_tracked_cylinders is not None:
                tracked_cylinders = self._latest_tracked_cylinders
                self._latest_tracked_cylinders = None

        if odom is not None:
            self._render_drone(odom)

        if clusters is not None:
            self._render_clusters(clusters, cylinders)
        if tracked_cylinders is not None:
            self._render_tracked_cylinders(tracked_cylinders)
        elif cloud is not None:
            self._render_point_cloud(cloud)
        else:
            self._vis.poll_events()

    def _render_point_cloud(self, voxel):
        while len(self._geometries) > 1:
            self._vis.remove_geometry(self._geometries.pop(), False)

        self._vis.add_geometry(voxel, reset_bounding_box=self._first_frame)
        self._first_frame = False

        self._geometries.append(voxel)
        self._vis.poll_events()
        self._vis.update_renderer()

    def _render_clusters(self, cluster_voxels, cylinder_wireframes):
        while len(self._geometries) > 1:
            self._vis.remove_geometry(self._geometries.pop(), False)

        for voxel in cluster_voxels:
            self._vis.add_geometry(voxel, reset_bounding_box=self._first_frame)
            self._first_frame = False
            self._geometries.append(voxel)

        if cylinder_wireframes:
            for lines in cylinder_wireframes:
                self._vis.add_geometry(lines, False)
                self._geometries.append(lines)


    def _render_tracked_cylinders(self, tracked_cylinder_wireframes=None):
        if tracked_cylinder_wireframes:
            self.get_logger().info(
                        f'Received {len(tracked_cylinder_wireframes)} tracked cylinders')
            for lines in tracked_cylinder_wireframes:
                self._vis.add_geometry(lines, reset_bounding_box=self._first_frame)
                self._first_frame = False
                self._geometries.append(lines)

        self._vis.poll_events()
        self._vis.update_renderer()

    def _render_drone(self, odom_msg):
        if self._drone_arrow is not None:
            self._vis.remove_geometry(self._drone_arrow, False)

        q = odom_msg.pose.pose.orientation
        p = odom_msg.pose.pose.position
        self._drone_arrow = _make_drone_arrow(p, (q.w, q.x, q.y, q.z))

        self._vis.add_geometry(self._drone_arrow, False)
        self._vis.poll_events()
        self._vis.update_renderer()

    def destroy_node(self):
        self._vis.destroy_window()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoxelVisualizer()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.render_step()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()