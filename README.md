Segmentasi pohon sawit dengan data point cloud dengan alur:
1. Odometri dan point cloud 
2. Voxel filtering point cloud
3. Transformasi dari local ke global
4. RANSAC ground removal
5. Clustering 
6. Cylinder Fitting
7. Indeks Pohon Sawit

Repo ini terdiri dari dua package
1. `open3d_vis`: visualisasi 3D point cloud dan cluster
2. `pcl_cstm_msg`: message ROS untuk mendeskripsikan bentuk pohon yang sudah dicluster
3. `point-cloud-test`: segmentasi point cloud


## Point-Cloud-Test
Package utama yang melakukan segmentasi point cloud terdiri dari dua komponen:
1. **Frontend**: subscribe ke topic tersinkronisasi antara odometri dan point cloud dan menyimpannya.
2. **Backend**: topik utama yang melakukan segmentasi dan jalan setiap 500ms melalui timer callback.

Kedua komponen berjalan secara paralel di node `pcl_proc_node`. 

Dependencies: 
1. PCL

Cara menjalankan node
```
ros2 launch point-cloud-test pcl_proc_node.launch.py \
input_cloud_topic:=/zed2i/depth/points \
odom_topic:=/mavros/odometry/out
```

Node ini memiliki beberapa launch argument: 
1. `input_cloud_topic` (string): topic untuk input point cloud 
2. `odom_topic` (string): topic untuk input odometry robot
3. `output_cluster` (string): topic untuk output cluster pohon
4. `output_cylinders` (string): topic untuk output indeks kelapa sawit
  

## opend3d_vis
Package untuk visualisasi 3d odometri, cluster, point cloud, dan indeks kelapa sawit


Dependencies Python: 
1. Open3D

Contoh cara menjalankan node
```
ros2 launch open3d_vis voxel_visualizer.launch.py input_cloud_topic:=/output_cloud input_odom_topic:=/mavros/odometry/out voxel_size:=0.3
```

Node ini memiliki beberapa launch argument: 
1. `input_cloud_topic` (string): topic untuk input point cloud 
2. `input_odom_topic` (string): topic untuk input odometry robot
3. `input_cylinders_topic` (string): topic untuk hasil cluster cylinder sawit
4. `input_tracked_cylinders_topic` (string): topic untuk input indeks kelapa sawit
5. `voxel_size` (float): Besar voxel leaf yang dipakai oleh `pcl_proc_node`
6. `input_clusters` (string): topic untuk input cluster dari `pcl_proc_node`


## pcl_cstm_msg
Message ROS2 untuk cylinder fitting yang ingin dipublish 

Message yang ada: 
1. `PointCloudArray.msg`: array dari point cloud
2. `TrackedCylinder.msg`: indeks hasil dari cylinder fitting
3. `TrackedCylinderArray.msg`: array dari `TrackedCylinder.msg`
4. `CylinderFit.msg`: deskripsi cylinder dari cylinder fitting
5. `VCylindersFit.msg`: array dari `CylinderFit.msg`