#include <mutex>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pcl_cstm_msg/msg/point_cloud_array.hpp"
#include "pcl_cstm_msg/msg/v_cylinders_fit.hpp"
#include "pcl_cstm_msg/msg/tracked_cylinder_array.hpp"
#include "point-cloud-test/pcl_processor.h"
#include "point-cloud-test/pcl_filter.h"
#include "point-cloud-test/global_cylinder_manager.hpp"

namespace point_cloud_test
{

  struct CloudPosePair
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  };

  class PclProcNode : public rclcpp::Node
  {
  public:
    PclProcNode()
        : Node("zed_pcl_proc_node")
    {
      int clustering_method_params = 1;
      int ground_removal_method_params = 1;

      this->declare_parameter<bool>("use_odom_pcl_pair", true);
      this->declare_parameter<float>("voxel_size", 0.3);
      this->declare_parameter<int>("max_pcl_points", 450000);
      this->declare_parameter<int>("callback_time", 500);

      // ground removal
      this->declare_parameter<int>("ground_removal.method", 1);
      
      this->get_parameter("ground_removal.method", ground_removal_method_params);
      this->ground_removal_method = castGroundRemovalMethod(ground_removal_method_params);

      // clustering params
      this->declare_parameter<int>("clustering_params.method", 0);
      this->declare_parameter<float>("clustering_params.euclidean_clustering.max_z_mean", 0.4);
      this->declare_parameter<float>("clustering_params.euclidean_clustering.max_z_variance", 0.1);
      this->declare_parameter<float>("clustering_params.euclidean_clustering.distance_tolerance", 0.5);
      this->declare_parameter<float>("clustering_params.region_growing.max_degrees", 3.0);
      this->declare_parameter<float>("clustering_params.region_growing.curvature_threshold", 1.0);
      

      this->get_parameter("clustering_params.method", clustering_method_params);
      this->get_parameter("clustering_params.euclidean_clustering.max_z_mean", euclidean_max_z_mean);
      this->get_parameter("clustering_params.euclidean_clustering.max_z_variance", euclidean_max_z_variance);
      this->get_parameter("clustering_params.euclidean_clustering.distance_tolerance", euclidean_max_distance);
      this->get_parameter("clustering_params.region_growing.max_degrees", regionGrowing_max_degrees);
      this->get_parameter("clustering_params.region_growing.curvature_threshold", regionGrowing_curvature_threshold);
      this->clustering_method = castClusteringMethod(clustering_method_params);

      bool is_use_odom_pcl_pair = false;
      int callback_time = 500;

      this->get_parameter("use_odom_pcl_pair", is_use_odom_pcl_pair);
      this->get_parameter("voxel_size", voxel_size);
      this->get_parameter("max_pcl_points", max_pcl_points);
      this->get_parameter("callback_time", callback_time);

      rclcpp::SubscriptionOptions sub_opts;
      rclcpp::CallbackGroup::SharedPtr sync_cb_group = create_callback_group(
          rclcpp::CallbackGroupType::Reentrant);
      sub_opts.callback_group = sync_cb_group;

      cloud_sub_.subscribe(this, "/input_cloud",
                            rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opts);

      if (is_use_odom_pcl_pair) {
        odom_sub_.subscribe(this, "/odom", rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opts);
        sync_PCL_Odometry_ = std::make_shared<SynchronizerPCLAndOdometry>(
            SyncPolicyPCLAndOdometry(10), cloud_sub_, odom_sub_);
        sync_PCL_Odometry_->registerCallback(&PclProcNode::sync_callback_odom_pcl, this);
      } else {
        pose_sub_.subscribe(this, "/pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opts);
        sync_PCL_PoseStamped_ = std::make_shared<SynchronizerPCLAndPoseStamped>(
            SyncPolicyPCLAndPoseStamped(10), cloud_sub_, pose_sub_);
        sync_PCL_PoseStamped_->registerCallback(&PclProcNode::sync_callback_pose_pcl, this);
      }
      

      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/output_cloud", rclcpp::SensorDataQoS());

      cluster_pub_ = create_publisher<pcl_cstm_msg::msg::PointCloudArray>(
          "/clusters", rclcpp::SensorDataQoS());

      cylinder_pub_ = create_publisher<pcl_cstm_msg::msg::VCylindersFit>(
          "/cylinders", rclcpp::SensorDataQoS());

      global_cylinder_pub_ = create_publisher<pcl_cstm_msg::msg::TrackedCylinderArray>(
          "/global/cylinders", rclcpp::SensorDataQoS());

      timer_ = create_wall_timer(
          std::chrono::milliseconds(callback_time),
          [this]()
          { timer_callback(); });
    }

  private:

    void sync_callback_pose_pcl(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg)
    {
      std::lock_guard<std::mutex> lock(swap_mutex_);
      write_buffer_->emplace_back(CloudPosePair{cloud_msg, pose_msg});
    }

    void sync_callback_odom_pcl(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
      auto pose_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose_ptr->pose = odom_msg->pose.pose;
      pose_ptr->header = odom_msg->header;
      std::lock_guard<std::mutex> lock(swap_mutex_);
      write_buffer_->emplace_back(CloudPosePair{cloud_msg, pose_ptr});
    }

    void timer_callback()
    {
      auto timer_cb_start = std::chrono::high_resolution_clock::now();
      std::shared_ptr<std::vector<CloudPosePair>> to_process;
      {
        std::lock_guard<std::mutex> lock(swap_mutex_);
        std::swap(write_buffer_, process_buffer_);
        to_process = process_buffer_;
      }

      if (to_process->empty())
      {
        return;
      }

      // Reduce ke 700k jika lebih dari 1,2 juta point cloud

      int total_point_clouds = 0;
      int processed_count = 0;
      for (const auto &pair : *to_process)
      {
        if (processed_count >= 7)
        {
          break; // Stop the loop after 7 pairs
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pair.cloud, *cloud);
        total_point_clouds += (cloud->width * cloud->height);
        processed_count++;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);

      auto time_filter_start = std::chrono::high_resolution_clock::now();

      processed_count = 0;
      for (auto it = to_process->rbegin(); it != to_process->rend(); ++it)
      {
        if (processed_count >= 8)
        {
          break; 
        }
        const auto &pair = *it;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NoNaN(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pair.cloud, *cloud_raw);
        std::vector<int> mapping_indices; // Menyimpan indeks poin yang valid
        pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_NoNaN, mapping_indices);
        
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(cloud_NoNaN);

        Eigen::Vector4f min_pt(-10.0f, -10.0f, -5.0f, 1.0f); 
        crop_box.setMin(min_pt);
        Eigen::Vector4f max_pt(10.0f, 10.0f, 5.0f, 1.0f);
        crop_box.setMax(max_pt);

        crop_box.filter(*cloud_filtered);

        pcl::RandomSample<pcl::PointXYZ> random_sampler;

        if (total_point_clouds > max_pcl_points)
        {
          // RCLCPP_INFO(get_logger(), "Resampling");
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
          unsigned int samples_per_pair = (max_pcl_points / to_process->size());
          random_sampler.setInputCloud(cloud_filtered);
          random_sampler.setSample(samples_per_pair);

          random_sampler.filter(*cloud_sampled);
          cloud_filtered = cloud_sampled;
        }



        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_filtered);
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel.filter(*filtered);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(filtered);
        sor.setMeanK(50);            // Default often used is 50
        sor.setStddevMulThresh(1.0); // Default often used is 1.0
        sor.filter(*filtered);

        const auto &pos = pair.pose->pose.position;
        const auto &q = pair.pose->pose.orientation;
        Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
        Eigen::Matrix3f R = rotation.toRotationMatrix();
        Eigen::Vector3f t(pos.x, pos.y, pos.z);

        for (const auto &pt : filtered->points)
        {
          if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
          {
            continue;
          }

          Eigen::Vector3f robot_pt(pt.x, pt.y, pt.z);

          if (robot_pt.norm() > 20.0f)
          {
            continue;
          }
          Eigen::Vector3f global_pt = R * robot_pt + t;
          pcl::PointXYZ p;
          p.x = global_pt.x();
          p.y = global_pt.y();
          p.z = global_pt.z();
          merged_cloud->push_back(p);
        }
        processed_count++;
      }

      auto time_filter_end = std::chrono::high_resolution_clock::now();
      double time_filter_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(time_filter_end - time_filter_start)
              .count() /
          1000.0;
      RCLCPP_INFO(get_logger(), "Total Time Filter: %lf ms, array size: %ld", time_filter_ms, to_process->size());

      merged_cloud->width = merged_cloud->size();
      merged_cloud->height = 1;
      merged_cloud->is_dense = true;

      auto time_ransac_start = std::chrono::high_resolution_clock::now();
      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> non_ground;
      if (ground_removal_method == PMF) {
        non_ground = processPMF(merged_cloud);
      } else {
        non_ground = processRANSAC(merged_cloud);
      }
      if (!non_ground || non_ground->empty())
      {
        RCLCPP_WARN(get_logger(), "Ground removal failed");
        to_process->clear();
        return;
      }
      auto time_ransac_end = std::chrono::high_resolution_clock::now();
      double time_ransac_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(time_ransac_end - time_ransac_start)
              .count() /
          1000.0;
      RCLCPP_INFO(get_logger(), "Total Time Ground Removal: %lf ms", time_ransac_ms);

      auto trunk_filter = removeNonNormals(non_ground);

      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*trunk_filter, output_msg);
      output_msg.header.stamp = now();
      output_msg.header.frame_id = "plantation";

      cloud_pub_->publish(output_msg);

      auto time_cluster_start = std::chrono::high_resolution_clock::now();
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
      if (clustering_method == RegionGrowing) {
        clusters = clusterTrees_RegionGrowing(trunk_filter, regionGrowing_max_degrees, regionGrowing_curvature_threshold);
      } else if (clustering_method == EuclideanClustering) {
        clusters = clusterTrees(trunk_filter, euclidean_max_z_mean, euclidean_max_z_variance);
      } else {
        clusters = clusterTrees(trunk_filter);
      }
      auto time_cluster_end = std::chrono::high_resolution_clock::now();

      double time_cluster_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(time_cluster_end - time_cluster_start)
              .count() /
          1000.0;
      RCLCPP_INFO(get_logger(), "Total Time Cluster: %lf ms, total_clusters: %ld", time_cluster_ms, clusters.size());

      if (!clusters.empty())
      {
        pcl_cstm_msg::msg::PointCloudArray cluster_msg;
        cluster_msg.header.stamp = now();
        cluster_msg.header.frame_id = "plantation";

        pcl_cstm_msg::msg::VCylindersFit cyl_array_msg;
        cyl_array_msg.header.stamp = now();
        cyl_array_msg.header.frame_id = "plantation";

        std::vector<CylinderParams> params_vec;
        params_vec.reserve(clusters.size());

        int is_valid = 0;
        int is_point = 0;
        auto time_fit_start = std::chrono::high_resolution_clock::now();
        for (const auto &cluster : clusters)
        {
          sensor_msgs::msg::PointCloud2 cluster_cloud;
          pcl::toROSMsg(*cluster, cluster_cloud);
          cluster_cloud.header.stamp = now();
          cluster_cloud.header.frame_id = "plantation";
          cluster_msg.clouds.push_back(std::move(cluster_cloud));

          auto params = fitCylinderZAxis(cluster);
          params_vec.push_back(params);

          pcl_cstm_msg::msg::CylinderFit cyl_msg;
          cyl_msg.header.stamp = now();
          cyl_msg.header.frame_id = "plantation";
          cyl_msg.radius = params.radius;
          cyl_msg.height = params.height;
          cyl_msg.confidence = params.confidence;

          cyl_msg.pose.position.x = params.center_x;
          cyl_msg.pose.position.y = params.center_y;
          cyl_msg.pose.position.z = params.center_z;

          Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
          Eigen::Vector3f cylinder_axis(params.dir_x, params.dir_y, params.dir_z);
          if (cylinder_axis.z() < 0)
          {
            cylinder_axis = -cylinder_axis;
          }

          Eigen::Quaternionf q;
          q.setFromTwoVectors(z_axis, cylinder_axis);

          cyl_msg.pose.orientation.x = q.x();
          cyl_msg.pose.orientation.y = q.y();
          cyl_msg.pose.orientation.z = q.z();
          cyl_msg.pose.orientation.w = q.w();

          cyl_msg.is_valid = params.isValid;

          if (params.isValid)
          {
            is_valid++;
          }
          if (params.clouds && !params.clouds->points.empty())
          {
            // Convert the point cloud
            is_point++;
            pcl::toROSMsg(*params.clouds, cyl_msg.clouds);

            cyl_msg.clouds.header = cyl_msg.header;
          }

          cyl_array_msg.cylinders.push_back(std::move(cyl_msg));
        }
        auto time_fit_end = std::chrono::high_resolution_clock::now();

        double time_fit_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(time_fit_end - time_fit_start)
                .count() /
            1000.0;
        RCLCPP_INFO(get_logger(), "Total Fit: %lf ms, valid: %d %d", time_fit_ms, is_valid, is_point);

        cluster_pub_->publish(cluster_msg);
        if (!cyl_array_msg.cylinders.empty())
        {
          cylinder_pub_->publish(cyl_array_msg);
        }

        std::vector<TrackedCylinder> tracked;

        auto time_manager_start = std::chrono::high_resolution_clock::now();
        global_manager_.process(params_vec, tracked);
        auto time_manager_end = std::chrono::high_resolution_clock::now();

        double time_manager_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(time_manager_end - time_manager_start)
                .count() /
            1000.0;
        RCLCPP_INFO(get_logger(), "Total Manager: %lf ms", time_manager_ms);

        pcl_cstm_msg::msg::TrackedCylinderArray tracked_msg;
        tracked_msg.header.stamp = now();
        tracked_msg.header.frame_id = "plantation";

        for (const auto &t : tracked)
        {
          pcl_cstm_msg::msg::TrackedCylinder tc_msg;
          tc_msg.id = t.id;
          tc_msg.seen_count = t.seen_count;
          tc_msg.missed_count = t.missed_count;
          tc_msg.cylinder.header.stamp = now();
          tc_msg.cylinder.header.frame_id = "plantation";
          tc_msg.cylinder.radius = t.radius;
          tc_msg.cylinder.height = t.height;
          tc_msg.cylinder.confidence = t.confidence;
          tc_msg.cylinder.pose.position.x = t.center_x;
          tc_msg.cylinder.pose.position.y = t.center_y;
          tc_msg.cylinder.pose.position.z = t.center_z;

          Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
          Eigen::Vector3f cylinder_axis(t.dir_x, t.dir_y, t.dir_z);
          if (cylinder_axis.z() < 0)
          {
            cylinder_axis = -cylinder_axis;
          }

          Eigen::Quaternionf q;
          q.setFromTwoVectors(z_axis, cylinder_axis);

          tc_msg.cylinder.pose.orientation.x = q.x();
          tc_msg.cylinder.pose.orientation.y = q.y();
          tc_msg.cylinder.pose.orientation.z = q.z();
          tc_msg.cylinder.pose.orientation.w = q.w();

          tc_msg.cylinder.is_valid = true;
          tracked_msg.cylinders.push_back(std::move(tc_msg));
        }

        global_cylinder_pub_->publish(tracked_msg);
      }

      to_process->clear();
      auto timer_cb_end = std::chrono::high_resolution_clock::now();

      double timer_cb_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(timer_cb_end - timer_cb_start)
              .count() /
          1000.0;
      RCLCPP_INFO(get_logger(), "Total Point Processed: %d", total_point_clouds);
      RCLCPP_INFO(get_logger(), "Total Times: %lf ms", timer_cb_ms);
    }

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;

    using SyncPolicyPCLAndOdometry = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
    using SyncPolicyPCLAndPoseStamped = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseStamped>;

    using SynchronizerPCLAndPoseStamped = message_filters::Synchronizer<SyncPolicyPCLAndPoseStamped>;
    std::shared_ptr<SynchronizerPCLAndPoseStamped> sync_PCL_PoseStamped_;

    using SynchronizerPCLAndOdometry = message_filters::Synchronizer<SyncPolicyPCLAndOdometry>;
    std::shared_ptr<SynchronizerPCLAndOdometry> sync_PCL_Odometry_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::PointCloudArray>::SharedPtr cluster_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::VCylindersFit>::SharedPtr cylinder_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::TrackedCylinderArray>::SharedPtr global_cylinder_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<std::vector<CloudPosePair>> write_buffer_{
        std::make_shared<std::vector<CloudPosePair>>()};
    std::shared_ptr<std::vector<CloudPosePair>> process_buffer_{
        std::make_shared<std::vector<CloudPosePair>>()};
    std::mutex swap_mutex_;

    GlobalCylinderManager global_manager_{2.0f};

    float voxel_size = 0.3f;
    int max_pcl_points = 450000;

    GroundRemovalMethod ground_removal_method = RANSAC;

    ClusteringMethod clustering_method = EuclideanClustering;
    float euclidean_max_z_mean = 0.4;
    float euclidean_max_z_variance = 0.1;
    float euclidean_max_distance = 0.5;
    float regionGrowing_max_degrees = 3.0;
    float regionGrowing_curvature_threshold = 1.0;
  };

} // namespace point_cloud_test

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<point_cloud_test::PclProcNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}