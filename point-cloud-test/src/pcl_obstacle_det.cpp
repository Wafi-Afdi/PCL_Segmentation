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
#include "point-cloud-test/custom_types.h"
#include "point-cloud-test/pcl_processor.h"
#include "point-cloud-test/pcl_filter.h"
#include "point-cloud-test/global_cylinder_manager.hpp"
#include "pcl_cstm_msg/msg/axis_aligned_elipsoid_array.hpp"

namespace point_cloud_test
{

  class PclObstacleDetectionNode : public rclcpp::Node
  {
  public:
    PclObstacleDetectionNode()
        : Node("obstacle_det_pcl_proc_node")
    {
      int clustering_method_params = 1;
      int ground_removal_method_params = 1;

      this->declare_parameter<bool>("use_odom_pcl_pair", false);
      this->declare_parameter<bool>("is_sim", false);
      this->declare_parameter<float>("voxel_size", 0.3);
      this->declare_parameter<int>("max_pcl_points", 200000);
      this->declare_parameter<int>("callback_time", 500);
      this->declare_parameter<bool>("is_log_enabled", true);
      this->declare_parameter<std::string>("output_frame", "map");

      // ground removal
      this->declare_parameter<int>("ground_removal.method", 1);

      this->get_parameter("ground_removal.method", ground_removal_method_params);
      this->ground_removal_method = castGroundRemovalMethod(ground_removal_method_params);

      // clustering params
      this->declare_parameter<int>("clustering.max_points", 10);
      this->declare_parameter<int>("clustering.min_points", 2);
      this->declare_parameter<float>("clustering.distance_thresh", 0.5);
      this->declare_parameter<float>("clustering.ellipsoid_extended_radii", 1.73589);

      this->get_parameter("clustering.max_points", max_cluster_points);
      this->get_parameter("clustering.min_points", min_cluster_points);
      this->get_parameter("clustering.distance_thresh", cluster_distance_thresh);
      this->get_parameter("clustering.ellipsoid_extended_radii", ellipsoids_extended_radii);

      bool is_use_odom_pcl_pair = false;
      int callback_time = 500;

      this->get_parameter("use_odom_pcl_pair", is_use_odom_pcl_pair);
      this->get_parameter("voxel_size", voxel_size);
      this->get_parameter("max_pcl_points", max_pcl_points);
      this->get_parameter("callback_time", callback_time);
      this->get_parameter("is_log_enabled", is_log_time);
      this->get_parameter("output_frame", output_frame_);
      this->get_parameter("is_sim", is_sim);

      rclcpp::SubscriptionOptions sub_opts;
      rclcpp::CallbackGroup::SharedPtr sync_cb_group = create_callback_group(
          rclcpp::CallbackGroupType::Reentrant);
      sub_opts.callback_group = sync_cb_group;

      cloud_sub_.subscribe(this, "/input_cloud",
                           rclcpp::QoS(10).best_effort().get_rmw_qos_profile(), sub_opts);
      zed_pose_sub_.subscribe(this, "/zed/zed_node/pose",
                              rclcpp::QoS(10).best_effort().get_rmw_qos_profile(), sub_opts);

      if (is_use_odom_pcl_pair)
      {
        odom_sub_.subscribe(this, "/odom", rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opts);
        sync_PCL_Odometry_ = std::make_shared<SynchronizerPCLAndOdometry>(
            SyncPolicyPCLAndOdometry(10), cloud_sub_, odom_sub_);
        sync_PCL_Odometry_->registerCallback(&PclObstacleDetectionNode::sync_callback_odom_pcl, this);
      }
      else
      {
        if (is_sim) {
          ekf_pose_sub_.subscribe(this, "/ekf_pose", rclcpp::QoS(10).best_effort().get_rmw_qos_profile(), sub_opts);
          sync_PCL_PoseStamped_ = std::make_shared<SynchronizerPCLAndPoseStamped>(
              SyncPolicyPCLAndPoseStamped(10), cloud_sub_, ekf_pose_sub_);
          sync_PCL_PoseStamped_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.5));
          sync_PCL_PoseStamped_->registerCallback(&PclObstacleDetectionNode::sync_callback_pose_pcl, this);
          RCLCPP_INFO(get_logger(), "SIM STATUS: %d, running sync ekf and pcl only", is_sim);
        } else {
          ekf_pose_sub_.subscribe(this, "/ekf_pose", rclcpp::QoS(10).best_effort().get_rmw_qos_profile(), sub_opts);
          sync_PCL_TwoPoseStamped_ = std::make_shared<SynchronizerPCLAndTwoPoseStamped>(
              SyncPolicyPCLAndTwoPoseStamped(10), cloud_sub_, ekf_pose_sub_, zed_pose_sub_);
          sync_PCL_TwoPoseStamped_->registerCallback(&PclObstacleDetectionNode::sync_callback_twoPose_pcl, this);
          RCLCPP_INFO(get_logger(), "SIM STATUS: %d, running sync ekf,zed and pcl", is_sim);
        }
      }

      ellipsoids_pub_ = create_publisher<pcl_cstm_msg::msg::AxisAlignedElipsoidArray>(
          "/ellipsoids",
          rclcpp::SensorDataQoS());
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/output_cloud", rclcpp::SensorDataQoS());
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
      // RCLCPP_INFO(get_logger(), "Running Pair");
    }

    void sync_callback_twoPose_pcl(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &ekf_pose_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &zed_pose_msg)
    {
      std::lock_guard<std::mutex> lock(swap_mutex_);
      write_buffer_->emplace_back(CloudPosePair{cloud_msg, zed_pose_msg});
      heightDifference = zed_pose_msg->pose.position.z - ekf_pose_msg->pose.position.z;
      ekf_pose_z = ekf_pose_msg->pose.position.z;
      zed_pose_z = zed_pose_msg->pose.position.z;
      // RCLCPP_INFO(get_logger(), "Running Pair 2");
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
      float heightDifferenceCopy = 0.0f;
      std::shared_ptr<std::vector<CloudPosePair>> to_process;
      {
        std::lock_guard<std::mutex> lock(swap_mutex_);
        std::swap(write_buffer_, process_buffer_);
        to_process = process_buffer_;
        heightDifferenceCopy = this->heightDifference;
      }
      if (to_process->empty())
      {
        RCLCPP_INFO(get_logger(), "Timer Callback activated, but no point clouds to process");
        return;
      }

      const rclcpp::Time latest_stamp(to_process->back().cloud->header.stamp);

      // Reduce ke 700k jika lebih dari 1,2 juta point cloud

      int total_point_clouds = 0;
      int processed_count = 0;
      for (const auto &pair : *to_process)
      {
        if (processed_count >= 10)
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

      /* Fusing Point cloud data */
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

        Eigen::Vector4f min_pt(-10.0f, -10.0f, -20.0f, 1.0f);
        crop_box.setMin(min_pt);
        Eigen::Vector4f max_pt(10.0f, 10.0f, 20.0f, 1.0f);
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

          Eigen::Vector3f optical_pt_(pt.x, pt.y, pt.z);

          if (optical_pt_.norm() > 20.0f)
          {
            continue;
          }
          
          Eigen::Vector3f global_pt, robot_pt;
          pcl::PointXYZ p;
          if (is_sim) {
            robot_pt = R_optical_to_robot_sim_ * optical_pt_;
            global_pt = R * robot_pt + t;
            p.x = robot_pt.x();
            p.y = robot_pt.y();
            p.z = robot_pt.z(); // align to z-axis of ekf world space
          } else {
            robot_pt = optical_pt_;
            global_pt = R * robot_pt + t;
            p.x = global_pt.x();
            p.y = global_pt.y();
            p.z = global_pt.z() - heightDifferenceCopy; // align to z-axis of ekf world space
          }
          merged_cloud->push_back(p);
        }
        processed_count++;
      }

      auto time_filter_end = std::chrono::high_resolution_clock::now();
      double time_filter_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(time_filter_end - time_filter_start)
              .count() /
          1000.0;
      // RCLCPP_INFO(get_logger(), "Total Time Filter: %lf ms, array size: %ld", time_filter_ms, to_process->size());

      merged_cloud->width = merged_cloud->size();
      merged_cloud->height = 1;
      merged_cloud->is_dense = true;

      auto time_ransac_start = std::chrono::high_resolution_clock::now();
      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> non_ground;
      if (ground_removal_method == PMF)
      {
        non_ground = processPMF(merged_cloud);
      }
      else
      {
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
      // RCLCPP_INFO(get_logger(), "Total Time Ground Removal: %lf ms", time_ransac_ms);

      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*non_ground, output_msg);
      output_msg.header.stamp = now();
      output_msg.header.frame_id = "map";

      cloud_pub_->publish(output_msg);

      /* TODO: Lanjut Bikin AxisAlignedElipsoid disini*/
      auto time_cluster_start = std::chrono::high_resolution_clock::now();

      std::vector<pcl_cstm_msg::msg::AxisAlignedElipsoid> ellipsoids =
          fitAxisAlignedEllipsoids(
              non_ground,
              min_cluster_points,   // min_points
              max_cluster_points, // max_points
              120,
              cluster_distance_thresh,  // cluster_tolerance
              ellipsoids_extended_radii
          );

      auto time_cluster_end = std::chrono::high_resolution_clock::now();
      double time_cluster_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(
            time_cluster_end - time_cluster_start)
            .count() / 1000.0;

      /* Publish ellipsoids */
      pcl_cstm_msg::msg::AxisAlignedElipsoidArray ellipsoid_array_msg;
            ellipsoid_array_msg.header.stamp = latest_stamp;
      ellipsoid_array_msg.header.frame_id = output_frame_;

      int total_ellips = ellipsoids.size();
      ellipsoid_array_msg.elipsoids = std::move(ellipsoids);
      ellipsoids_pub_->publish(ellipsoid_array_msg);

      to_process->clear();
      auto timer_cb_end = std::chrono::high_resolution_clock::now();

      double timer_cb_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(timer_cb_end - timer_cb_start)
              .count() /
          1000.0;
      if (is_log_time) {
        RCLCPP_INFO(
          get_logger(),
          "Summary:\n"
          "  Total Ellipsoid: %d\n"
          "  Total Points Processed: %d\n"
          "  Total Time: %.3f ms\n"
          "  Ellipsoid Fit and Cluster: %.3f ms\n"
          "  Ground Removal Time: %.3f ms\n"
          "  Filter Time: %.3f ms",
          total_ellips,
          total_point_clouds,
          timer_cb_ms,
          time_cluster_ms,
          time_ransac_ms,
          time_filter_ms);
      }
    }

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> ekf_pose_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> zed_pose_sub_;

    /* Synch Policy */
    using SyncPolicyPCLAndOdometry = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
    using SyncPolicyPCLAndPoseStamped = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseStamped>;
    using SyncPolicyPCLAndTwoPoseStamped = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>;

    /* Synchronizer Definition */
    using SynchronizerPCLAndTwoPoseStamped = message_filters::Synchronizer<SyncPolicyPCLAndTwoPoseStamped>;
    std::shared_ptr<SynchronizerPCLAndTwoPoseStamped> sync_PCL_TwoPoseStamped_;

    using SynchronizerPCLAndPoseStamped = message_filters::Synchronizer<SyncPolicyPCLAndPoseStamped>;
    std::shared_ptr<SynchronizerPCLAndPoseStamped> sync_PCL_PoseStamped_;

    using SynchronizerPCLAndOdometry = message_filters::Synchronizer<SyncPolicyPCLAndOdometry>;
    std::shared_ptr<SynchronizerPCLAndOdometry> sync_PCL_Odometry_;

    /* Publishers */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::PointCloudArray>::SharedPtr cluster_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::VCylindersFit>::SharedPtr cylinder_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::TrackedCylinderArray>::SharedPtr global_cylinder_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::AxisAlignedElipsoidArray>::SharedPtr ellipsoids_pub_;
    ;
    rclcpp::TimerBase::SharedPtr timer_;

    /* Mutex */
    std::shared_ptr<std::vector<CloudPosePair>> write_buffer_{
        std::make_shared<std::vector<CloudPosePair>>()};
    std::shared_ptr<std::vector<CloudPosePair>> process_buffer_{
        std::make_shared<std::vector<CloudPosePair>>()};
    std::mutex swap_mutex_;

    GlobalCylinderManager global_manager_{2.0f};

    float voxel_size = 0.3f;
    int max_pcl_points = 450000;
    std::string output_frame_ = "map";

    int max_cluster_points = 10;
    int min_cluster_points = 2;
    float ellipsoids_extended_radii = 1.73859;
    float cluster_distance_thresh = 0.5f;

    GroundRemovalMethod ground_removal_method = RANSAC;

    float heightDifference = 0.0f; // perbedaan axis z di ekf dan zed
    float zed_pose_z = 0.0f;
    float ekf_pose_z = 0.0f;

    bool is_log_time = true;
    bool is_sim = false;

    Eigen::Matrix3f R_optical_to_robot_sim_{
        (Eigen::Matrix3f() << 0, 0, 1,
         -1, 0, 0,
         0, -1, 0)
            .finished()};
  };

} // namespace point_cloud_test

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<point_cloud_test::PclObstacleDetectionNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}