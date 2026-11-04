#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <mutex>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
      this->declare_parameter<double>("cloud_history_seconds", 2.0);
      this->declare_parameter<int>("max_history_clouds", 200);
      this->declare_parameter<bool>("is_log_enabled", true);
      this->declare_parameter<bool>("is_lidar", false);
      this->declare_parameter<float>("lidar_min_dist_thresh", 0.10);
      this->declare_parameter<float>("lidar_max_dist_thresh", 0.10);
      this->declare_parameter<int>("max_pcl_odom_pair", 10);
      this->declare_parameter<std::string>("output_frame", "map");
      this->declare_parameter<std::string>("map_frame", "map");

      // ground removal
      this->declare_parameter<int>("ground_removal.method", 1);

      this->get_parameter("ground_removal.method", ground_removal_method_params);
      this->ground_removal_method = castGroundRemovalMethod(ground_removal_method_params);

      // clustering params
      this->declare_parameter<int>("clustering.max_points", 80);
      this->declare_parameter<int>("clustering.split_cluster_threshold", 250);
      this->declare_parameter<int>("clustering.min_points", 2);
      this->declare_parameter<float>("clustering.distance_thresh", 0.5);
      this->declare_parameter<float>("clustering.ellipsoid_extended_radii", 1.73589);

      this->get_parameter("clustering.max_points", max_cluster_points);
      this->get_parameter("clustering.min_points", min_cluster_points);
      this->get_parameter("clustering.distance_thresh", cluster_distance_thresh);
      this->get_parameter("clustering.ellipsoid_extended_radii", ellipsoids_extended_radii);
      this->get_parameter("clustering.split_cluster_threshold", split_threshold);

      bool is_use_odom_pcl_pair = false;
      int callback_time = 500;

      this->get_parameter("use_odom_pcl_pair", is_use_odom_pcl_pair);
      this->get_parameter("voxel_size", voxel_size);
      this->get_parameter("max_pcl_points", max_pcl_points);
      this->get_parameter("max_pcl_odom_pair", max_odom_pcl_pair);
      this->get_parameter("callback_time", callback_time);
      this->get_parameter("cloud_history_seconds", cloud_history_seconds_);
      this->get_parameter("max_history_clouds", max_history_clouds_);
      this->get_parameter("is_log_enabled", is_log_time);
      this->get_parameter("output_frame", output_frame_);
      this->get_parameter("map_frame", map_frame_);
      this->get_parameter("is_sim", is_sim);
      this->get_parameter("is_lidar", is_lidar);
      this->get_parameter("lidar_min_dist_thresh", lidar_dist_thresh);
      this->get_parameter("lidar_max_dist_thresh", lidar_dist_max);

      if (!std::isfinite(cloud_history_seconds_) || cloud_history_seconds_ <= 0.0 ||
          !std::isfinite(voxel_size) || voxel_size <= 0.0f ||
          max_pcl_points <= 0 || max_odom_pcl_pair <= 0 ||
          max_history_clouds_ <= 0 || callback_time <= 0)
      {
        throw std::invalid_argument(
            "cloud_history_seconds, voxel_size, max_pcl_points, "
            "max_pcl_odom_pair, max_history_clouds and callback_time must be positive and finite");
      }

      rclcpp::SubscriptionOptions sub_opts;
      sync_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      sub_opts.callback_group = sync_cb_group_;

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
        RCLCPP_INFO(get_logger(), "SIM STATUS: %d, running sync odom and pcl", is_sim);
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
      ellipsoid_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ellipsoids/markers",
          rclcpp::QoS(10));
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/output_cloud", rclcpp::SensorDataQoS());
      global_cylinders_sub_ = create_subscription<pcl_cstm_msg::msg::TrackedCylinderArray>(
          "/global/cylinders",
          rclcpp::SensorDataQoS(),
          std::bind(&PclObstacleDetectionNode::callback_global_cylinders,
          this, std::placeholders::_1));
      timer_ = create_wall_timer(
          std::chrono::milliseconds(callback_time),
          [this]()
          { timer_callback(); });
    }

  private:
    struct PendingCloud
    {
      sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
      geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
      float height_difference = 0.0f;
    };

    struct CachedCloud
    {
      rclcpp::Time stamp;
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    };

    // Called with swap_mutex_ held. Bound raw sensor memory between timer ticks.
    void enqueue_cloud(PendingCloud pair)
    {
      if (write_buffer_->size() >= static_cast<std::size_t>(max_odom_pcl_pair))
      {
        write_buffer_->erase(write_buffer_->begin());
      }
      write_buffer_->push_back(std::move(pair));
    }

    // Sensor timestamps and now() must use the same ROS time source.
    void prune_cloud_history(const rclcpp::Time &current_time)
    {
      cloud_history_.erase(
          std::remove_if(cloud_history_.begin(), cloud_history_.end(),
              [&](const CachedCloud &entry) {
                const double age = (current_time - entry.stamp).seconds();
                return age < 0.0 || age >= cloud_history_seconds_;
              }),
          cloud_history_.end());
    }


    void callback_global_cylinders(
      const pcl_cstm_msg::msg::TrackedCylinderArray::ConstSharedPtr &msg
    )
    {
      std::lock_guard<std::mutex> lock(global_cylinders_mutex_);
      latest_global_cylinders_ = msg;
    }


    void append_global_cylinder_ellipsoids(
      std::vector<pcl_cstm_msg::msg::AxisAlignedElipsoid> &ellipsoids
    )
    {
      pcl_cstm_msg::msg::TrackedCylinderArray::ConstSharedPtr global_cylinders;
      {
        std::lock_guard<std::mutex> lock(global_cylinders_mutex_);
        global_cylinders = latest_global_cylinders_;
      }

      if (!global_cylinders)
      {
        return;
      }

      for (const auto &tracked : global_cylinders->cylinders)
      {
        const auto &cylinder = tracked.cylinder;
        const double radius = cylinder.radius;
        const double half_height = 0.5 * cylinder.height;

        if (!cylinder.is_valid ||
            !std::isfinite(radius) || !std::isfinite(half_height) ||
            radius <= 0.0 || half_height <= 0.0)
        {
          continue;
        }

        // Cylinder local Z axis, expressed in the global frame.
        const auto &rotation = cylinder.pose.orientation;
        Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
        if (!std::isfinite(q.norm()) || q.squaredNorm() < 1e-12)
        {
          continue;
        }
        q.normalize();
        const Eigen::Vector3d axis = q * Eigen::Vector3d::UnitZ();

        // Half-size of the cylinder's axis-aligned bounding box. These
        // values provide an axis-aligned ellipsoid approximation even when
        // the cylinder is tilted. For a vertical cylinder: (r, r, h/2).
        const auto projected_radius = [radius, half_height](double axis_component) {
          const double a = std::max(-1.0, std::min(1.0, axis_component));
          return half_height * std::abs(a) +
                 radius * std::sqrt(std::max(0.0, 1.0 - a * a));
        };

        pcl_cstm_msg::msg::AxisAlignedElipsoid ellipsoid;
        ellipsoid.center = cylinder.pose.position;
        ellipsoid.radii.x = projected_radius(axis.x());
        ellipsoid.radii.y = projected_radius(axis.y());
        ellipsoid.radii.z = projected_radius(axis.z());

        ellipsoids.push_back(std::move(ellipsoid));
      }
    }

    void sync_callback_pose_pcl(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg)
    {
      std::lock_guard<std::mutex> lock(swap_mutex_);
      enqueue_cloud(PendingCloud{cloud_msg, pose_msg, 0.0f});
      // RCLCPP_INFO(get_logger(), "Running Pair");
    }

    void sync_callback_twoPose_pcl(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &ekf_pose_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &zed_pose_msg)
    {
      std::lock_guard<std::mutex> lock(swap_mutex_);
      const float height_difference =
          zed_pose_msg->pose.position.z - ekf_pose_msg->pose.position.z;
      enqueue_cloud(PendingCloud{cloud_msg, zed_pose_msg, height_difference});
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
      enqueue_cloud(PendingCloud{cloud_msg, pose_ptr, 0.0f});
    }

    void timer_callback()
    {
      auto timer_cb_start = std::chrono::high_resolution_clock::now();
      std::shared_ptr<std::vector<PendingCloud>> to_process;
      {
        std::lock_guard<std::mutex> lock(swap_mutex_);
        std::swap(write_buffer_, process_buffer_);
        to_process = process_buffer_;
      }
      const rclcpp::Time current_time = now();

      // A simulation or bag restart invalidates observations from the old timeline.
      if (last_history_time_ && current_time < *last_history_time_)
      {
        cloud_history_.clear();
      }
      last_history_time_.reset(new rclcpp::Time(current_time));
      prune_cloud_history(current_time);

      std::size_t total_point_clouds = 0;
      const std::size_t batch_size = to_process->size();
      const unsigned int samples_per_pair = batch_size == 0 ? 0 :
          std::max<std::size_t>(1, static_cast<std::size_t>(max_pcl_points) / batch_size);
      auto time_filter_start = std::chrono::high_resolution_clock::now();

      // Filter and transform NEW scans once; retain their world-frame points.
      for (const auto &pair : *to_process)
      {
        const rclcpp::Time stamp(pair.cloud->header.stamp, current_time.get_clock_type());
        const double age = (current_time - stamp).seconds();
        if (age < 0.0 || age >= cloud_history_seconds_)
        {
          continue;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NoNaN(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pair.cloud, *cloud_raw);
        total_point_clouds += cloud_raw->size();
        std::vector<int> mapping_indices; // Menyimpan indeks poin yang valid
        pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_NoNaN, mapping_indices);

        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(cloud_NoNaN);

        Eigen::Vector4f min_pt(-7.0f, -7.0f, -7.0f, 1.0f);
        crop_box.setMin(min_pt);
        Eigen::Vector4f max_pt(7.0f, 7.0f, 7.0f, 1.0f);
        crop_box.setMax(max_pt);

        crop_box.filter(*cloud_filtered);

        if (cloud_filtered->empty())
        {
          continue;
        }

        pcl::RandomSample<pcl::PointXYZ> random_sampler;

        if (cloud_filtered->size() > samples_per_pair)
        {
          // RCLCPP_INFO(get_logger(), "Resampling");
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
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

        if (filtered->size() > 2)
        {
          pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
          sor.setInputCloud(filtered);
          sor.setMeanK(static_cast<int>(std::min<std::size_t>(50, filtered->size() - 1)));
          sor.setStddevMulThresh(1.0);
          sor.filter(*filtered);
        }

        const auto &pos = pair.pose->pose.position;
        const auto &q = pair.pose->pose.orientation;
        Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
        if (!rotation.coeffs().allFinite() || rotation.squaredNorm() < 1e-12f ||
            !std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z) ||
            !std::isfinite(pair.height_difference))
        {
          continue;
        }
        rotation.normalize();
        Eigen::Matrix3f R = rotation.toRotationMatrix();
        Eigen::Vector3f t(pos.x, pos.y, pos.z);

        for (const auto &pt : filtered->points)
        {
          if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
          {
            continue;
          }

          Eigen::Vector3f optical_pt_(pt.x, pt.y, pt.z);
          float dist_norm = optical_pt_.norm();
          if (dist_norm > lidar_dist_max || dist_norm < lidar_dist_thresh)
          {
            continue;
          }
          
          Eigen::Vector3f global_pt, robot_pt;
          pcl::PointXYZ p;
          if (is_sim && !is_lidar) {
            // sim pakai kamera
            robot_pt = R_optical_to_robot_sim_ * optical_pt_;
            global_pt = R * robot_pt + t;
            p.x = global_pt.x();
            p.y = global_pt.y();
            p.z = global_pt.z(); // align to z-axis of sim world space
          } else if (is_sim && is_lidar) {
            // sim pakai lidar
            robot_pt = optical_pt_;
            global_pt = R * robot_pt + t;
            p.x = global_pt.x();
            p.y = global_pt.y();
            p.z = global_pt.z(); // align to z-axis of sim world space
          } else if (is_lidar) {
            // real pakai lidar
            robot_pt = optical_pt_;
            global_pt = R * robot_pt + t;
            p.x = global_pt.x();
            p.y = global_pt.y();
            p.z = global_pt.z(); // align to z-axis of sim world space
          } else {
            // kondisi real pakai zed
            robot_pt = optical_pt_;
            global_pt = R * robot_pt + t;
            p.x = global_pt.x();
            p.y = global_pt.y();
            p.z = global_pt.z() - pair.height_difference; // align to z-axis of ekf world space
          }
          world_cloud->push_back(p);
        }
        if (!world_cloud->empty())
        {
          cloud_history_.push_back(CachedCloud{stamp, world_cloud});
          if (cloud_history_.size() > static_cast<std::size_t>(max_history_clouds_))
          {
            // A safety cap; raise it if your scan rate requires more than 200 scans/window.
            const auto oldest = std::min_element(
                cloud_history_.begin(), cloud_history_.end(),
                [](const CachedCloud &a, const CachedCloud &b) { return a.stamp < b.stamp; });
            cloud_history_.erase(oldest);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "History scan cap reached; max_history_clouds may shorten the time window");
          }
        }
      }
      // Release raw scans immediately; history owns only filtered world-frame points.
      to_process->clear();

      pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto &entry : cloud_history_)
      {
        *merged_cloud += *entry.cloud;
      }
      if (!merged_cloud->empty())
      {
        // Merge repeated observations of the same surface before clustering.
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(merged_cloud);
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel.filter(*voxelized);
        merged_cloud = voxelized;
        if (merged_cloud->size() > static_cast<std::size_t>(max_pcl_points))
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::RandomSample<pcl::PointXYZ> sampler;
          sampler.setInputCloud(merged_cloud);
          sampler.setSample(static_cast<unsigned int>(max_pcl_points));
          sampler.filter(*sampled);
          merged_cloud = sampled;
        }
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
      pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZ>);
      if (merged_cloud->empty())
      {
        // Still publish an empty result so expired obstacles disappear downstream.
      }
      else if (ground_removal_method == PMF)
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
        non_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
      }
      auto time_ransac_end = std::chrono::high_resolution_clock::now();
      double time_ransac_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(time_ransac_end - time_ransac_start)
              .count() /
          1000.0;
      // RCLCPP_INFO(get_logger(), "Total Time Ground Removal: %lf ms", time_ransac_ms);

      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*non_ground, output_msg);
      output_msg.header.stamp = current_time;
      output_msg.header.frame_id = output_frame_;

      cloud_pub_->publish(output_msg);

      /* TODO: Lanjut Bikin AxisAlignedElipsoid disini*/
      auto time_cluster_start = std::chrono::high_resolution_clock::now();

      std::vector<pcl_cstm_msg::msg::AxisAlignedElipsoid> ellipsoids;
      if (!non_ground->empty())
      {
        ellipsoids = fitAxisAlignedEllipsoids(
              non_ground,
              min_cluster_points,   // min_points
              max_cluster_points, // max_points
              split_threshold,
              cluster_distance_thresh,  // cluster_tolerance
              ellipsoids_extended_radii
          );
      }

      const std::size_t pcl_ellipsoid_count = ellipsoids.size();

      append_global_cylinder_ellipsoids(ellipsoids);

      const std::size_t global_cylinder_count = ellipsoids.size() - pcl_ellipsoid_count;

      auto time_cluster_end = std::chrono::high_resolution_clock::now();
      double time_cluster_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(
            time_cluster_end - time_cluster_start)
            .count() / 1000.0;

      /* Publish ellipsoids */
      pcl_cstm_msg::msg::AxisAlignedElipsoidArray ellipsoid_array_msg;
      ellipsoid_array_msg.header.stamp = current_time;
      ellipsoid_array_msg.header.frame_id = output_frame_;

      int total_ellips = ellipsoids.size();
      ellipsoid_array_msg.elipsoids = std::move(ellipsoids);
      ellipsoids_pub_->publish(ellipsoid_array_msg);

      // Visualize every axis-aligned ellipsoid using one consistent color.
      // Marker scale is the full diameter, while the custom message stores radii.
      visualization_msgs::msg::MarkerArray marker_array_msg;

      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.stamp = current_time;
      clear_marker.header.frame_id = map_frame_;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array_msg.markers.push_back(clear_marker);


      for (std::size_t index = 0; index < ellipsoid_array_msg.elipsoids.size(); ++index)
      {
        const auto &ellipsoid = ellipsoid_array_msg.elipsoids[index];

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = current_time;
        marker.header.frame_id = map_frame_;
        marker.ns = "pcl_obstacle_ellipsoids";
        marker.id = static_cast<int>(index);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = ellipsoid.center;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 2.0 * ellipsoid.radii.x;
        marker.scale.y = 2.0 * ellipsoid.radii.y;
        marker.scale.z = 2.0 * ellipsoid.radii.z;

        marker.color.r = 1.0f;
        marker.color.g = 0.30f;
        marker.color.b = 0.05f;
        marker.color.a = 0.80f;

        marker_array_msg.markers.push_back(std::move(marker));
      }

      ellipsoid_markers_pub_->publish(marker_array_msg);

      auto timer_cb_end = std::chrono::high_resolution_clock::now();

      double timer_cb_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(timer_cb_end - timer_cb_start)
              .count() /
          1000.0;
      if (is_log_time) {
        RCLCPP_INFO(
          get_logger(),
          "Summary:\n"
          "  Total Ellipsoid: %d (PCL: %zu, Cylinders: %zu)\n"
          "  New Raw Points Processed: %zu\n"
          "  History Scans: %zu (window %.2f s)\n"
          "  Total Time: %.3f ms\n"
          "  Ellipsoid Fit and Cluster: %.3f ms\n"
          "  Ground Removal Time: %.3f ms\n"
          "  Filter Time: %.3f ms",
          total_ellips,
          pcl_ellipsoid_count,
          global_cylinder_count,
          total_point_clouds,
          cloud_history_.size(),
          cloud_history_seconds_,
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
    rclcpp::Subscription<pcl_cstm_msg::msg::TrackedCylinderArray>::SharedPtr global_cylinders_sub_;

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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ellipsoid_markers_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr sync_cb_group_;

    std::mutex global_cylinders_mutex_;
    pcl_cstm_msg::msg::TrackedCylinderArray::ConstSharedPtr latest_global_cylinders_;

    /* Mutex */
    std::shared_ptr<std::vector<PendingCloud>> write_buffer_{
        std::make_shared<std::vector<PendingCloud>>()};
    std::shared_ptr<std::vector<PendingCloud>> process_buffer_{
        std::make_shared<std::vector<PendingCloud>>()};
    std::mutex swap_mutex_;

    // Accessed only by the timer's mutually exclusive callback group.
    std::deque<CachedCloud> cloud_history_;
    std::unique_ptr<rclcpp::Time> last_history_time_;
    double cloud_history_seconds_ = 2.0;
    int max_history_clouds_ = 200;

    GlobalCylinderManager global_manager_{2.0f};

    float voxel_size = 0.3f;
    int max_pcl_points = 450000;
    int max_odom_pcl_pair = 10;
    std::string output_frame_ = "map";
    std::string map_frame_ = "map";

    int max_cluster_points = 10;
    int min_cluster_points = 2;
    int split_threshold = 180;
    float ellipsoids_extended_radii = 1.73859;
    float cluster_distance_thresh = 0.5f;

    GroundRemovalMethod ground_removal_method = RANSAC;

    float lidar_dist_thresh = 0.3f;
    float lidar_dist_max = 10.0f;

    bool is_log_time = true;
    bool is_sim = false;
    bool is_lidar = false;

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