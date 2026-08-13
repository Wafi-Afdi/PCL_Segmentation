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
#include "point-cloud-test/custom_types.h"

#include "zed_msgs/msg/objects_stamped.hpp"

namespace point_cloud_test
{


  class BB_PCL_Proc_Node : public rclcpp::Node
  {
  public:
    BB_PCL_Proc_Node()
        : Node("bb_pcl_proc_node")
    {
      int clustering_method_params = 1;
      int ground_removal_method_params = 1;

      this->declare_parameter<bool>("use_odom_pcl_pair", true);
      this->declare_parameter<float>("voxel_size", 0.3);
      this->declare_parameter<int>("max_pcl_points", 450000);
      this->declare_parameter<int>("callback_time", 500);
      this->declare_parameter<std::string>("object_label_target", "pohon");

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
      this->get_parameter("object_label_target", object_label_target);

      rclcpp::SubscriptionOptions sub_opts;
      rclcpp::CallbackGroup::SharedPtr sync_cb_group = create_callback_group(
          rclcpp::CallbackGroupType::Reentrant);
      sub_opts.callback_group = sync_cb_group;

      cloud_sub_.subscribe(this, "/input_cloud",
                            rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opts);
      sub_object_det_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
      "/zed/zed_node/obj_det/objects", 10, std::bind(&BB_PCL_Proc_Node::callback_obj_det, this, std::placeholders::_1));


      if (is_use_odom_pcl_pair) {
        odom_sub_.subscribe(this, "/odom", rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opts);
        sync_PCL_Odometry_ = std::make_shared<SynchronizerPCLAndOdometry>(
            SyncPolicyPCLAndOdometry(10), cloud_sub_, odom_sub_);
        sync_PCL_Odometry_->registerCallback(&BB_PCL_Proc_Node::sync_callback_odom_pcl, this);
      } else {
        pose_sub_.subscribe(this, "/pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opts);
        sync_PCL_PoseStamped_ = std::make_shared<SynchronizerPCLAndPoseStamped>(
            SyncPolicyPCLAndPoseStamped(10), cloud_sub_, pose_sub_);
        sync_PCL_PoseStamped_->registerCallback(&BB_PCL_Proc_Node::sync_callback_pose_pcl, this);
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
    void callback_obj_det(const zed_msgs::msg::ObjectsStamped::ConstSharedPtr &obj_det_msg) {
      std::lock_guard<std::mutex> lock(obj_det_mutex_);
      latest_object_det_ = std::make_shared<zed_msgs::msg::ObjectsStamped>(*obj_det_msg);
    }

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

      if (!latest_object_det_->objects.empty())
      {
        pcl_cstm_msg::msg::PointCloudArray cluster_msg;
        cluster_msg.header.stamp = now();
        cluster_msg.header.frame_id = "plantation";

        pcl_cstm_msg::msg::VCylindersFit cyl_array_msg;
        cyl_array_msg.header.stamp = now();
        cyl_array_msg.header.frame_id = "plantation";

        std::vector<CylinderParams> params_vec;
        params_vec.reserve(latest_object_det_->objects.size());

        int is_valid = 0;
        int is_point = 0;
        auto time_fit_start = std::chrono::high_resolution_clock::now();
        for (const auto &obj : latest_object_det_->objects)
        {
          if (obj.label != object_label_target) {
            continue;
          }
          auto params = fitCylinderZAxis(obj, *to_process->back().pose);
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
            RCLCPP_INFO(get_logger(), "Cylinder Tracked: height %lf radius %lf, valid: %d", cyl_msg.height, cyl_msg.radius, is_valid);
          }

          cyl_array_msg.cylinders.push_back(std::move(cyl_msg));
        }
        auto time_fit_end = std::chrono::high_resolution_clock::now();

        double time_fit_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(time_fit_end - time_fit_start)
                .count() /
            1000.0;
        RCLCPP_INFO(get_logger(), "Total Fit: %lf ms, valid: %d", time_fit_ms, is_valid);

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

    std::mutex obj_det_mutex_;
    rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr sub_object_det_;
    zed_msgs::msg::ObjectsStamped::SharedPtr latest_object_det_; 

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

    std::string object_label_target = "pohon";
  };

} // namespace point_cloud_test

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<point_cloud_test::BB_PCL_Proc_Node>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}