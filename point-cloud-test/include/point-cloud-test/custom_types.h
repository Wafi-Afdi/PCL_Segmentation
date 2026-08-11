#ifndef POINT_CLOUD_TEST_CUSTOM_TYPES_H_
#define POINT_CLOUD_TEST_CUSTOM_TYPES_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace point_cloud_test
{

  struct CloudPosePair
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  };
}

#endif
