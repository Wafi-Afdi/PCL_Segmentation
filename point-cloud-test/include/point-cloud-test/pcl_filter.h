#ifndef POINT_CLOUD_TEST__PCL_FILTER_H_
#define POINT_CLOUD_TEST__PCL_FILTER_H_

#include "point-cloud-test/pcl_processor.h"
#include "zed_msgs/msg/objects_stamped.hpp"

inline pcl::PointCloud<pcl::PointXYZ>::Ptr removeNonNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float max_z_components = 0.5f)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr trunk_candidates(new pcl::PointCloud<pcl::PointXYZ>());

  if (!cloud || cloud->empty())
  {
    return trunk_candidates;
  }

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);

  ne.setRadiusSearch(0.8);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  trunk_candidates->reserve(cloud->points.size());

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    const auto &pt = cloud->points[i];
    const auto &normal = normals->points[i];

    // Check 1: Does it have enough points to represent a normal?
    // If PCL can't find enough neighbors in the 0.9m radius, it sets the normal to NaN.
    if (std::isnan(normal.normal_x) ||
        std::isnan(normal.normal_y) ||
        std::isnan(normal.normal_z))
    {
      continue; // Skip isolated points
    }

    // Check z angle threshold
    if (std::abs(normal.normal_z) < max_z_components)
    {
      trunk_candidates->points.push_back(pt);
    }
  }

  trunk_candidates->width = trunk_candidates->points.size();
  trunk_candidates->height = 1;
  trunk_candidates->is_dense = true;

  return trunk_candidates;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr cropBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const zed_msgs::msg::ObjectsStamped::SharedPtr &obj_det_)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (!cloud || cloud->empty() || !obj_det_ || obj_det_->objects.empty())
  {
    return filtered_cloud;
  }

  pcl::PointIndices::Ptr all_inliers(new pcl::PointIndices);
  pcl::CropBox<pcl::PointXYZ> box_filter;
  box_filter.setInputCloud(cloud);

  // Loop through each detected object
  for (const auto &obj : obj_det_->objects)
  {

    // Find the min and max coordinates from the 8 bounding box corners
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();

    float max_x = -std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    // Extract bounds using the specific zed_msgs nested structure
    for (int i = 0; i < 8; ++i)
    {
      // kp[0] = x, kp[1] = y, kp[2] = z
      float x = obj.bounding_box_3d.corners[i].kp[0];
      float y = obj.bounding_box_3d.corners[i].kp[1];
      float z = obj.bounding_box_3d.corners[i].kp[2];

      if (x < min_x)
        min_x = x;
      if (y < min_y)
        min_y = y;
      if (z < min_z)
        min_z = z;

      if (x > max_x)
        max_x = x;
      if (y > max_y)
        max_y = y;
      if (z > max_z)
        max_z = z;
    }

    // Set the calculated Axis-Aligned Bounding Box (AABB) limits
    box_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0f));
    box_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0f));

    // Extract indices for points inside this bounding box
    std::vector<int> indices;
    box_filter.filter(indices);

    // Append to our master list of inliers
    all_inliers->indices.insert(all_inliers->indices.end(), indices.begin(), indices.end());
  }

  // Remove duplicate indices in case multiple bounding boxes overlap
  std::sort(all_inliers->indices.begin(), all_inliers->indices.end());
  all_inliers->indices.erase(
      std::unique(all_inliers->indices.begin(), all_inliers->indices.end()),
      all_inliers->indices.end());

  // Extract the final points from the master indices list
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(all_inliers);
  extract.setNegative(false); // Keep points inside the boxes
  extract.filter(*filtered_cloud);

  return filtered_cloud;
}

#endif // POINT_CLOUD_TEST__PCL_PROCESSOR_H_