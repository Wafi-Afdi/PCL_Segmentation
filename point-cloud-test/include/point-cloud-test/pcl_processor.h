#ifndef POINT_CLOUD_TEST__PCL_PROCESSOR_H_
#define POINT_CLOUD_TEST__PCL_PROCESSOR_H_

#include <iostream>
#include <chrono>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/segmentation/progressive_morphological_filter.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

inline pcl::PointCloud<pcl::PointXYZ>::Ptr processPMF(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::cout << "Starting Progressive Morphological Filter (PMF) ground removal..."
            << std::endl;
  pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices());
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;

  pmf.setInputCloud(cloud);

  pmf.setMaxWindowSize(20);
  pmf.setSlope(1.0f);
  pmf.setInitialDistance(0.5f);
  pmf.setMaxDistance(2.5f);

  auto pmf_start = std::chrono::high_resolution_clock::now();
  pmf.extract(ground_inliers->indices);
  auto pmf_end = std::chrono::high_resolution_clock::now();
  double pmf_ms =
    std::chrono::duration_cast<std::chrono::microseconds>(pmf_end - pmf_start).count() /
    1000.0;
  std::cout << "Ground segmentation finished in " << pmf_ms << " ms" << std::endl;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trees(
    new pcl::PointCloud<pcl::PointXYZ>);

  extract.setInputCloud(cloud);
  extract.setIndices(ground_inliers);
  extract.setNegative(true);
  extract.filter(*cloud_trees);

  std::cout << "Remaining tree points: "
            << cloud_trees->width * cloud_trees->height << std::endl;

  return cloud_trees;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr processRANSAC(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // std::cout << "Starting RANSAC ground removal..." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);

  seg.setDistanceThreshold(0.8);
  seg.setInputCloud(cloud);

  //auto ransac_start = std::chrono::high_resolution_clock::now();
  seg.segment(*inliers, *coefficients);
  //auto ransac_end = std::chrono::high_resolution_clock::now();

  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
    return nullptr;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trees(
    new pcl::PointCloud<pcl::PointXYZ>);

  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_trees);

  // double ransac_ms =
  //   std::chrono::duration_cast<std::chrono::microseconds>(ransac_end - ransac_start)
  //     .count() / 1000.0;
  // std::cout << "Ground removed in " << ransac_ms << " ms" << std::endl;
  // std::cout << "Remaining tree points: "
  //           << cloud_trees->width * cloud_trees->height << std::endl;

  return cloud_trees;
}

inline std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterTrees(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_trees)
{
  // std::cout << "Starting Euclidean Clustering to isolate trees..." << std::endl;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_search(
    new pcl::search::KdTree<pcl::PointXYZ>);
  tree_search->setInputCloud(cloud_trees);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  ec.setClusterTolerance(1);

  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(10000);

  ec.setSearchMethod(tree_search);
  ec.setInputCloud(cloud_trees);

  auto start_time = std::chrono::high_resolution_clock::now();
  ec.extract(cluster_indices);
  auto end_time = std::chrono::high_resolution_clock::now();

  double duration_ms =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)
      .count() / 1000.0;
  // std::cout << "Found " << cluster_indices.size() << " individual trees in "
  //           << duration_ms << " ms." << std::endl;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> isolated_trees;

  for (const auto & indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr single_tree_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & idx : indices.indices) {
      single_tree_cloud->push_back((*cloud_trees)[idx]);
    }

    single_tree_cloud->width = single_tree_cloud->size();
    single_tree_cloud->height = 1;
    single_tree_cloud->is_dense = true;

    isolated_trees.push_back(single_tree_cloud);
  }

  return isolated_trees;
}

struct CylinderParams
{
  float center_x = 0.0f;
  float center_y = 0.0f;
  float base_z = 0.0f;
  float radius = 0.0f;
  float height = 0.0f;
  float confidence = 0.0f;
  bool isValid = false;
};

inline CylinderParams fitCylinderZAxis(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster)
{
  CylinderParams result;

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cluster, minPt, maxPt);

  result.height = maxPt.z - minPt.z;
  result.base_z = minPt.z;

  float trunk_fraction = std::min(2.5f, result.height * 0.8f);

  pcl::PointCloud<pcl::PointXYZ>::Ptr trunk(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cluster);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(minPt.z, minPt.z + trunk_fraction);
  pass.filter(*trunk);

  if (trunk->points.size() < 10) {
    return result;
  }

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  ne.setSearchMethod(tree);
  ne.setInputCloud(trunk);
  ne.setKSearch(std::min(50, static_cast<int>(trunk->points.size())));
  ne.compute(*normals);

  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.10);
  seg.setRadiusLimits(0.05, 2.0);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle(0.05f);

  seg.setInputCloud(trunk);
  seg.setInputNormals(normals);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    return result;
  }

  result.radius = coefficients->values[6];

  float sum_x = 0.0f, sum_y = 0.0f;
  int count = 0;
  for (const auto & idx : inliers->indices) {
    const auto & pt = (*trunk)[idx];
    sum_x += pt.x;
    sum_y += pt.y;
    count++;
  }

  if (count > 0) {
    result.center_x = sum_x / count;
    result.center_y = sum_y / count;
  } else {
    result.center_x = coefficients->values[0];
    result.center_y = coefficients->values[1];
  }

  result.confidence = static_cast<float>(inliers->indices.size()) /
    static_cast<float>(trunk->points.size());

  result.isValid = true;
  return result;
}

#endif  // POINT_CLOUD_TEST__PCL_PROCESSOR_H_
