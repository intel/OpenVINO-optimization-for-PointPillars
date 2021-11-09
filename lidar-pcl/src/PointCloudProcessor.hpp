/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
 *
 * Permission is hereby granted, free of charge, to any person 
 * obtaining a copy of this software and associated documentation 
 * files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH 
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef LIDAR_OBSTACLE_DETECTION_POINTCLOUDPROCESSOR_HPP
#define LIDAR_OBSTACLE_DETECTION_POINTCLOUDPROCESSOR_HPP

#include "Box.hpp"
#include "Ransac.hpp"
#include "EuclideanClusterExtraction.hpp"

#include <chrono>
#include <boost/filesystem.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

namespace ser94mor::lidar_obstacle_detection
{
  // shorthand for point cloud pointer
  template<typename PointT>
  using PCPtr = typename pcl::PointCloud<PointT>::Ptr;

  /**
   * Template class containing different utility methods for working with point clouds.
   * @tparam PointT the type of points in the point cloud (from PCL library)
   */
  template<typename PointT>
  class PointCloudProcessor
  {
  private:
    // flag determining which implementation of RANSAC, Euclidean clustering, and k-d tree to use
    bool use_pcl_;
  public:

    /**
     * Constructor.
     * @param use_pcl flag indicating whether to use PCL library implementations of RANSAC,
     * Euclidean clustering, and k-d tree algorithms or not
     */
    explicit PointCloudProcessor(bool use_pcl) : use_pcl_{use_pcl}
    {}

    /**
     * Get number of points in the point cloud
     * @param cloud the point cloud of interest
     * @return the number of points in the point cloud
     */
    size_t NumberOfPointsIn(const PCPtr<PointT>& cloud) const;

    /**
     * Filter the initial point cloud, so that the resulting point cloud is ready for further processing.
     * It reduces number of points in the point cloud using voxel grid filtering, crops the points that are outside
     * of the region of interest, and removes points that belongs to the ego car's roof.
     *
     * @param cloud the initial point cloud
     * @param voxel_grid_leaf_size voxel grid leaf size
     * @param min_point
     * @param max_point
     * @return the pointer to filtered point cloud
     */
    PCPtr<PointT>
    FilterCloud(const PCPtr<PointT>& cloud,
                double_t voxel_grid_leaf_size,
                const Eigen::Vector4f& min_point,
                const Eigen::Vector4f& max_point) const;

 PCPtr<PointT>
plane_clip(const PCPtr<PointT>&src_cloud, 
					const Eigen::Vector4f& plane, 
					bool negative) const;
    /**
     * Separates the road from obstacles.
     * @param inliers indices of points that belong to the road
     * @param cloud cloud to separate
     * @return a pair, where the first point cloud is an obstacle point cloud and the second one is a road point cloud
     */
    std::pair<PCPtr<PointT>, PCPtr<PointT>>
    SeparateClouds(const pcl::PointIndices::Ptr& inliers, const PCPtr<PointT>& cloud) const;

    /**
     * Perform segmentation on the given point cloud using the RANSAC algorithm.
     * @param cloud the input point cloud
     * @param max_iterations the number of iteration for the RANSAC algorithm
     * @param distance_threshold points that are at te distance less than or equal to this
     *                           value will be included into the road plane
     * @return a pair, where the first point cloud is an obstacle point cloud and the second one is a road point cloud
     */
    std::pair<PCPtr<PointT>, PCPtr<PointT>>
    SegmentPlane(const PCPtr<PointT>& cloud, size_t max_iterations, double_t distance_threshold) const;

    /**
     * Split the input cloud to several clusters based on the Euclidean clustering algorithm.
     * @param cloud the input cloud
     * @param cluster_tolerance the maximum distance between the two points that should belong to one cluster
     * @param min_size the minimum cluster size
     * @param max_size the maximum cluster size
     * @return the vector of clusters extracted from the given point cloud
     */
    std::vector<PCPtr<PointT>>
    Clustering(const PCPtr<PointT>& cloud, double_t cluster_tolerance, size_t min_size, size_t max_size) const;

    /**
     * Determine coordinates of a bounding box around the point cloud.
     * @param cluster_cloud the point cloud
     * @return a Box object containing bounding box coordinates
     */
    Box BoundingBox(const PCPtr<PointT>& cluster_cloud) const;

    /**
     * Loads a *.pcd file and returns a point cloud.
     * @param file path to a *.pcd file
     * @return a point cloud
     */
    PCPtr<PointT> ReadPcdFile(const std::string& file) const;

    /**
     * Get a vector of *.pcd file paths.
     * @param data_path path to directory with *.pcd files
     * @return a vector of *.pcd file paths from {@param data_path}
     */
    std::vector<boost::filesystem::path> ListPcdFiles(const std::string& data_path) const;

  };


  template<typename PointT>
  size_t PointCloudProcessor<PointT>::NumberOfPointsIn(const PCPtr<PointT>& cloud) const
  {
    return cloud->points.size();
  }
template<typename PointT>
PCPtr<PointT>
PointCloudProcessor<PointT>::plane_clip(const PCPtr<PointT>&src_cloud, 
					const Eigen::Vector4f& plane, 
					bool negative) const
{


	pcl::PlaneClipper3D<PointT> clipper(plane);
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);

	clipper.clipPointCloud3D(*src_cloud, indices->indices);

	PCPtr<PointT> dst_cloud{new pcl::PointCloud<PointT>{}};

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(src_cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.filter(*dst_cloud);

	return dst_cloud;
}


  template<typename PointT>
  PCPtr<PointT>
  PointCloudProcessor<PointT>::FilterCloud(const PCPtr<PointT>& cloud,
                                           const double_t voxel_grid_leaf_size,
                                           const Eigen::Vector4f& min_point,
                                           const Eigen::Vector4f& max_point) const
  {

    PCPtr<PointT> cloud_filtered{new pcl::PointCloud<PointT>{}};

    // Create the filtering voxel grid point reduction object
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
    sor.filter(*cloud_filtered);  // here we significantly reduced the number of points to process

    // Create region based filtering object
    pcl::CropBox<pcl::PointXYZI> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(min_point);
    cb.setMax(max_point);
    cb.filter(*cloud_filtered); // here we cropped points that are far away from us, in which we are not interested
    // Create region based filtering object for removing car roof points
    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZI> cb_roof(true);
    cb_roof.setInputCloud(cloud_filtered);
    cb_roof.setMin(Eigen::Vector4f(-2.0, -2.0, -3.0, 1.0));
    cb_roof.setMax(Eigen::Vector4f(3.0, 2.0, 3.0, 1.0));
    cb_roof.filter(indices); // here we have indices of points belonging to the car's roof
    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices());
    for (auto ind : indices)
    {
      point_indices->indices.push_back(ind);
    }
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    // Extract all the points that DO NOT belong to car's roof
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(point_indices);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);


    return cloud_filtered;
  }


  template<typename PointT>
  std::pair<PCPtr<PointT>, PCPtr<PointT>>
  PointCloudProcessor<PointT>::SeparateClouds(
      const pcl::PointIndices::Ptr& inliers, const PCPtr<PointT>& cloud) const
  {
    // Two new point clouds, one cloud with obstacles and other with segmented plane
    PCPtr<PointT> road_cloud{new pcl::PointCloud<PointT>()};
    PCPtr<PointT> obstacles_cloud{new pcl::PointCloud<PointT>()};

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers (road points)
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road_cloud);

    // Extract the obstacle points
    extract.setNegative(true);
    extract.filter(*obstacles_cloud);

    return std::make_pair(obstacles_cloud, road_cloud);
  }


  template<typename PointT>
  std::pair<PCPtr<PointT>, PCPtr<PointT>>
  PointCloudProcessor<PointT>::SegmentPlane(
      const PCPtr<PointT>& cloud, const size_t max_iterations, const double_t distance_threshold) const
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    if (use_pcl_)
    {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      // Create the segmentation object
      pcl::SACSegmentation<PointT> seg;
      // Optional
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(max_iterations);
      seg.setDistanceThreshold(distance_threshold);

      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);
    }
    else
    {
      auto inliers_set = Ransac3D<PointT, int>::Segment(cloud, max_iterations, distance_threshold);
      inliers->indices.insert(inliers->indices.begin(), inliers_set.begin(), inliers_set.end());
    }

    if (inliers->indices.empty())
    {
      std::cerr << "[PointCloudProcessor<PointT>::SegmentPlane] "
                   "could not estimate a planar model for the given dataset." << std::endl;
      // not a fatal error
    }

    auto obst_road_pair{SeparateClouds(inliers, cloud)};


    return obst_road_pair;
  }


  template<typename PointT>
  std::vector<PCPtr<PointT>>
  PointCloudProcessor<PointT>::Clustering(
      const PCPtr<PointT>& cloud, const double_t cluster_tolerance, const size_t min_size, const size_t max_size) const
  {

    // vector to store discovered clusters in
    std::vector<PCPtr<PointT>> clusters;

    std::vector<pcl::PointIndices> cluster_indices;

    if (use_pcl_)
    {
      // Creating the KdTree object for the search method of the extraction
      typename pcl::search::KdTree<PointT>::Ptr tree{new pcl::search::KdTree<PointT>{}};
      tree->setInputCloud(cloud);

      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance(cluster_tolerance); // 2cm
      ec.setMinClusterSize(min_size);
      ec.setMaxClusterSize(max_size);
      ec.setSearchMethod(tree);
      ec.setMultiThread(true);
      ec.setInputCloud(cloud);
      ec.extract(cluster_indices);
    }
    else
    {
      using point_type = typename KDTree<3, int>::point_type;
      using vector_type = point_type::vector_type;
      std::vector<point_type> points;
      int index = 0;
      for (const auto& point : cloud->points)
      {
        points.emplace_back(point_type{index++, vector_type(point.x, point.y, point.z)});
      }
      auto indices = EuclideanClusterExtraction<3, int>::Extract(points, min_size, max_size, cluster_tolerance);
      for (const auto& cl_ind : indices)
      {
        cluster_indices.emplace_back();
        cluster_indices.back().indices.insert(cluster_indices.back().indices.begin(), cl_ind.begin(), cl_ind.end());
      }
    }


    int j = 0;
    for (const auto& cluster_indice : cluster_indices)
    {
      clusters.emplace_back(new pcl::PointCloud<PointT>()); // add new cluster
      auto& cloud_cluster{clusters.back()};
      for (auto index : cluster_indice.indices)
      {
        cloud_cluster->points.push_back(cloud->points[index]);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      j++;
    }

     //         << " clusters" << std::endl;

    return clusters;
  }


  template<typename PointT>
  Box PointCloudProcessor<PointT>::BoundingBox(const PCPtr<PointT>& cluster_cloud) const
  {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster_cloud, minPoint, maxPoint);

    Box box{minPoint.x, minPoint.y, minPoint.z,
            maxPoint.x, maxPoint.y, maxPoint.z};

    return box;
  }


  template<typename PointT>
  PCPtr<PointT> PointCloudProcessor<PointT>::ReadPcdFile(const std::string& file) const
  {

    PCPtr<PointT> cloud(new pcl::PointCloud<PointT>{});

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)
    {
      PCL_ERROR("Couldn't read file \n");
    }
    std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
              << cloud->points.size() << " data points from " << file << std::endl;

    return cloud;
  }


  template<typename PointT>
  std::vector<boost::filesystem::path> PointCloudProcessor<PointT>::ListPcdFiles(const std::string& data_path) const
  {

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{data_path},
                                               boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(paths.begin(), paths.end());

    return paths;
  }

}


#endif //LIDAR_OBSTACLE_DETECTION_POINTCLOUDPROCESSOR_HPP
