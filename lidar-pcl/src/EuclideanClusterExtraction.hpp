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

#ifndef LIDAR_OBSTACLE_DETECTION_EUCLIDEANCLUSTEREXTRACTION_HPP
#define LIDAR_OBSTACLE_DETECTION_EUCLIDEANCLUSTEREXTRACTION_HPP

#include "KDTree.hpp"

#include <vector>
#include <cstdint>
#include <random>


namespace ser94mor::lidar_obstacle_detection
{

  template<size_t dims, typename id_type = uint64_t>
  class EuclideanClusterExtraction
  {
  public:
    using kdtree_type = KDTree<dims, id_type>;
    using point_type = typename kdtree_type::point_type;

    /**
     * Extract clusters from points from point cloud.
     * @param points k-d points with identifiers
     * @param distance_tolerance the maximum distance between the two points in the cluster
     * @return vector of cluster, where cluster is a vector of point identifiers.
     */
    static std::vector<std::vector<id_type>>
    Extract(std::vector<point_type>& points,
            size_t min_cluster_size,
            size_t max_cluster_size,
            double_t distance_tolerance);

  private:
    /**
     * Function that is responsible for finding all the points from a particular cluster.
     * @param ind point index
     * @param points vector of k-d points
     * @param cluster vector to push k-d point indexes into
     * @param processed boolean vector containing flags indicating whether a particular point has been processed or not
     * @param tree a k-d tree with all the points from points
     * @param distance_tolerance the maximum distance between the two points in the cluster
     */
    static void
    Proximity(size_t ind, const std::vector<point_type>& points, std::vector<id_type>& cluster,
              std::vector<bool>& processed, const kdtree_type& tree, double_t distance_tolerance);

  };



  template<size_t dims, typename id_type>
  std::vector<std::vector<id_type>>
  EuclideanClusterExtraction<dims, id_type>::Extract(std::vector<point_type>& points,
      const size_t min_cluster_size, const size_t max_cluster_size, const double_t distance_tolerance)
  {
    // put points into the KD-Tree
    kdtree_type tree{points, kdtree_type::AS_IS};
    // vector to store found clusters in
    std::vector<std::vector<id_type>> clusters;
    // boolean array to indicate whether or not a particular point has been processed
    std::vector<bool> processed(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i)
    {
      if (processed[i])
        continue;

      clusters.emplace_back(); // add a new cluster

      Proximity(i, points, clusters.back(), processed, tree, distance_tolerance);

      if (clusters.back().size() < min_cluster_size or clusters.back().size() >= max_cluster_size)
      {
        clusters.pop_back();
      }
    }

    std::cout << "[EuclideanClusterExtraction<dims, id_type>::Extract] There are " << clusters.size()
              << " clusters found." << std::endl;

    return clusters;
  }

  template<size_t dims, typename id_type>
  void EuclideanClusterExtraction<dims, id_type>::Proximity(const size_t ind,
                                                            const std::vector<point_type>& points,
                                                            std::vector<id_type>& cluster,
                                                            std::vector<bool>& processed,
                                                            const kdtree_type& tree,
                                                            const double_t distance_tolerance)
  {
    processed[ind] = true;
    cluster.push_back(ind);
    auto search_res{tree.Search(points[ind].data, distance_tolerance)};
    
    for (auto i : search_res)
    {
      if (not processed[i])
      {
        Proximity(i, points, cluster, processed, tree, distance_tolerance);
      }
    }

  }

}

#endif //LIDAR_OBSTACLE_DETECTION_EUCLIDEANCLUSTEREXTRACTION_HPP
