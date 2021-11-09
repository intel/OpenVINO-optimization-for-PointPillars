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

#ifndef LIDAR_OBSTACLE_DETECTION_RANSAC_HPP
#define LIDAR_OBSTACLE_DETECTION_RANSAC_HPP


#include <cstdint>
#include <cstddef>
#include <unordered_set>
#include <random>
#include <tuple>

#include <pcl/common/common.h>


namespace ser94mor::lidar_obstacle_detection
{

  // shorthand for point cloud pointer
  template<typename PointT>
  using PCPtr = typename pcl::PointCloud<PointT>::Ptr;


  template<typename PointT, typename id_type = uint64_t>
  class Ransac3D
  {
  private:
    static std::tuple<size_t, size_t, size_t> GetUniqueIndexes(size_t min_ind, size_t max_ind);
  public:
    static std::unordered_set<id_type>
    Segment(PCPtr<PointT> cloud, size_t max_iterations, double_t distance_tolerance);
  };

  template<typename PointT, typename id_type>
  std::unordered_set<id_type>
  Ransac3D<PointT, id_type>::Segment(PCPtr<PointT> cloud, size_t max_iterations, double_t distance_tolerance)
  {
    std::unordered_set<id_type> inliers_max{};

    double a, b, c, d;
    auto distance = [&a, &b, &c, &d](double x, double y, double z) {
      return fabs(a*x + b*y + c*z + d) / sqrt(a*a + b*b + c*c);
    };

    // For max iterations
    while (max_iterations --> 0)
    {
      std::unordered_set<id_type> inliers{};
      // get tree distinct point indexes
      auto [i1, i2, i3] = GetUniqueIndexes(0, cloud->size() - 1);

      // calculate line coefficients
      a =   (cloud->points[i2].y - cloud->points[i1].y)
          * (cloud->points[i3].z - cloud->points[i1].z)
          - (cloud->points[i2].z - cloud->points[i1].z)
          * (cloud->points[i3].y - cloud->points[i1].y);

      b =   (cloud->points[i2].z - cloud->points[i1].z)
          * (cloud->points[i1].x - cloud->points[i3].x)
          - (cloud->points[i2].x - cloud->points[i1].x)
          * (cloud->points[i3].z - cloud->points[i1].z);

      c =   (cloud->points[i2].x - cloud->points[i1].x)
          * (cloud->points[i3].y - cloud->points[i1].y)
          - (cloud->points[i2].y - cloud->points[i1].y)
          * (cloud->points[i3].x - cloud->points[i1].x);

      d = -(a * cloud->points[i1].x + b * cloud->points[i1].y + c * cloud->points[i1].z);

      for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (distance(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z) < distance_tolerance) {
          inliers.emplace(i);
        }
      }

      if (inliers.size() > inliers_max.size()) {
        inliers_max = std::move(inliers);
      }
    }

    return inliers_max;
  }

  template<typename PointT, typename id_type>
  std::tuple<size_t, size_t, size_t>
  Ransac3D<PointT, id_type>::GetUniqueIndexes(size_t min_ind, size_t max_ind)
  {
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<size_t> dis(min_ind, max_ind);

    size_t i1, i2, i3;
    i1 = dis(gen);
    do { i2 = dis(gen); } while(i1 == i2);
    do { i3 = dis(gen); } while( i1 == i3 || i2 == i3);

    return std::make_tuple(i1, i2, i3);
  }

}


#endif //LIDAR_OBSTACLE_DETECTION_RANSAC_HPP
