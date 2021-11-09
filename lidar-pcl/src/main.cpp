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

//#include "Renderer.hpp"
#include "PointCloudProcessor.hpp"

#include <iostream>
#include <sstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <sys/sysinfo.h>

/* for test */
#define _FILE_PATH "./file.fifo"
#define _OBJECT_DATA "./data.fifo"
#define _SIZE_ 128

using namespace ser94mor::lidar_obstacle_detection;
using namespace std;

bool CreateFIFO(string fileName)
{
  if (access(fileName.c_str(), F_OK) == -1) {
    int ret = mkfifo(fileName.c_str(), O_CREAT | 0666);
    if (-1 == ret) {
      printf("make fifo error \n");
      return false;
    }
  }
  return true;
}

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

int main(int argc, char* argv[])
{
  cpu_set_t  mask;
  CPU_ZERO(&mask);
  //phys core1
  CPU_SET(1, &mask);
  CPU_SET(5, &mask);  
  //phys core2
  CPU_SET(2, &mask);
  CPU_SET(6, &mask);
  //phys core3
  CPU_SET(3, &mask);
  CPU_SET(7, &mask);
  sched_setaffinity(0, sizeof(mask), &mask);

  char * file_num = getCmdOption(argv, argv + argc, "-num");
  if (file_num == 0) {
    std::cout << "-num parameter is missing!" << std::endl;
    return -1;
  }
  int filenum = atoi(file_num);
  std::cout << "pcd file num : " << filenum << std::endl;

  char * file_path = getCmdOption(argv, argv + argc, "-pcd");
  if (file_path == 0) {
    std::cout << "-pcd parameter is missing!" << std::endl;
    return -1;
  }
  std::cout << "pcd file path: " << file_path << std::endl;

  using PointT = pcl::PointXYZI;
  auto point_processor = PointCloudProcessor<PointT>(true);
  pcl::PointCloud<PointT>::Ptr input_cloud;
  std::vector<pcl::PointCloud<PointT>::Ptr> input_cloud_set;
  char filename[_SIZE_];
  
  for (int j = 0; j < filenum; j++) {
        char *cur = filename;
        char *end = filename + sizeof(filename);
        cur += snprintf(cur, end - cur, file_path);
        snprintf(cur, end - cur, "/%06i.bin.pcd", j);

        //snprintf(filename, sizeof(filename),
        //    "/home/iotg/work/kitti_dataset/training/velodyne_reduced_pcd/%06i.bin.pcd", j);
        //printf("\nread --- %s\n", filename);
        std::string filename_str(filename);
        if (access(filename_str.c_str(), F_OK ) == -1) {
            std::cout << filename_str.c_str() << " file not found!!" << std::endl;
            return -1;
        }
        input_cloud = point_processor.ReadPcdFile(filename);
        input_cloud_set.push_back(input_cloud);
  }

  printf("create fifo \n");
  CreateFIFO(_FILE_PATH);
  CreateFIFO(_OBJECT_DATA);
  printf ("open fifo \n"); 
  int fd_file = open(_FILE_PATH, O_RDONLY);
  int fd_data = open(_OBJECT_DATA, O_WRONLY);
  printf("open fifo: fd_file %d, fd_data %d \n", fd_file, fd_data);

  char buf[_SIZE_] = {0};
  float dataxyz[6*100*4] = {0.0};
  int loop_id = 0;
  auto duration_total = 0; //not sure about the type
  std::chrono::high_resolution_clock::time_point t1, t2;

  while (1) {
    memset(buf, 0, sizeof(buf));
    memset(dataxyz, 0, sizeof(dataxyz));

    //auto startTime = std::chrono::steady_clock::now();
    int ret = read(fd_file, buf, _SIZE_);
    auto startTime = std::chrono::steady_clock::now();

    t1 = std::chrono::high_resolution_clock::now();
    //t1 = std::chrono::high_resolution_clock::now(); 

    if (ret < 0) {
      printf("ret = %d, read end or error \n", ret);
      break;
    } else {
      if (ret == 0)
         break;
    }

    //input_cloud = point_processor.ReadPcdFile(buf);
    input_cloud = input_cloud_set[loop_id];

    auto filterCloud =
    point_processor.FilterCloud(input_cloud, 0.5f, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(20, 6, 3, 1));
    auto segmentCloud = point_processor.SegmentPlane(input_cloud, 100, 0.20);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters =
        point_processor.Clustering(segmentCloud.first, 0.7, 10, 1000);
    
    int i = 0;
    for(const auto& cluster : cloudClusters)
    {
      point_processor.NumberOfPointsIn(cluster);
      Box box = point_processor.BoundingBox(cluster);
      dataxyz[i++] = box.x_min;
      dataxyz[i++] = box.y_min;
      dataxyz[i++] = box.z_min;
      dataxyz[i++] = box.x_max;
      dataxyz[i++] = box.y_max;
      dataxyz[i++] = box.z_max;
      if (i>=510)
         break;
    }

    ret = write(fd_data, dataxyz, i * sizeof(float));
    if (ret < 0) {
      printf("write fail\n");
      break;
    }
    t2 = std::chrono::high_resolution_clock::now();
    auto startTime2 = std::chrono::steady_clock::now();
    auto elapsedTime1 = std::chrono::duration_cast<std::chrono::milliseconds>(startTime2 - startTime);
    std::cout<< "pcl process time ---  " << elapsedTime1.count() << " ms" << std::endl;
    loop_id++;

    int duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    duration_total += duration;

    if (loop_id >= filenum)
        break;
  }

  printf("******pcl avg time = %d ms******, duration_total = %d ms, loop_id = %d\n",
      duration_total/loop_id, duration_total, loop_id);
  close(fd_file);
  close(fd_data);

  int retval;
  retval = unlink(_FILE_PATH);
  if(retval == 0){
      printf("pcl read fifo deleted.\n");
  }
  retval = unlink(_OBJECT_DATA);
  if(retval == 0){
      printf("pcl write fifo deleted.\n");
  }

  return EXIT_SUCCESS;
}
