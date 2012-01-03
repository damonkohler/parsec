// Copyright 2011 Google Inc.
// Author: moesenle@google.com (Lorenz Moesenlechner)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "parsec_perception/laser_to_pointcloud_converter.h"

#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>

namespace parsec_perception {

void LaserToPointCloudConverter::onInit() {
  input_scan_subscriber_ =
      getPrivateNodeHandle().subscribe<sensor_msgs::LaserScan>(
          "input", 1, boost::bind(
              &LaserToPointCloudConverter::ScanCallback, this, _1));
  output_cloud_publisher_ =
      getPrivateNodeHandle().advertise<pcl::PointCloud<pcl::PointXYZ> >(
          "output", 100);
}

void LaserToPointCloudConverter::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  laser_geometry::LaserProjection laser_projection;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2());
  laser_projection.projectLaser(*scan, *cloud_msg);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  output_cloud_publisher_.publish(cloud);
}

}  // namespace parsec_perception

PLUGINLIB_DECLARE_CLASS(parsec_perception, LaserToPointCloudConverter,
                        parsec_perception::LaserToPointCloudConverter,
                        nodelet::Nodelet);
