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

#ifndef PARSEC_PERCEPTION_LASER_TO_POINTCLOUD_H
#define PARSEC_PERCEPTION_LASER_TO_POINTCLOUD_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

namespace parsec_perception {

class LaserToPointCloudConverter : public nodelet::Nodelet {
 public:
  LaserToPointCloudConverter()
    : Nodelet() {}

 private:
  ros::Subscriber input_scan_subscriber_;
  ros::Publisher output_cloud_publisher_;

  virtual void onInit();
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &cloud);
};

}   // namespace parsec_perception

#endif  // PARSEC_PERCEPTION_LASER_TO_POINTCLOUD_H
