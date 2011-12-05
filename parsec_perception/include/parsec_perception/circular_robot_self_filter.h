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

#ifndef PARSEC_PERCEPTION_CIRCULAR_ROBOT_SELF_FILTER_H
#define PARSEC_PERCEPTION_CIRCULAR_ROBOT_SELF_FILTER_H

#include <string>

#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace parsec_perception {

class CircularRobotSelfFilter : public nodelet::Nodelet {
 public:
  CircularRobotSelfFilter()
    : Nodelet() {}

 private:
  /**
   * The base frame of the robot. Default: base_link
   */
  std::string base_frame_;
  /**
   * The radius of the robot. All points that are closer in the
   * x-y-plane are filtered out.
   */
  double radius_;
  /**
   * Minimal z value. All points below are not filtered.
   */
  double minimal_z_value_;
  /**
   * Maximal z value. All points above are not filtered.
   */
  double maximal_z_value_;
  ros::Subscriber input_cloud_subscriber_;
  ros::Publisher output_cloud_publisher_;
  tf::TransformListener tf_listener_;

  virtual void onInit();
  void CloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
};

}  // namespace parsec_perception

#endif  // PARSEC_PERCEPTION_CIRCULAR_ROBOT_SELF_FILTER_H
