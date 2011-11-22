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

#ifndef PARSEC_ODOMETRY_PARSEC_ODOMETRY_H
#define PARSEC_ODOMETRY_PARSEC_ODOMETRY_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <parsec_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace parsec_odometry {

class ParsecOdometry {
 public:
  ParsecOdometry(const ros::NodeHandle &nh);

 private:
  static const double kDefaultMinimalOdometryRate = 2.0;
  static const bool kDefaultPublishTf = true;
  static const std::string kDefaultBaseFrame;
  static const std::string kDefaultOdometryFrame;

  bool publish_tf_;
  double minimal_odometry_rate_;
  std::string base_frame_;
  std::string odometry_frame_;
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_;
  ros::Subscriber parsec_odometry_subscriber_;
  ros::Subscriber laser_subscriber_;
  ros::Publisher odometry_publisher_;
  nav_msgs::Odometry::Ptr last_odometry_;
  nav_msgs::Odometry::Ptr last_corrected_odometry_;
  sensor_msgs::PointCloud2::ConstPtr last_valid_laser_cloud_;
  boost::shared_ptr<tf::Transform> correction_transform_;

  void ParsecOdometryCallback(const parsec_msgs::Odometry::ConstPtr &parsec_odometry);
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan);
  void OdometryToTransform(const nav_msgs::Odometry &odometry, tf::StampedTransform *transform);
  void TransformToOdometry(const tf::StampedTransform &transform, nav_msgs::Odometry *odometry);
  void ParsecOdometryToOdometry(const parsec_msgs::Odometry &parsec_odometry,
                                nav_msgs::Odometry *odometry);
  void CorrectOdometry(
      const nav_msgs::Odometry &uncorrected_odometry, const tf::Transform &transform,
      nav_msgs::Odometry *odometry);
  void CalculateCorrectionTransform(const nav_msgs::Odometry &last_odometry,
                                    const nav_msgs::Odometry &last_corrected_odometry,
                                    const tf::Transform &laser_transform,
                                    tf::Transform *correction);
  bool CalculateLaserCorrectionTransform(const sensor_msgs::PointCloud2 &old_cloud_msg,
                                         const sensor_msgs::PointCloud2 &new_cloud_msg,
                                         tf::Transform *transform);
};

}  // parsec_odometry

#endif  // PARSEC_ODOMETRY_PARSEC_ODOMETRY_H
