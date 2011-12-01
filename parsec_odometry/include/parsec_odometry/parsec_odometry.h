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

#include <nav_msgs/Odometry.h>
#include <parsec_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace parsec_odometry {

class ParsecOdometry {
 public:
  /**
   * Default constructor to instantiate without using ros. Mainly
   * useful for testing.
   */
  ParsecOdometry();

  ParsecOdometry(const ros::NodeHandle &node_handle);

  /**
   * Apply a correction transform to an odometry message.
   *
   * Public for testing.
   */
  void CorrectOdometry(
      const nav_msgs::Odometry &uncorrected_odometry, const tf::Transform &transform,
      nav_msgs::Odometry *odometry);

  /**
   * Calculates the correction transform to be applied to an odometry
   * message to correct for errors introduced by restarts of the micro
   * controller. This function also takes an offset form the last
   * corrected odometry to calculate the new correction. This offset
   * usually comes from laser based ICP to correct for small pose
   * errors introduced by e.g. exchanging batteries.
   *
   * Public for testing.
   *
   * @param last_corrected_odometry the last correct odometry message
   * @param offset offset of last_corrected_odometry to the real pose
   * @param correction the new correction tranform
   */
  void CalculateCorrectionTransform(const nav_msgs::Odometry &last_corrected_odometry,
                                    const tf::Transform &offset,
                                    tf::Transform *correction);

 private:
  static const bool kDefaultPublishTf = true;
  static const double kDefaultMinimalOdometryRate = 2.0;
  static const std::string kDefaultBaseFrame;
  static const std::string kDefaultOdometryFrame;

  bool publish_tf_;
  double minimal_odometry_rate_;  
  std::string base_frame_;
  std::string odometry_frame_;
  ros::NodeHandle node_handle_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber parsec_odometry_subscriber_;
  ros::Publisher odometry_publisher_;
  nav_msgs::Odometry::Ptr last_corrected_odometry_;
  tf::Transform correction_transform_;

  void ParsecOdometryCallback(const parsec_msgs::Odometry::ConstPtr &parsec_odometry);
  void OdometryToTransform(const nav_msgs::Odometry &odometry, tf::StampedTransform *transform);
  void TransformToOdometry(const tf::StampedTransform &transform, nav_msgs::Odometry *odometry);
  void ParsecOdometryToOdometry(const parsec_msgs::Odometry &parsec_odometry,
                                nav_msgs::Odometry *odometry);
  /**
   * Returns if the odometry message is (almost) zero, i.e. has an
   * identity rotation and na identity vector.
   */
  bool OdometryIsZero(const nav_msgs::Odometry &odometry);
};

}  // parsec_odometry

#endif  // PARSEC_ODOMETRY_PARSEC_ODOMETRY_H
