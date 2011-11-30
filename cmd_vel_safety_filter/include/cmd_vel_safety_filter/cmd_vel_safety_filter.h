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

#ifndef CMD_VEL_SAFETY_FILTER_CMD_VEL_SAFETY_FILTER_H
#define CMD_VEL_SAFETY_FILTER_CMD_VEL_SAFETY_FILTER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

namespace cmd_vel_safety_filter {

class CmdVelSafetyFilter {
 public:
  CmdVelSafetyFilter(const ros::NodeHandle &node_handle);

  /**
   * Computes and applies a slow-down factor if a point in cloud is too close.
   *
   * Public for testing.
   */
  bool FilterCmdVel(
      const geometry_msgs::Twist &cmd_vel, const std::vector<tf::Point> &cloud,
      geometry_msgs::Twist *filtered_cmd_vel);

  /**
   * Finds all pionts that might come close than radius when we move
   * along direction.
   *
   * Public for testing.
   */
  void FindPointsInDirection(
      btVector3 direction, double radius, std::vector<tf::Point> points,
      std::vector<tf::Point> *filtered_points);
 private:
  static const double kDefaultScanTimeout = 1.0;
  static const std::string kDefaultBaseFrame;

  ros::NodeHandle node_handle_;
  tf::TransformListener tf_;
  double max_acceleration_;
  double radius_;
  double stop_distance_;
  ros::Duration scan_timeout_;
  std::string base_frame_;
  ros::Subscriber cmd_vel_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Publisher cmd_vel_publisher_;
  ros::Time last_scan_reception_time_;
  std::vector<tf::Point> last_cloud_;

  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
  bool ConvertLaserScan(
      const sensor_msgs::LaserScan &scan, const std::string &base_frame,
      std::vector<tf::Point> *cloud);
  bool FindClosestDistance(
      const tf::Point &point, const std::vector<tf::Point> &cloud, double *distance);

  template<typename T>
  void ToPoint(const T &point_in, tf::Point *point) {
    *point = tf::Point(point_in.x, point_in.y, point_in.z);
  }
  template<typename T>
  void FromPoint(const tf::Point &point_in, T *point) {
    point->x = point_in.x();
    point->y = point_in.y();
    point->z = point_in.z();
  }
};

}  // namespace cmd_vel_safety_filter

#endif  // CMD_VEL_SAFETY_FILTER_CMD_VEL_SAFETY_FILTER_H
