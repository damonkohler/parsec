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

#include "cmd_vel_safety_filter/cmd_vel_safety_filter.h"

#include <cmath>

#include <laser_geometry/laser_geometry.h>
#include <ros_check/ros_check.h>

#include <sensor_msgs/PointCloud.h>

namespace cmd_vel_safety_filter {

const std::string CmdVelSafetyFilter::kDefaultBaseFrame = "base_link";

CmdVelSafetyFilter::CmdVelSafetyFilter(const ros::NodeHandle &node_handle)
    : node_handle_(node_handle) {
  if (!node_handle_.getParam("max_acceleration", max_acceleration_)) {
    ROS_FATAL("Required parameter not found: max_acceleration");
    ros::shutdown();
    return;
  }
  if (!node_handle_.getParam("radius", radius_)) {
    ROS_FATAL("Required parameter not found: radius");
    ros::shutdown();
    return;
  }
  node_handle_.param("stop_distance", stop_distance_, radius_);
  double scan_timeout;
  node_handle_.param("scan_timeout", scan_timeout, kDefaultScanTimeout);
  scan_timeout_ = ros::Duration(scan_timeout);
  node_handle_.param("base_frame", base_frame_, kDefaultBaseFrame);

  cmd_vel_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 10, boost::bind(&CmdVelSafetyFilter::CmdVelCallback, this, _1));
  scan_subscriber_ = node_handle_.subscribe<sensor_msgs::LaserScan>(
      "scan", 10,  boost::bind(&CmdVelSafetyFilter::ScanCallback, this, _1));
  cmd_vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>(
      "cmd_vel_filtered", 10);
}

void CmdVelSafetyFilter::CmdVelCallback(
    const geometry_msgs::Twist::ConstPtr &cmd_vel) {
  geometry_msgs::Twist filtered_cmd_vel;
  if (last_cloud_.size() == 0) {
    ROS_WARN("No laser scan received yet. Cannot filter.");
    filtered_cmd_vel = *cmd_vel;
  } else if (last_scan_reception_time_ + scan_timeout_ < ros::Time::now()) {
    ROS_WARN("Laser scan is older than %f seconds. Cannot filter.",
             scan_timeout_.toSec());
    filtered_cmd_vel = *cmd_vel;
  }
  if (!FilterCmdVel(*cmd_vel, last_cloud_, &filtered_cmd_vel)) {
    filtered_cmd_vel = *cmd_vel;
  }
  cmd_vel_publisher_.publish(filtered_cmd_vel);
}

void CmdVelSafetyFilter::ScanCallback(
    const sensor_msgs::LaserScan::ConstPtr &scan) {
  std::vector<tf::Point> cloud;
  if (ConvertLaserScan(*scan, base_frame_, &cloud)) {
    last_cloud_.swap(cloud);
    last_scan_reception_time_ = scan->header.stamp;
  }
}

bool CmdVelSafetyFilter::ConvertLaserScan(
    const sensor_msgs::LaserScan &scan, const std::string &base_frame,
    std::vector<tf::Point> *cloud) {
  laser_geometry::LaserProjection projection;
  sensor_msgs::PointCloud transformed_cloud;
  try {
    projection.transformLaserScanToPointCloud(
        base_frame, scan, transformed_cloud, tf_);
  } catch (tf::TransformException e) {
    ROS_WARN("Unable to transform laser scan from %s to %s",
             scan.header.frame_id.c_str(), base_frame.c_str());
    return false;
  }
  for (size_t i = 0; i < transformed_cloud.points.size(); i++) {
    tf::Point point;
    ToPoint(transformed_cloud.points[i], &point);
    cloud->push_back(point);
  }
  return true;
}

bool CmdVelSafetyFilter::FindClosestDistance(
    const tf::Point &point, const std::vector<tf::Point> &cloud,
    double *distance) {
  if (cloud.size() == 0) {
    return false;
  }
  *distance = point.distance(cloud[0]);
  for (size_t i = 1; i < cloud.size(); i++) {
    double current_distance = point.distance(cloud[i]);
    if (current_distance < *distance) {
      *distance = current_distance;
    }
  }
  return true;
}

bool CmdVelSafetyFilter::FilterCmdVel(
    const geometry_msgs::Twist &cmd_vel, const std::vector<tf::Point> &cloud,
    geometry_msgs::Twist *filtered_cmd_vel) {
  if (cloud.size() == 0) {
    ROS_WARN("No laser scan received yet. Cannot filter.");
    return false;
  }

  *filtered_cmd_vel = cmd_vel;
  tf::Point linear_velocity;
  ToPoint(cmd_vel.linear, &linear_velocity);
  std::vector<tf::Point> possible_obstacles;
  FindPointsInDirection(linear_velocity, radius_, cloud, &possible_obstacles);
  if (possible_obstacles.size() == 0) {
    return true;
  }

  double distance;
  CHECK(FindClosestDistance(tf::Point(0, 0, 0), possible_obstacles, &distance));

  // For extrapolation, we use the time needed to drive
  // kDefaultStopDistance.
  double extrapolation_time = stop_distance_ / linear_velocity.length();
  tf::Point extrapolated_position = linear_velocity * extrapolation_time;
  double extrapolated_distance;
  CHECK(FindClosestDistance(extrapolated_position, possible_obstacles,
                            &extrapolated_distance));
  // Only filter the velocity if extrapilation reduced the distance to
  // the closest obstacle.
  double slow_down_distance =
      linear_velocity.length2() / max_acceleration_ + stop_distance_;
  if (extrapolated_distance < distance &&
      distance < slow_down_distance) {
    double slope = 1 / (slow_down_distance - stop_distance_);
    double factor = fmax(slope * (distance - stop_distance_),
                         0.0);
    CHECK_LE(factor, 1);
    CHECK_GE(factor, 0);
    tf::Point filtered_linear_velocity = linear_velocity * factor;
    FromPoint(filtered_linear_velocity, &filtered_cmd_vel->linear);
  }
  return true;
}

void CmdVelSafetyFilter::FindPointsInDirection(
    btVector3 direction, double radius, std::vector<tf::Point> points,
    std::vector<tf::Point> *filtered_points) {

  btVector3 x_axis = btVector3(1, 0, 0);
  btVector3 axis = direction.cross(x_axis);
  double angle = direction.angle(x_axis);
  for (size_t i = 0; i < points.size(); i++) {
    tf::Point rotated_point = points[i].rotate(axis, angle);
    if (fabs(rotated_point.y()) <= radius &&
        fabs(rotated_point.z()) <= radius) {
      filtered_points->push_back(points[i]);
    }
  }
}

}  // namespace cmd_vel_safety_filter
