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

#include "lightweight_navigation_visualization/map_pose_to_pose_stamped.h"

#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>

namespace lightweight_navigation_visualization {

MapPoseToPoseStamped::MapPoseToPoseStamped(
    const ros::NodeHandle &node_handle)
    : node_handle_(node_handle) {
  map_subscriber_ = node_handle_.subscribe<nav_msgs::OccupancyGrid>(
      "/map", 10, boost::bind(&MapPoseToPoseStamped::MapCallback, this, _1));
  pose_subscriber_ = node_handle_.subscribe<geometry_msgs::Pose>(
      "map_pose", 10, boost::bind(&MapPoseToPoseStamped::PoseCallback, this, _1));
  pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
      "pose_stamped", 10);
}

void MapPoseToPoseStamped::MapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr &map) {
  boost::mutex::scoped_lock lock(mutex_);
  current_map_ = map;
}

void MapPoseToPoseStamped::PoseCallback(
    const geometry_msgs::Pose::ConstPtr &pose_message) {
  boost::mutex::scoped_lock lock(mutex_);
  if (!current_map_) {
    ROS_WARN("No map received so far. Cannot transform.");
    return;
  }
  tf::Pose pose;
  tf::poseMsgToTF(*pose_message, pose);
  tf::Pose map_origin;
  tf::poseMsgToTF(current_map_->info.origin, map_origin);
  geometry_msgs::PoseStamped transformed_pose;
  tf::poseTFToMsg(map_origin * pose, transformed_pose.pose);
  transformed_pose.header.stamp = ros::Time::now();
  transformed_pose.header.frame_id = current_map_->header.frame_id;
  pose_publisher_.publish(transformed_pose);
}

}  // namespace lightweight_navigation_visualization
