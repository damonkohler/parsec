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

#include "lightweight_navigation_visualization/tf_frame_map_pose_publisher.h"

#include <geometry_msgs/PoseStamped.h>

namespace lightweight_navigation_visualization {

const std::string TfFrameMapPosePublisher::kDefaultFrame = "base_link";
const std::string TfFrameMapPosePublisher::kDefaultMapOriginFrame = "map_origin";

TfFrameMapPosePublisher::TfFrameMapPosePublisher(
    const ros::NodeHandle &node_handle)
    : node_handle_(node_handle),
      publish_rate_(kDefaultPublishRate) {
  node_handle_.param("frame_id", frame_id_, kDefaultFrame);
  node_handle_.param("map_origin_frame_id", map_origin_frame_id_,
                     kDefaultMapOriginFrame);
  double publish_rate;
  if (node_handle_.getParam("publish_rate", publish_rate)) {
    publish_rate_ = ros::Rate(publish_rate);
  }
  map_subscriber_ = node_handle_.subscribe<nav_msgs::OccupancyGrid>(
      "/map", 10, boost::bind(&TfFrameMapPosePublisher::MapCallback, this, _1));
  pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
      "frame_pose", 10);
}

void TfFrameMapPosePublisher::Run() {
  while(ros::ok()) {
    publish_rate_.sleep();
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!current_map_) {
        ROS_WARN("No map received so far.");
        continue;
      }
    }
    
    ros::Time now = ros::Time::now();
    PublishMapOriginTransform(now);
    PublishFramePose(now);
  }
}

void TfFrameMapPosePublisher::MapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr &map) {
  boost::mutex::scoped_lock lock(mutex_);
  current_map_ = map;
}

void TfFrameMapPosePublisher::PublishMapOriginTransform(const ros::Time &time) {
  boost::mutex::scoped_lock lock(mutex_);
  tf::Pose origin;
  tf::poseMsgToTF(current_map_->info.origin, origin);
  tf::StampedTransform transform(
      origin, time,
      current_map_->header.frame_id,
      map_origin_frame_id_);
  tf_broadcaster_.sendTransform(transform);
}

void TfFrameMapPosePublisher::PublishFramePose(const ros::Time &time) {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = time;
  pose.header.frame_id = frame_id_;
  pose.pose.orientation.w = 1.0;
  geometry_msgs::PoseStamped transformed_pose;
  if (!tf_listener_.waitForTransform(
          map_origin_frame_id_, frame_id_, time, ros::Duration(0.2))) {
    ROS_WARN("Unable to transform frame into reference frame (%s -> %s).",
             frame_id_.c_str(), map_origin_frame_id_.c_str());
    return;
  }
  tf_listener_.transformPose(map_origin_frame_id_, pose, transformed_pose);
  pose_publisher_.publish(transformed_pose);
}

}  // namespace lightweight_navigation_visualization
