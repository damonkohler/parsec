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

#include <geometry_msgs/Pose.h>

namespace lightweight_navigation_visualization {

const std::string TfFrameMapPosePublisher::kDefaultReferenceFrameId = "map";
const std::string TfFrameMapPosePublisher::kDefaultFrameId = "base_link";

TfFrameMapPosePublisher::TfFrameMapPosePublisher(
    const ros::NodeHandle &node_handle)
    : node_handle_(node_handle),
      publish_rate_(kDefaultPublishRate) {
  node_handle_.param(
      "reference_frame_id", reference_frame_id_, kDefaultReferenceFrameId);
  node_handle_.param("frame_id", frame_id_, kDefaultFrameId);
  double publish_rate;
  if (node_handle_.getParam("publish_rate", publish_rate)) {
    publish_rate_ = ros::Rate(publish_rate);
  }
  map_subscriber_ = node_handle_.subscribe<nav_msgs::OccupancyGrid>(
      "/map", 10, boost::bind(&TfFrameMapPosePublisher::MapCallback, this, _1));
  pose_publisher_ = node_handle_.advertise<geometry_msgs::Pose>(
      "frame_pose", 10);
}

void TfFrameMapPosePublisher::Run() {
  while(ros::ok()) {
    publish_rate_.sleep();
    boost::mutex::scoped_lock lock(mutex_);
    if (!current_map_) {
      ROS_WARN("No map received so far.");
      continue;
    }
    ros::Time now = ros::Time::now();
    if (!tf_listener_.waitForTransform(reference_frame_id_, frame_id_, now,
                                       ros::Duration(0.2))) {
      ROS_WARN("Unable to transform frame into reference frame (%s -> %s).",
               frame_id_.c_str(), reference_frame_id_.c_str());
      continue;
    }
    tf::Stamped<tf::Pose> pose;
    pose.stamp_ = now;
    pose.frame_id_ = frame_id_;
    pose.setIdentity();
    tf::Stamped<tf::Pose> transformed_pose;
    tf_listener_.transformPose(reference_frame_id_, pose, transformed_pose);
    tf::Pose map_origin;
    tf::poseMsgToTF(current_map_->info.origin, map_origin);
    tf::Pose pose_in_map_origin = transformed_pose * map_origin.inverse();
    geometry_msgs::Pose pose_in_map_origin_msg;
    tf::poseTFToMsg(pose_in_map_origin, pose_in_map_origin_msg);
    pose_publisher_.publish(pose_in_map_origin_msg);
  }
}

void TfFrameMapPosePublisher::MapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr &map) {
  boost::mutex::scoped_lock lock(mutex_);
  current_map_ = map;
}

}  // namespace lightweight_navigation_visualization
