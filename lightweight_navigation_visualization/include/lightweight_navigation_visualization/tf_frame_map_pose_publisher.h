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

#ifndef LIGHTWEIGHT_NAVIGATION_VISUALIZATION_TF_FRAME_MAP_POSE_PUBLISHER_H
#define LIGHTWEIGHT_NAVIGATION_VISUALIZATION_TF_FRAME_MAP_POSE_PUBLISHER_H

#include <string>

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>

namespace lightweight_navigation_visualization {

class TfFrameMapPosePublisher {
 public:
  TfFrameMapPosePublisher(
      const ros::NodeHandle &node_handle);
  void Run();

 private:
  static const std::string kDefaultFrame;
  static const std::string kDefaultMapOriginFrame;
  static const double kDefaultPublishRate = 10;

  ros::NodeHandle node_handle_;
  std::string frame_id_;
  std::string map_origin_frame_id_;
  ros::Rate publish_rate_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber map_subscriber_;
  ros::Publisher pose_publisher_;
  nav_msgs::OccupancyGrid::ConstPtr current_map_;
  boost::mutex mutex_;

  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
  void PublishMapOriginTransform(const ros::Time &time);
  void PublishFramePose(const ros::Time &time);
};

}  // namespace lightweight_navigation_visualization

#endif  // LIGHTWEIGHT_NAVIGATION_VISUALIZATION_TF_FRAME_MAP_POSE_PUBLISHER_H
