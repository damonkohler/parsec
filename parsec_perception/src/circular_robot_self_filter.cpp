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

#include "parsec_perception/circular_robot_self_filter.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>

namespace parsec_perception {

void CircularRobotSelfFilter::onInit() {
  PCLNodelet::onInit();

  pnh_->param("base_frame", base_frame_, std::string("base_link"));
  if (!pnh_->getParam("radius", radius_)) {
    ROS_FATAL("Parameter 'radius' not found.");
    return;
  }
  if (!pnh_->getParam("minimal_z_value", minimal_z_value_)) {
    ROS_FATAL("Parameter 'minimal_z_value' not found.");
    return;
  }
  if (!pnh_->getParam("maximal_z_value", maximal_z_value_)) {
    ROS_FATAL("Parameter 'maximal_z_value' not found.");
    return;
  }
  input_cloud_subscriber_ = pnh_->subscribe<pcl::PointCloud<pcl::PointXYZ> >(
      "input", 100, boost::bind(&CircularRobotSelfFilter::CloudCallback, this, _1));
  output_cloud_publisher_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "output", 100);
}

void CircularRobotSelfFilter::CloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    pcl_ros::transformPointCloud(base_frame_, *cloud, *transformed_cloud, tf_listener_);
  } catch (tf::TransformException e) {
    // Transformation fails in particular at start up because tilting
    // laser transforms might not be coming in yet. This is logged by
    // TF already, so we don't add another logging here.
    return;
  }

  boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
  for (size_t i = 0; i < transformed_cloud->points.size(); i++) {
    pcl::PointXYZ point = transformed_cloud->points[i];
    pcl::PointXYZ origin_xy(0, 0, transformed_cloud->points[i].z);
    if (point.z <= maximal_z_value_ && point.z >= minimal_z_value_ &&
        pcl::euclideanDistance(point, origin_xy) <= radius_) {
      continue;
    }
    indices->push_back(i);
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  extract_indices.setIndices(indices);
  extract_indices.setInputCloud(transformed_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  extract_indices.filter(*output_cloud);
  output_cloud_publisher_.publish(output_cloud);
}

}  // namespace parsec_perception

PLUGINLIB_DECLARE_CLASS(parsec_perception, CircularRobotSelfFilter, parsec_perception::CircularRobotSelfFilter, nodelet::Nodelet);
