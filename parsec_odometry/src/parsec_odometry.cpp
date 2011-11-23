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

#include "parsec_odometry/parsec_odometry.h"

#include <cmath>

#include <boost/bind.hpp>
#include <Eigen/Geometry>
#include <ros/ros.h>

namespace parsec_odometry {

const std::string ParsecOdometry::kDefaultBaseFrame = "base_link";
const std::string ParsecOdometry::kDefaultOdometryFrame = "odom";

ParsecOdometry::ParsecOdometry()
  : publish_tf_(false),
    minimal_odometry_rate_(kDefaultMinimalOdometryRate),
    base_frame_(kDefaultBaseFrame),
    odometry_frame_(kDefaultOdometryFrame),
    correction_transform_(tf::Transform::getIdentity()) {
}

ParsecOdometry::ParsecOdometry(const ros::NodeHandle &node_handle)
  : node_handle_(node_handle),
    tf_broadcaster_(),
    correction_transform_(tf::Transform::getIdentity()) {
  node_handle_.param("publish_tf", publish_tf_, kDefaultPublishTf);
  node_handle_.param("minimal_odometry_rate", minimal_odometry_rate_,
                     kDefaultMinimalOdometryRate);
  node_handle_.param("base_frame", base_frame_, kDefaultBaseFrame);
  node_handle_.param("odometry_frame", odometry_frame_, kDefaultOdometryFrame);
  parsec_odometry_subscriber_ = node_handle_.subscribe<parsec_msgs::Odometry>(
      "odom_simple", 10, boost::bind(&ParsecOdometry::ParsecOdometryCallback, this, _1));
  odometry_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom", 10);
}

void ParsecOdometry::ParsecOdometryCallback(
    const parsec_msgs::Odometry::ConstPtr &parsec_odometry) {
  nav_msgs::Odometry odometry;
  ParsecOdometryToOdometry(*parsec_odometry, &odometry);
  // If we see a time difference between subsequent odometry messages,
  // recalculate the correction transform.
  if (last_corrected_odometry_ &&
      !CompareOdometry(*last_corrected_odometry_, odometry, 1e-2, 1e-2) &&
      parsec_odometry->header.stamp - last_corrected_odometry_->header.stamp >
      ros::Duration(1 / minimal_odometry_rate_)) {
    ROS_INFO("Time difference between two succeeding odometry messages too big. "
             "Recalculating error correction.");
    CalculateCorrectionTransform(*last_corrected_odometry_, tf::Transform::getIdentity(),
                                 &correction_transform_);
    ROS_INFO("Calculated odometry offset transform: (%f, %f, %f), (%f, %f, %f, %f)",
             correction_transform_.getOrigin().x(),
             correction_transform_.getOrigin().y(),
             correction_transform_.getOrigin().z(),
             correction_transform_.getRotation().x(),
             correction_transform_.getRotation().y(),
             correction_transform_.getRotation().z(),
             correction_transform_.getRotation().w());
  }
  last_corrected_odometry_.reset(new nav_msgs::Odometry());
  CorrectOdometry(odometry, correction_transform_, last_corrected_odometry_.get());
  odometry_publisher_.publish(last_corrected_odometry_);
  if (publish_tf_) {
    tf::StampedTransform transform;
    OdometryToTransform(*last_corrected_odometry_, &transform);
    tf_broadcaster_.sendTransform(transform);
  }
}

void ParsecOdometry::OdometryToTransform(
    const nav_msgs::Odometry &odometry, tf::StampedTransform *transform) {
  transform->stamp_ = odometry.header.stamp;
  transform->frame_id_ = odometry.header.frame_id;
  transform->child_frame_id_ = odometry.child_frame_id;
  transform->setOrigin(
      btVector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y,
                odometry.pose.pose.position.z));
  transform->setRotation(
      btQuaternion(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,
                   odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w));
}

void ParsecOdometry::TransformToOdometry(
    const tf::StampedTransform &transform, nav_msgs::Odometry *odometry) {
  odometry->header.stamp = transform.stamp_;
  odometry->header.frame_id = transform.frame_id_;
  odometry->child_frame_id = transform.child_frame_id_;
  odometry->pose.pose.position.x = transform.getOrigin().x();
  odometry->pose.pose.position.y = transform.getOrigin().y();
  odometry->pose.pose.position.z = transform.getOrigin().z();
  odometry->pose.pose.orientation.x = transform.getRotation().x();
  odometry->pose.pose.orientation.y = transform.getRotation().y();
  odometry->pose.pose.orientation.z = transform.getRotation().z();
  odometry->pose.pose.orientation.w = transform.getRotation().w();
}

void ParsecOdometry::ParsecOdometryToOdometry(
    const parsec_msgs::Odometry &parsec_odometry, nav_msgs::Odometry *odometry) {
  odometry->header = parsec_odometry.header;
  odometry->header.frame_id = odometry_frame_;
  odometry->child_frame_id = base_frame_;
  odometry->pose.pose.position.x = parsec_odometry.position_x;
  odometry->pose.pose.position.y = parsec_odometry.position_y;
  odometry->pose.pose.orientation.z = parsec_odometry.orientation_z;
  odometry->pose.pose.orientation.w = parsec_odometry.orientation_w;
  odometry->twist.twist.linear.x = parsec_odometry.linear_x;
  odometry->twist.twist.linear.y = parsec_odometry.linear_y;
  odometry->twist.twist.angular.z = parsec_odometry.angular_z;
}

void ParsecOdometry::CorrectOdometry(
    const nav_msgs::Odometry &uncorrected_odometry, const tf::Transform &transform,
    nav_msgs::Odometry *odometry) {
  tf::StampedTransform odometry_transform;
  OdometryToTransform(uncorrected_odometry, &odometry_transform);
  tf::Transform corrected_odometry_transform = transform * odometry_transform;
  TransformToOdometry(
      tf::StampedTransform(corrected_odometry_transform, odometry_transform.stamp_,
                           odometry_transform.frame_id_, odometry_transform.child_frame_id_),
      odometry);
  // The twist didn't change, just copy it.
  odometry->twist = uncorrected_odometry.twist;
}

void ParsecOdometry::CalculateCorrectionTransform(
    const nav_msgs::Odometry &last_corrected_odometry,
    const tf::Transform &offset, tf::Transform *correction) {
  tf::StampedTransform last_corrected_odometry_transform;
  OdometryToTransform(last_corrected_odometry, &last_corrected_odometry_transform);
  // We use the following equation to calculate the correction factor:
  //
  //   last_odometry * correction = last_corrected_odometry * offset
  //
  // last_odometry is the identity transform because we assume that
  // the new odometry origin is exactly where stoped receiving
  // odometry messages.
  *correction = last_corrected_odometry_transform * offset;
}

bool ParsecOdometry::CompareOdometry(
    const nav_msgs::Odometry &lhs, const nav_msgs::Odometry &rhs,
    double max_linear_error, double max_angular_error) {
  return fabs(lhs.pose.pose.position.x - rhs.pose.pose.position.x) < max_linear_error &&
      fabs(lhs.pose.pose.position.y - rhs.pose.pose.position.y) < max_linear_error &&
      fabs(lhs.pose.pose.position.z - rhs.pose.pose.position.z) < max_linear_error &&
      fabs(lhs.pose.pose.orientation.x - rhs.pose.pose.orientation.x) < max_angular_error &&
      fabs(lhs.pose.pose.orientation.y - rhs.pose.pose.orientation.y) < max_angular_error &&
      fabs(lhs.pose.pose.orientation.z - rhs.pose.pose.orientation.z) < max_angular_error &&
      fabs(lhs.pose.pose.orientation.w - rhs.pose.pose.orientation.w) < max_angular_error;
}

}  // parsec_odometry
