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
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/ros/conversions.h>
#include <ros/ros.h>

namespace parsec_odometry {

const std::string ParsecOdometry::kDefaultBaseFrame = "base_link";
const std::string ParsecOdometry::kDefaultOdometryFrame = "odom";

static void MatrixToTransfrom(const Eigen::Matrix4f &matrix, tf::Transform *transform) {
  Eigen::Transform<float, 3, Eigen::Affine> eigen_transform(matrix);
  transform->setOrigin(
      btVector3(eigen_transform.translation()(0), eigen_transform.translation()(1),
                eigen_transform.translation()(2)));
  Eigen::Quaternion<float> rotation(eigen_transform.rotation());
  transform->setRotation(
    btQuaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));
}

ParsecOdometry::ParsecOdometry()
  : publish_tf_(false),
    minimal_odometry_rate_(kDefaultMinimalOdometryRate),
    base_frame_(kDefaultBaseFrame),
    odometry_frame_(kDefaultOdometryFrame),
    correction_transform_(new tf::Transform(tf::Transform::getIdentity())) {
}

ParsecOdometry::ParsecOdometry(const ros::NodeHandle &nh)
  : nh_(nh),
    tf_broadcaster_(),
    tf_ (nh_),
    correction_transform_(new tf::Transform(tf::Transform::getIdentity())) {
  nh_.param("publish_tf", publish_tf_, kDefaultPublishTf);
  nh_.param("minimal_odometry_rate", minimal_odometry_rate_, kDefaultMinimalOdometryRate);
  nh_.param("base_frame", base_frame_, kDefaultBaseFrame);
  nh_.param("odometry_frame", odometry_frame_, kDefaultOdometryFrame);
  parsec_odometry_subscriber_ = nh_.subscribe<parsec_msgs::Odometry>(
      "odom_simple", 10, boost::bind(&ParsecOdometry::ParsecOdometryCallback, this, _1));
  laser_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan>(
      "scan", 10, boost::bind(&ParsecOdometry::LaserCallback, this, _1));
  odometry_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
}

void ParsecOdometry::ParsecOdometryCallback(
    const parsec_msgs::Odometry::ConstPtr &parsec_odometry) {
  nav_msgs::Odometry odometry;
  last_odometry_.reset(new nav_msgs::Odometry());
  ParsecOdometryToOdometry(*parsec_odometry, last_odometry_.get());
  if (!correction_transform_) {
    return;
  }
  // If we are getting laser scans and we see a time difference
  // between subsequent odometry messages, invalidate the correction
  // transform to recalculate it.
  if (last_corrected_odometry_ && last_valid_laser_cloud_ &&
      parsec_odometry->header.stamp - last_corrected_odometry_->header.stamp >
      ros::Duration(1 / minimal_odometry_rate_)) {
    correction_transform_.reset();
    ROS_INFO("Time difference between two succeeding odometry messages too big. "
             "Recalculating error correction.");
    return;
  }
  last_corrected_odometry_.reset(new nav_msgs::Odometry());
  CorrectOdometry(*last_odometry_, *correction_transform_, last_corrected_odometry_.get());
  odometry_publisher_.publish(last_corrected_odometry_);
  if (publish_tf_) {
    tf::StampedTransform transform;
    OdometryToTransform(*last_corrected_odometry_, &transform);
    tf_broadcaster_.sendTransform(transform);
  }
}

void ParsecOdometry::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan) {
  laser_geometry::LaserProjection laser_projection;

  sensor_msgs::PointCloud2 scan_cloud;
  try {
    laser_projection.transformLaserScanToPointCloud(base_frame_, *laser_scan, scan_cloud, tf_);
  } catch (tf::TransformException &e) {
    ROS_WARN("Couldn't transform laser scan into base frame: %s", base_frame_.c_str());
    return;
  }

  if (!last_odometry_ || !last_corrected_odometry_) {
    return;
  }
  if (!last_valid_laser_cloud_) {
    // Only initialize the last scan if it is approximately as old as the
    // last odometry message we received.
    if (last_corrected_odometry_->header.stamp - scan_cloud.header.stamp <
        ros::Duration(1 / minimal_odometry_rate_)) {
      last_valid_laser_cloud_ = sensor_msgs::PointCloud2::Ptr(
          new sensor_msgs::PointCloud2(scan_cloud));
    }
    return;
  }
  if (!correction_transform_ && last_valid_laser_cloud_) {
    tf::Transform laser_transform;
    if (!CalculateLaserCorrectionTransform(
          *last_valid_laser_cloud_, scan_cloud, &laser_transform)) {
      ROS_WARN("Unable to calculate laser scan transform.");
      return;
    }
    tf::Transform new_correction_transform;
    CalculateCorrectionTransform(*last_odometry_, *last_corrected_odometry_, laser_transform,
                                 &new_correction_transform);
    ROS_INFO("Calculated laser scan transform: (%f, %f, %f), (%f, %f, %f, %f)",
             new_correction_transform.getOrigin().x(),
             new_correction_transform.getOrigin().y(),
             new_correction_transform.getOrigin().z(),
             new_correction_transform.getRotation().x(),
             new_correction_transform.getRotation().y(),
             new_correction_transform.getRotation().z(),
             new_correction_transform.getRotation().w());
    correction_transform_.reset(new tf::Transform(new_correction_transform));
    // Clear the old odometry to make the callback publish odometry
    // again.
    last_corrected_odometry_.reset();
    return;
  }
  if (fabs((last_valid_laser_cloud_->header.stamp - last_corrected_odometry_->header.stamp).toSec()) >
      fabs((scan_cloud.header.stamp - last_corrected_odometry_->header.stamp).toSec())) {
    last_valid_laser_cloud_ = sensor_msgs::PointCloud2::Ptr(
        new sensor_msgs::PointCloud2(scan_cloud));
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
  tf::Transform corrected_odometry_transform = odometry_transform * transform;
  TransformToOdometry(
      tf::StampedTransform(corrected_odometry_transform, odometry_transform.stamp_,
                           odometry_transform.frame_id_, odometry_transform.child_frame_id_),
      odometry);
  // The twist didn't change, just copy it.
  odometry->twist = uncorrected_odometry.twist;
}

void ParsecOdometry::CalculateCorrectionTransform(
    const nav_msgs::Odometry &last_odometry,
    const nav_msgs::Odometry &last_corrected_odometry,
    const tf::Transform &offset, tf::Transform *correction) {
  tf::StampedTransform last_odometry_transform;
  OdometryToTransform(last_odometry, &last_odometry_transform);
  tf::StampedTransform last_corrected_odometry_transform;
  OdometryToTransform(last_corrected_odometry, &last_corrected_odometry_transform);
  // We use the following equation to calculate the correction factor:
  // last_odometry * correction = last_corrected_odometry * offset;
  *correction = last_odometry_transform.inverse() *
      (last_corrected_odometry_transform * offset);
}

bool ParsecOdometry::CalculateLaserCorrectionTransform(
    const sensor_msgs::PointCloud2 &old_cloud_msg,
    const sensor_msgs::PointCloud2 &new_cloud_msg,
    tf::Transform *transform) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(old_cloud_msg, *old_cloud);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(new_cloud_msg, *new_cloud);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(50);
  icp.setInputCloud(old_cloud);
  icp.setInputTarget(new_cloud);

  // We actually don't use the variable new_cloud_in_old but icp.align
  // needs it.
  pcl::PointCloud<pcl::PointXYZ> new_cloud_in_old;
  icp.align(new_cloud_in_old);

  if (!icp.hasConverged()) {
    return false;
  }
  btTransform icp_result;
  MatrixToTransfrom(icp.getFinalTransformation(), &icp_result);

  // In some rare cases the ICP algorithm also returns a z offset
  // which is definitely wrong because the robot only moves in 2D. To
  // remove the z offset again and rotations around the x and y axis,
  // we first set rotations around x and y to zero. Then we project
  // the transform back to the x-y-plane by setting z to 0 and
  // changing the x and y components to keep the length of the
  // original origin vector and the new one equal.
  double angle_x, angle_y, angle_z;
  icp_result.getBasis().getEulerYPR(angle_z, angle_y, angle_x);
  btMatrix3x3 x_y_rotation;
  x_y_rotation.setEulerYPR(0.0, angle_y, angle_x);
  *transform = icp_result * btTransform(x_y_rotation).inverse();
  double x_y_normalization_factor = icp_result.getOrigin().length() /
      btVector3(icp_result.getOrigin().x(), icp_result.getOrigin().y(), 0).length();
  transform->setOrigin(
      btVector3(icp_result.getOrigin().x() * x_y_normalization_factor,
                icp_result.getOrigin().y() * x_y_normalization_factor,
                0));

  return true;
}

}  // parsec_odometry
