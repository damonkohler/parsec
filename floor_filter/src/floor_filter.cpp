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

#include <cmath>

#include <algorithm>
#include <iterator>

#include <boost/bind.hpp>
#include <boost/timer.hpp>

#include <laser_geometry/laser_geometry.h>
#include <pcl/ros/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Geometry>
#include <pluginlib/class_list_macros.h>

#include "floor_filter/floor_filter.h"

namespace floor_filter {

/**
 * Intersects two planes and returns the intersecting line. If the
 * calculation fails, i.e. if the two planes are paralle, return
 * false.
 */
static bool IntersectPlanes(
    const Eigen::Hyperplane<float, 3> &plane1,
    const Eigen::Hyperplane<float, 3> &plane2,
    Eigen::ParametrizedLine<float, 3> *intersection) {
  Eigen::ParametrizedLine<float, 3>::VectorType direction =
      plane1.normal().cross(plane2.normal());
  // When planes are parallel, i.e. the cross product is close to 0,
  // return false.
  if (direction.norm() < 1e-6) {
    return false;
  }

  // Calculate the intersection of two planes using the formulas as,
  // for instance, found at http://paulbourke.net/geometry/planeplane/.
  Eigen::Hyperplane<float, 3>::Scalar n1_n1  =
      (plane1.normal() * plane1.normal().transpose())(0);
  Eigen::Hyperplane<float, 3>::Scalar n2_n2  =
      (plane2.normal() * plane2.normal().transpose())(0);
  Eigen::Hyperplane<float, 3>::Scalar n1_n2  =
      (plane1.normal() * plane2.normal().transpose())(0);
  Eigen::Hyperplane<float, 3>::Scalar determinant =
      n1_n1 * n2_n2 - (n1_n2 * n1_n2);
  
  Eigen::ParametrizedLine<float, 3>::VectorType origin =
      Eigen::ParametrizedLine<float, 3>::VectorType(
        (plane1.offset() * n2_n2 - plane2.offset() * n1_n2) / determinant,
        (plane2.offset() * n1_n1 - plane1.offset() * n1_n2) / determinant,
        0.0);
  *intersection = Eigen::ParametrizedLine<float, 3>(origin, direction);
  return true;
}

void FloorFilter::onInit() {
  PCLNodelet::onInit();
  
  if (!pnh_->getParam("sensor_frame", sensor_frame_)) {
    ROS_FATAL("Parameter 'sensor_frame' not found");
    return;
  }
  pnh_->param("reference_frame", reference_frame_, std::string("odom"));
  pnh_->param("floor_z_distance", floor_z_distance_, 0.05);
  double max_slope_angle;
  pnh_->param("max_slope_angle", max_slope_angle,
              static_cast<double>(2.0 * M_PI/180.0));
  max_slope_ = tan(max_slope_angle);
  pnh_->param("ransac_distance_threshold", ransac_distance_threshold_, 0.03);
  pnh_->param("cliff_distance_threshold", cliff_distance_threshold_, 0.03);
      
  input_scan_subscriber_ = pnh_->subscribe<sensor_msgs::LaserScan>(
      "scan", 100, boost::bind(&FloorFilter::LaserCallback, this, _1));
  input_cloud_subscriber_ = pnh_->subscribe<pcl::PointCloud<pcl::PointXYZ> >(
      "cloud_in", 100, boost::bind(&FloorFilter::CloudCallback, this, _1));
  floor_cloud_publisher_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "floor_cloud", 10);
  filtered_cloud_publisher_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "filtered_cloud", 10);
  cliff_cloud_publisher_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "cliff_cloud", 10);
  cliff_generating_cloud_publisher_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "cliff_generating_cloud", 10);
}

void FloorFilter::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  laser_geometry::LaserProjection laser_projection;
  sensor_msgs::PointCloud2 cloud_msg;
  laser_projection.projectLaser(*scan, cloud_msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *cloud);
  CloudCallback(cloud);
}

void FloorFilter::CloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ros::transformPointCloud(reference_frame_, *cloud, *transformed_cloud, tf_listener_);
  if(!transformed_cloud->points.size()) {
    ROS_WARN("No input points");
    return;
  }
  std::vector<int> all_indices(transformed_cloud->points.size());
  for (size_t i = 0; i < all_indices.size(); i++) {
    all_indices[i] = i;
  }
  
  std::vector<int> floor_candidate_indices;
  FilterFloorCandidates(floor_z_distance_, max_slope_, *transformed_cloud,
                        &floor_candidate_indices);
  std::sort(floor_candidate_indices.begin(), floor_candidate_indices.end());

  Eigen::ParametrizedLine<float, 3> line;
  std::vector<int> line_inlier_indices;
  if (!FindFloorLine(transformed_cloud, floor_candidate_indices, &line, &line_inlier_indices)) {
    Eigen::Hyperplane<float, 3> sensor_plane = GetSensorPlane(transformed_cloud->header.stamp);
    // Use intersection of the x-y-plane and the sensor plane to
    // generate an artificial floor line.
    if (!IntersectPlanes(sensor_plane,
                         Eigen::Hyperplane<float, 3>(
                             Eigen::Hyperplane<float, 3>::VectorType(0, 0, 1),
                             0.0),
                         &line)) {
      // If the sensor plane and the floor plane happen to be
      // parallel, just re-publish the input cloud because we cannot
      // see the floor anyway.
      filtered_cloud_publisher_.publish(transformed_cloud);
      return;
    }
    ROS_INFO("No floor line found. Using the intersection between the sensor plane and the x-y-plain.");
  }
  std::sort(line_inlier_indices.begin(), line_inlier_indices.end());
  
  std::vector<int> indices_without_floor;
  std::set_difference(all_indices.begin(), all_indices.end(), line_inlier_indices.begin(), line_inlier_indices.end(),
                      std::back_insert_iterator<std::vector<int> >(indices_without_floor));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cliff_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> cliff_indices;
  GenerateCliffCloud(line, transformed_cloud, indices_without_floor,
                     cliff_cloud.get(), &cliff_indices);
  std::sort(cliff_indices.begin(), cliff_indices.end());

  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  MakeCloudFromIndices(*transformed_cloud, line_inlier_indices, floor_cloud.get());

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  MakeCloudFromIndices(*transformed_cloud, indices_without_floor, filtered_cloud.get());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cliff_generating_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  MakeCloudFromIndices(*transformed_cloud, cliff_indices, cliff_generating_cloud.get());


  if (floor_cloud->points.size()) {
    floor_cloud_publisher_.publish(floor_cloud);
  }
  if (filtered_cloud->points.size()) {
    ros::Duration message_age = ros::Time::now() - filtered_cloud->header.stamp;
    // This is just a hint. Throw a warning to make the user know
    // about something being fishy with the current configuration
    // because input data is pretty old.
    if (message_age > ros::Duration(1.0)) {
      ROS_WARN("Filtered cloud already %lf seconds old.", message_age.toSec());
    }
    filtered_cloud_publisher_.publish(filtered_cloud);
  }
  if (cliff_cloud->points.size()) {
    cliff_cloud_publisher_.publish(cliff_cloud);
  }
  if (cliff_generating_cloud->points.size()) {
    cliff_generating_cloud_publisher_.publish(cliff_generating_cloud);
  }
}

void FloorFilter::FilterFloorCandidates(
    double floor_z_distance, double max_slope, pcl::PointCloud<pcl::PointXYZ> &cloud,
    std::vector<int> *indices) {
  for (size_t i = 0; i < cloud.points.size(); i++) {
    double point_distance_xy = sqrt(cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y);
    if (fabs(cloud.points[i].z) < floor_z_distance + max_slope_ * point_distance_xy) {
      indices->push_back(i);
    }
  }
}

bool FloorFilter::FindFloorLine(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
    const std::vector<int> &indices,
    Eigen::ParametrizedLine<float, 3> *line,
    std::vector<int> *inlier_indices) {
  if (indices.size() == 0) {
    return false;
  }
  pcl::SACSegmentation<pcl::PointXYZ> ransac_line_finder;
  
  ransac_line_finder.setOptimizeCoefficients(true);
  ransac_line_finder.setModelType(pcl::SACMODEL_LINE);
  ransac_line_finder.setMethodType(pcl::SAC_RANSAC);
  ransac_line_finder.setDistanceThreshold(ransac_distance_threshold_);
  ransac_line_finder.setMaxIterations(100);

  ransac_line_finder.setInputCloud(input_cloud);
  pcl::PointIndicesPtr point_input_indices(new pcl::PointIndices);
  point_input_indices->indices.resize(indices.size());
  std::copy(indices.begin(), indices.end(), point_input_indices->indices.begin());
  ransac_line_finder.setIndices(point_input_indices);

  pcl::PointIndices point_inlier_indices;
  pcl::ModelCoefficients line_coefficients;
  ransac_line_finder.segment(point_inlier_indices, line_coefficients);

  if (point_inlier_indices.indices.size() == 0) {
    ROS_WARN("Could not estimate a line model for the given dataset.");
    return false;
  }

  *line = LineFromCoefficients(line_coefficients);
  inlier_indices->resize(point_inlier_indices.indices.size());
  std::copy(point_inlier_indices.indices.begin(),  point_inlier_indices.indices.end(),
            inlier_indices->begin());

  return true;
}

void FloorFilter::GenerateCliffCloud(
    const Eigen::ParametrizedLine<float, 3> &line_coefficients,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
    const std::vector<int> &input_indices,
    pcl::PointCloud<pcl::PointXYZ> *cliff_cloud,
    std::vector<int> *cliff_indices) {
  cliff_cloud->header = input_cloud->header;
  cliff_cloud->height = 1;
  cliff_cloud->is_dense = false;
  cliff_cloud->points.clear();

  pcl::PointXYZ viewpoint = GetViewpointPoint(input_cloud->header.stamp);
  for (size_t i = 0; i < input_indices.size(); i++) {
    pcl::PointXYZ projected_cliff_point = ProjectPointOnLine(
        line_coefficients, input_cloud->points[i]);
    if (EuclideanDistance(viewpoint, input_cloud->points[i]) >
        EuclideanDistance(viewpoint, projected_cliff_point) + cliff_distance_threshold_) {
      cliff_indices->push_back(i);
      cliff_cloud->points.push_back(projected_cliff_point);
    }
  }
  cliff_cloud->width = cliff_cloud->points.size();
}

void FloorFilter::MakeCloudFromIndices(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
                                       const std::vector<int> &input_indices,
                                       pcl::PointCloud<pcl::PointXYZ> *output_cloud) {
  output_cloud->header = input_cloud.header;
  output_cloud->width = input_indices.size();
  output_cloud->height = 1;
  output_cloud->is_dense = false;
  output_cloud->points.resize(input_indices.size());

  for (size_t i = 0; i < input_indices.size(); i++) {
    output_cloud->points[i] = input_cloud.points[input_indices[i]];
  }
}

double FloorFilter::PointToLineDistance(const Eigen::ParametrizedLine<float, 3> &line, const pcl::PointXYZ &point) {
  return sqrt(line.squaredDistance(point.getVector3fMap()));
}

double FloorFilter::EuclideanDistance(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2) {
  float x = point1.x - point2.x;
  float y = point1.y - point2.y;
  float z = point1.z - point2.z;
  return sqrt(x * x + y * y + z * z);
}

pcl::PointXYZ FloorFilter::ProjectPointOnLine(const Eigen::ParametrizedLine<float, 3> &line, const pcl::PointXYZ &point) {
  Eigen::ParametrizedLine<float, 3>::VectorType projected_point = line.projection(point.getVector3fMap());
  return pcl::PointXYZ(projected_point[0], projected_point[1], projected_point[2]);
}

pcl::PointXYZ FloorFilter::GetViewpointPoint(const ros::Time &time) {
  tf::Stamped<tf::Point> sensor_point;
  tf::Stamped<tf::Point> viewpoint;
  sensor_point.frame_id_ = sensor_frame_;
  sensor_point.stamp_ = time;
  tf_listener_.transformPoint(reference_frame_, sensor_point, viewpoint);
  return pcl::PointXYZ(viewpoint.x(), viewpoint.y(), viewpoint.z());
}

Eigen::ParametrizedLine<float, 3> FloorFilter::LineFromCoefficients(const pcl::ModelCoefficients &line_coefficients) {
  Eigen::ParametrizedLine<float, 3>::VectorType point_on_line(
      line_coefficients.values[0], line_coefficients.values[1], line_coefficients.values[2]);
  Eigen::ParametrizedLine<float, 3>::VectorType direction(
      line_coefficients.values[3], line_coefficients.values[4], line_coefficients.values[5]);
  return Eigen::ParametrizedLine<float, 3>(point_on_line, direction);
}

Eigen::Hyperplane<float, 3> FloorFilter::GetSensorPlane(const ros::Time &time) {
  tf::Stamped<tf::Vector3> z_axis;
  z_axis.stamp_ = time;
  z_axis.frame_id_ = sensor_frame_;
  z_axis.setX(0.0);
  z_axis.setY(0.0);
  z_axis.setZ(1.0);
  tf::Stamped<tf::Vector3> z_axis_in_reference;
  tf_listener_.transformVector(reference_frame_, z_axis, z_axis_in_reference);
  Eigen::Hyperplane<float, 3>::VectorType normal(
      z_axis_in_reference.x(),
      z_axis_in_reference.y(),
      z_axis_in_reference.z());
  return Eigen::Hyperplane<float, 3>(normal, 0.0);
}

}

PLUGINLIB_DECLARE_CLASS(floor_filter, FloorFilter, floor_filter::FloorFilter, nodelet::Nodelet);
