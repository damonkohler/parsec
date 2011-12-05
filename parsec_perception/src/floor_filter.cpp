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

#include "parsec_perception/floor_filter.h"

#include <cmath>

#include <algorithm>
#include <iterator>

#include <boost/bind.hpp>
#include <boost/timer.hpp>

#include <Eigen/Geometry>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <ros_check/ros_check.h>

#include "parsec_perception/geometry.h"

namespace parsec_perception {

const std::string FloorFilter::kDefaultReferenceFrame("base_link");

FloorFilter::FloorFilter(const ros::NodeHandle &node_handle)
    : node_handle_(node_handle) {
  if (!node_handle_.getParam("sensor_frame", sensor_frame_)) {
    ROS_FATAL("Parameter 'sensor_frame' not found.");
    return;
  }
  node_handle_.param(
      "reference_frame", reference_frame_, kDefaultReferenceFrame);
  node_handle_.param(
      "floor_z_distance", floor_z_distance_, kDefaultFloorZDistance);
  node_handle_.param(
      "max_floor_y_rotation", max_floor_y_rotation_, kDefaultMaxFloorYRotation);
  node_handle_.param(
      "max_floor_x_rotation", max_floor_x_rotation_, kDefaultMaxFloorXRotation);
  node_handle_.param(
      "line_distance_threshold", line_distance_threshold_, kDefaultLineDistanceThreshold);
  node_handle_.param(
      "cliff_distance_threshold", cliff_distance_threshold_, kDefaultCliffDistanceThreshold);
      
  input_cloud_subscriber_ =
      node_handle_.subscribe<pcl::PointCloud<pcl::PointXYZ> >(
          "input", 100, boost::bind(&FloorFilter::CloudCallback, this, _1));
  filtered_cloud_publisher_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
          "output", 10);
  floor_cloud_publisher_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
          "floor_cloud", 10);
  cliff_cloud_publisher_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
          "cliff_cloud", 10);
  cliff_generating_cloud_publisher_ =
      node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
          "cliff_generating_cloud", 10);
}

void FloorFilter::CloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (!WaitForTransformToReferenceFrame(cloud->header.frame_id, cloud->header.stamp)) {
    ROS_WARN("Cannot transform pointcloud to reference frame (%s -> %s).",
             cloud->header.frame_id.c_str(), reference_frame_.c_str());
    return;
  }
  try {
    pcl_ros::transformPointCloud(reference_frame_, *cloud, *transformed_cloud, tf_listener_);
  } catch (tf::TransformException e) {
    // Transformation fails in particular at start up because tilting
    // laser transforms might not be coming in yet. This is logged by
    // TF already, so we don't add another logging here.
    return;
  }
  if(!transformed_cloud->points.size()) {
    ROS_WARN("The input cloud is empty. No obstacles in range?");
    return;
  }
  
  std::vector<int> floor_candidate_indices;
  FilterFloorCandidates(floor_z_distance_, tan(max_floor_y_rotation_), *transformed_cloud,
                        &floor_candidate_indices);

  Eigen::ParametrizedLine<float, 3> floor_line;
  std::vector<int> line_inlier_indices;
  std::vector<int> indices_without_floor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cliff_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> cliff_indices;
  if (GetFloorLine(transformed_cloud, floor_candidate_indices, &floor_line, &line_inlier_indices)) {
    GetIndicesDifference(transformed_cloud->size(), line_inlier_indices, &indices_without_floor);
    GenerateCliffCloud(floor_line, *transformed_cloud, indices_without_floor,
                       cliff_cloud.get(), &cliff_indices);
  }
  else {
    cliff_cloud->header = transformed_cloud->header;
  }
  // Always publish clouds even if they are empty to signal that
  // perception is still alive.
  PublishCloudFromIndices(*transformed_cloud, line_inlier_indices, floor_cloud_publisher_);
  PublishCloudFromIndices(*transformed_cloud, indices_without_floor, filtered_cloud_publisher_);
  PublishCloudFromIndices(*transformed_cloud, cliff_indices, cliff_generating_cloud_publisher_);
  cliff_cloud_publisher_.publish(cliff_cloud);
  
  ros::Duration message_age = ros::Time::now() - cloud->header.stamp;
  // This is just a hint. Throw a warning to make the user know
  // about something being fishy with the current configuration
  // because input data is pretty old.
  if (message_age > ros::Duration(1.0)) {
    ROS_WARN("Filtered cloud already %lf seconds old.", message_age.toSec());
  }
}

void FloorFilter::FilterFloorCandidates(
    double floor_z_distance, double max_slope, pcl::PointCloud<pcl::PointXYZ> &cloud,
    std::vector<int> *indices) {
  for (size_t i = 0; i < cloud.points.size(); i++) {
    double point_distance_xy = sqrt(cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y);
    if (fabs(cloud.points[i].z) < floor_z_distance + max_slope * point_distance_xy) {
      indices->push_back(i);
    }
  }
}

bool FloorFilter::FindLine(
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
  ransac_line_finder.setDistanceThreshold(line_distance_threshold_);
  ransac_line_finder.setAxis(Eigen::Vector3f(0, 1, 0));
  ransac_line_finder.setEpsAngle(max_floor_x_rotation_);
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

bool FloorFilter::GetFloorLine(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
    const std::vector<int> &indices,
    Eigen::ParametrizedLine<float, 3> *line, std::vector<int> *inlier_indices) {
  Eigen::ParametrizedLine<float, 3> sensor_floor_intersection_line;
  if (!FindSensorPlaneIntersection(input_cloud->header.stamp, &sensor_floor_intersection_line)) {
    // It is impossible to find a correct floor line because the
    // sensor plane is either parallel to the floor line or intersects
    // with it behind the robot.
    return false;
  }
  Eigen::Vector3f y_axis(0, 1, 0);
  if (!FindLine(input_cloud, indices, line, inlier_indices)) {
    ROS_DEBUG("RANSAC couldn't find a floor line.");
    *line = sensor_floor_intersection_line;
  }
  else if (!geometry::VectorsParallel(line->direction(), y_axis, max_floor_x_rotation_)) {
    ROS_DEBUG("The angle between the found floor line and the x-y-plane above threshold."
              "Rejecting the line and using the intersection between x-y-plane and sensor plane.");
    inlier_indices->clear();
    *line = sensor_floor_intersection_line;
  }
  return true;
}

bool FloorFilter::FindSensorPlaneIntersection(
    const ros::Time &time, Eigen::ParametrizedLine<float, 3> *intersection_line) {
  Eigen::Hyperplane<float, 3> sensor_plane;
  if (!GetSensorPlane(time, &sensor_plane)) {
    return false;
  }
  // The sensor plane and the floor plane will intersect behind the
  // robot when the x component of its normal is smaller than zero.
  if (sensor_plane.normal()(0) < 0) {
    ROS_DEBUG("The sensor plane intersects the floor plane behind the robot.");
    return false;
  }  
  // Use intersection of the x-y-plane and the sensor plane to
  // generate an artificial floor line.
  Eigen::Hyperplane<float, 3> floor_plane(Eigen::Hyperplane<float, 3>::VectorType(0, 0, 1), 0.0);
  // Clear all line inliers because even if we found a line we
  // rejected it because it was not parallel to the x-y-plane.
  if (!geometry::IntersectPlanes(sensor_plane, floor_plane, intersection_line)) {
    ROS_DEBUG("No intersection between sensor plane and x-y-plane.");
    return false;
  }
  return true;
}

bool FloorFilter::GenerateCliffCloud(
    const Eigen::ParametrizedLine<float, 3> &floor_line,
    const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
    const std::vector<int> &input_indices,
    pcl::PointCloud<pcl::PointXYZ> *cliff_cloud,
    std::vector<int> *cliff_indices) {
  cliff_cloud->header = input_cloud.header;
  cliff_cloud->height = 1;
  cliff_cloud->is_dense = false;
  cliff_cloud->points.clear();

  pcl::PointXYZ viewpoint;
  if (!GetViewpointPoint(input_cloud.header.stamp, &viewpoint)) {
    return false;
  }
  for (size_t i = 0; i < input_indices.size(); i++) {
    pcl::PointXYZ cliff_point;
    if (!IntersectWithSightline(input_cloud.header.stamp, floor_line, input_cloud.points[i],
                                &cliff_point)) {
      continue;
    }
    double distance_to_input_point = pcl::euclideanDistance(viewpoint, input_cloud.points[i]);
    double distance_to_cliff_point = pcl::euclideanDistance(viewpoint, cliff_point);
    double distance_cliff_from_point_xy =
      pcl::euclideanDistance(cliff_point,
                        pcl::PointXYZ(input_cloud.points[i].x,
                                      input_cloud.points[i].y,
                                      cliff_point.z));
    double distance_from_floor =
      sqrt((distance_to_input_point - distance_to_cliff_point) *
           (distance_to_input_point - distance_to_cliff_point)
           - distance_cliff_from_point_xy * distance_cliff_from_point_xy);
    if (distance_from_floor > cliff_distance_threshold_) {
      cliff_indices->push_back(i);
      cliff_cloud->points.push_back(cliff_point);
    }
  }
  cliff_cloud->width = cliff_cloud->points.size();
  return true;
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

void FloorFilter::PublishCloudFromIndices(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                          const std::vector<int> &indices,
                                          ros::Publisher &publisher) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr indices_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  MakeCloudFromIndices(cloud, indices, indices_cloud.get());
  publisher.publish(indices_cloud);
}

bool FloorFilter::IntersectWithSightline(
    const ros::Time &time, const Eigen::ParametrizedLine<float, 3> &line, const pcl::PointXYZ &point,
    pcl::PointXYZ *intersection_point) {
  pcl::PointXYZ viewpoint_pcl;
  if (!GetViewpointPoint(time, &viewpoint_pcl)) {
    return false;
  }
  Eigen::ParametrizedLine<float, 3>::VectorType viewpoint = viewpoint_pcl.getVector3fMap();
  Eigen::ParametrizedLine<float, 3> viewpoint_line(viewpoint, point.getVector3fMap() - viewpoint);
  Eigen::ParametrizedLine<float, 3>::VectorType intersection;
  if (geometry::IntersectLines(line, viewpoint_line, &intersection)) {
    *intersection_point = pcl::PointXYZ(intersection[0], intersection[1], intersection[2]);
    // If the point is actually closer to the viewpoint than the
    // intersection point we have no real intersection because the
    // point is above the floor.
    if (pcl::euclideanDistance(viewpoint_pcl, *intersection_point) >
        pcl::euclideanDistance(viewpoint_pcl, point)) {
      return false;
    }
    return true;
  }
  else {
    return false;
  }
}

bool FloorFilter::GetViewpointPoint(const ros::Time &time, pcl::PointXYZ *point) {
  tf::Stamped<tf::Point> sensor_point(btVector3(0, 0, 0), time, sensor_frame_);
  tf::Stamped<tf::Point> viewpoint;
  if (!WaitForTransformToReferenceFrame(sensor_frame_, time)) {
    ROS_WARN("Cannot get sensor transform (%s -> %s).",
             reference_frame_.c_str(), sensor_frame_.c_str());
    return false;
  }
  try {
    tf_listener_.transformPoint(reference_frame_, sensor_point, viewpoint);
  } catch (tf::TransformException e) {
    return false;
  }
  *point = pcl::PointXYZ(viewpoint.x(), viewpoint.y(), viewpoint.z());
  return true;
}

Eigen::ParametrizedLine<float, 3> FloorFilter::LineFromCoefficients(
    const pcl::ModelCoefficients &line_coefficients) {
  Eigen::ParametrizedLine<float, 3>::VectorType point_on_line(
      line_coefficients.values[0], line_coefficients.values[1], line_coefficients.values[2]);
  Eigen::ParametrizedLine<float, 3>::VectorType direction(
      line_coefficients.values[3], line_coefficients.values[4], line_coefficients.values[5]);
  return Eigen::ParametrizedLine<float, 3>(point_on_line, direction);
}

bool FloorFilter::GetSensorPlane(
    const ros::Time &time, Eigen::Hyperplane<float, 3> *sensor_plane) {
  tf::Stamped<tf::Vector3> z_axis;
  z_axis.stamp_ = time;
  z_axis.frame_id_ = sensor_frame_;
  z_axis.setX(0.0);
  z_axis.setY(0.0);
  z_axis.setZ(1.0);
  tf::Stamped<tf::Vector3> z_axis_in_reference;
  if (!WaitForTransformToReferenceFrame(sensor_frame_, time)) {
    ROS_WARN("Cannot get sensor transform to calculate sensor plane (%s -> %s).",
             reference_frame_.c_str(), sensor_frame_.c_str());
    return false;
  }
  try {
    tf_listener_.transformVector(reference_frame_, z_axis, z_axis_in_reference);
  } catch (tf::TransformException e) {
    return false;
  }
  Eigen::Hyperplane<float, 3>::VectorType normal(
      z_axis_in_reference.x(),
      z_axis_in_reference.y(),
      z_axis_in_reference.z());
  pcl::PointXYZ viewpoint;
  if (!GetViewpointPoint(time, &viewpoint)) {
    return false;
  }
  // Eigen seems to have a bug. This prevents us from using the
  // constructor that takes a normal and a point on the
  // plane. Instead, we calculate the distance of the plane from the
  // origin similar to Eigen but without the negative sign.
  *sensor_plane = Eigen::Hyperplane<float, 3>(
      normal, normal.dot(viewpoint.getVector3fMap()));
  return true;
}

void FloorFilter::GetIndicesDifference(size_t cloud_size, const std::vector<int> &indices,
                                       std::vector<int> *difference) {
  std::vector<int> all_indices(cloud_size);
  for (size_t i = 0; i < all_indices.size(); i++) {
    all_indices[i] = i;
  }
  std::vector<int> copied_indices(indices.begin(), indices.end());
  std::sort(copied_indices.begin(), copied_indices.end());
  std::set_difference(all_indices.begin(), all_indices.end(),
                      copied_indices.begin(), copied_indices.end(),
                      std::back_insert_iterator<std::vector<int> >(*difference));
}

}  // namespace parsec_perception
