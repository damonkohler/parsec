/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 *
 * Author: moesenle@google.com (Lorenz Moesenlechner)
 */

#ifndef FLOOR_FILTER_H
#define FLOOR_FILTER_H

#include <vector>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/LaserScan.h>

namespace floor_filter {

class FloorFilter : public pcl_ros::PCLNodelet {
 public:
  FloorFilter()
    : PCLNodelet() {}

 protected:
  virtual void onInit();
  
 private:
  ros::Subscriber input_scan_sub_;
  ros::Subscriber input_cloud_sub_;
  ros::Publisher floor_cloud_pub_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher cliff_cloud_pub_;
  double floor_z_distance_;
  double max_slope_;
  double ransac_distance_threshold_;
  std::string viewpoint_frame_;
  std::string reference_frame_;

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

  void CloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

  /**
   * Get the indices of points that are possibly the floor. Uses a
   * threshold and a slope parameter and returns only the indices of
   * points that are in the corresponding area.
   *
   * Candidates that are possibly the floor need to have a z
   * coordinate close to 0 (the z axis is pointing upwards) but if we
   * have slope on the floor which might be caused by bad calibration
   * of the sensor, the z-distance for points further away might be
   * bigger. Thus, we need to take into account a maximal slope.
   */
  void FilterFloorCandidates(double floor_z_distance, double max_slope,
                             pcl::PointCloud<pcl::PointXYZ> &cloud,
                             std::vector<int> *indices);

  /**
   * Takes an input cloud and point indices and returns the
   * coefficients of the dominat line as well as the indices of all
   * line inliers.
   *
   * @param input_cloud the input cloud
   * 
   * @param indices
   *     the indices of points in the input cloud to take into account
   *
   * @param line the Eigen representation of the line
   *
   * @param inlier_indices the indices of all points on the floor
   */
  bool FindFloorLine(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
                     const std::vector<int> &indices,
                     Eigen::ParametrizedLine<float, 3> *line,
                     std::vector<int> *inlier_indices);

  /**
   * Generates cliff points by projecting them on the
   * floor. I.e. cliff points are put on the intersection between the
   * floor line and the line from the viewpoint to the actual point
   * and only if the point is further away from the view point than
   * the floor line.
   *
   * @param line the Eigen representation of the line
   *
   * @param input_cloud input point cloud
   *
   * @param input_indices
   *     indices of points in the input cloud to consider
   *
   * @param cliff_cloud generated points indicating cliffs
   *
   * @param cliff_indices indices in input_cloud indicating points
   *     that were used to generate cliff points, i.e. points below
   *     the floor
   */
  void GenerateCliffCloud(const Eigen::ParametrizedLine<float, 3> &line,
                          const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
                          const std::vector<int> &input_indices,
                          pcl::PointCloud<pcl::PointXYZ> *cliff_cloud,
                          std::vector<int> *cliff_indices);

  void MakeCloudFromIndices(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
                            const std::vector<int> &input_indices,
                            pcl::PointCloud<pcl::PointXYZ> *output_cloud);

  double PointToLineDistance(const Eigen::ParametrizedLine<float, 3> &line, const pcl::PointXYZ &point);

  double EuclideanDistance(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2);

  /**
   * Projects a point on a line, i.e. returns the point on the line
   * that is closest to the point.
   *
   * @param line the Eigen representation of the line
   *
   * @param point the point to project
   *
   * @return the point on the line that is closest to point
   */
  pcl::PointXYZ ProjectPointOnLine(const Eigen::ParametrizedLine<float, 3> &line, const pcl::PointXYZ &point);

  pcl::PointXYZ GetViewpointPoint(const ros::Time &time);

  Eigen::ParametrizedLine<float, 3> LineFromCoefficients(const pcl::ModelCoefficients &line_coefficients);
};

}

#endif
