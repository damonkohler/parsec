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

#ifndef PARSEC_PERCEPTION_FLOOR_FILTER_H
#define PARSEC_PERCEPTION_FLOOR_FILTER_H

#include <vector>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <pcl/ModelCoefficients.h>

namespace parsec_perception {

class FloorFilter {
 public:
  FloorFilter(const ros::NodeHandle &node_handle);

  /**
   * Generates the indices of all points that are not in indices.
   *
   * Public for testing.
   *
   * @param cloud_size the number of points in the point cloud
   * @param indices the indices that should not be in the result
   * @param difference cloud indices not in indices
   */
  void GetIndicesDifference(size_t cloud_size, const std::vector<int> &indices,
                            std::vector<int> *difference);
 
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
   *
   * Public for testing.
   */
  void FilterFloorCandidates(double floor_z_distance, double max_slope,
                             pcl::PointCloud<pcl::PointXYZ> &cloud,
                             std::vector<int> *indices);

private:
  static const double kDefaultFloorZDistance = 0.05;
  static const double kDefaultMaxFloorYRotation = 0.035;  // 2 degrees
  static const double kDefaultMaxFloorXRotation = 0.087;  // 5 degrees
  static const double kDefaultLineDistanceThreshold = 0.03;
  static const double kDefaultCliffDistanceThreshold = 0.02;
  static const std::string kDefaultReferenceFrame;

  ros::NodeHandle node_handle_;
  ros::Subscriber input_cloud_subscriber_;
  ros::Publisher floor_cloud_publisher_;
  ros::Publisher filtered_cloud_publisher_;
  ros::Publisher cliff_cloud_publisher_;
  ros::Publisher cliff_generating_cloud_publisher_;
  tf::TransformListener tf_listener_;

  /**
   * The maximal distance from the x-y-planes points can have to be
   * considered as floor candidates.
   */
  double floor_z_distance_;
  
  /**
   * The maximal rotation of the floor around the y axis. This value
   * is used for filtering candidate points. Actual candidate points
   * are points that have a z coordinate of at most floor_z_distance
   * m, taking into account a maximal y rotation. I.e. points that are
   * further away can also be further away from the x-y-plane to still
   * be considered as floor candidates.
   */
  double max_floor_y_rotation_;

  /**
   * The maximal rotation around the x axis in radians the floor can
   * have before the floor line is rejected as a false
   * positive.
   */
  double max_floor_x_rotation_;
  
  /**
   * Distance threshold for points to be considered as inliers. This
   * parameter is directly used by PCL's ransac.
   */
  double line_distance_threshold_;

  /**
   * Distance threshold for determining if a point is possibly a
   * cliff. Points that are further away from the floor line than this
   * threshold are considered for cliff checking.
   */
  double cliff_distance_threshold_;

  /**
   * The frame name of the sensor creating our input data. Mandatory.
   */
  std::string sensor_frame_;

  /**
   * The reference frame in which we perform our processing.
   */
  std::string reference_frame_;

  void CloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

  /**
   * Takes an input cloud and point indices and returns the
   * coefficients of the dominant line as well as the indices of all
   * line inliers.
   *
   * @param input_cloud the input cloud
   * @param indices
   *     the indices of points in the input cloud to take into account
   * @param line the Eigen representation of the line
   * @param inlier_indices the indices of all points on the line
   */
  bool FindLine(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
                const std::vector<int> &indices,
                Eigen::ParametrizedLine<float, 3> *line,
                std::vector<int> *inlier_indices);

  /**
   * Finds the floor line. If no floor line could be found, i.e. the
   * sensor plane doesn't intersect with the x-y-plane, returns false.
   *
   * @param input_cloud the input cloud
   * @param indices
   *     the indices of points in the input cloud to take into account
   * @param line the Eigen representation of the line
   * @param inlier_indices the indices of all points on the floor line
   */
  bool GetFloorLine(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
                    const std::vector<int> &indices,
                    Eigen::ParametrizedLine<float, 3> *line,
                    std::vector<int> *inlier_indices);

  /**
   * Generates cliff points by calculating the intersection between
   * the sightline to a point and the floor line. This method assumes
   * that the sensor is pointing in the direction of the floor line at
   * the time input_indices was collected.
   *
   * @param floor_line the Eigen representation of the floor line
   * @param input_cloud input point cloud
   * @param input_indices
   *     indices of points in the input cloud to consider
   * @param cliff_cloud generated points indicating cliffs
   * @param cliff_indices indices in input_cloud indicating points
   *     that were used to generate cliff points, i.e. points below
   *     the floor
   */
  bool GenerateCliffCloud(const Eigen::ParametrizedLine<float, 3> &floor_line,
                          const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
                          const std::vector<int> &input_indices,
                          pcl::PointCloud<pcl::PointXYZ> *cliff_cloud,
                          std::vector<int> *cliff_indices);

  void MakeCloudFromIndices(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
                            const std::vector<int> &input_indices,
                            pcl::PointCloud<pcl::PointXYZ> *output_cloud);

  void PublishCloudFromIndices(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                               const std::vector<int> &indices,
                               ros::Publisher &publisher);

  /**
   * Calculates the intersection point between the sight line,
   * i.e. the line between the view point and point, and line.
   *
   * @param time the time at which to take the viewpoint
   * @param line the Eigen representation of the line
   * @param point the point to project
   * @param intersection_point
   *     the point where line and the view point line intersect
   *
   * @return true if valid intersection could be found, false otherwise
   */
  bool IntersectWithSightline(
      const ros::Time &time, const Eigen::ParametrizedLine<float, 3> &line,
      const pcl::PointXYZ &point,
      pcl::PointXYZ *intersection_point);

  /**
   * Returns the position of the view point, i.e. the sensor frame, at
   * a specific time in reference frame.
   *
   * @param time indicates when to take the view point position
   *
   * @return the viewpoint at a specific time
   */
  bool GetViewpointPoint(const ros::Time &time, pcl::PointXYZ *point);

  Eigen::ParametrizedLine<float, 3> LineFromCoefficients(
      const pcl::ModelCoefficients &line_coefficients);

  /**
   * Returns the plane in which all 2D sensor measurements
   * (e.g. laser) are supposed to be in. Uses the time to look up the
   * position of the sensor in TF and returns the plane created by the
   * sensor frame along its x- and y-axis in reference_frame_.
   *
   * @param time indicates which sensor pose to use
   */ 
  bool GetSensorPlane(const ros::Time &time, Eigen::Hyperplane<float, 3> *sensor_plane);

  /**
   * Calculates the intersection line between the sensor plane with
   * the x-y-plane. If the line is behind the robot, returns false.
   */
  bool FindSensorPlaneIntersection(
      const ros::Time &time, Eigen::ParametrizedLine<float, 3> *intersection_line);

  /**
   * Checks if a transform to the reference frame is available.
   */
  bool WaitForTransformToReferenceFrame(
      const std::string source_frame, const ros::Time &time) {
    return tf_listener_.waitForTransform(
        reference_frame_, source_frame, time, ros::Duration(0.2));
  }
};

}  // namespace parsec_perception

#endif  // PARSEC_PERCEPTION_FLOOR_FILTER_H
