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

#include <gtest/gtest.h>

#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

#include <nodelet/loader.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "parsec_perception/geometry.h"

class FloorFilterTest : public testing::Test {
 public:
  FloorFilterTest()
      : node_handle_("~"),
        floor_filter_(NULL),
        sensor_transform_stamp_(1.0) {
  }
  ~FloorFilterTest() {
    delete floor_filter_;
  }

 protected:
  ros::NodeHandle node_handle_;
  parsec_perception::FloorFilter *floor_filter_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Time sensor_transform_stamp_;
  tf::Pose sensor_pose_;

  virtual void SetUp() {
    node_handle_.setParam("sensor_frame", "laser");
    floor_filter_ = new parsec_perception::FloorFilter(node_handle_);
    sensor_pose_.setIdentity();
    sensor_pose_.getOrigin().setZ(1.0);
    sensor_pose_.setRotation(tf::createQuaternionFromRPY(0, M_PI / 4, 0));
    tf::StampedTransform transform(
        sensor_pose_, sensor_transform_stamp_,
        "base_link", "laser");
    tf_broadcaster_.sendTransform(transform);
  }
};

TEST_F(FloorFilterTest, IndicesDifference) {
  std::vector<int> indices;
  indices.push_back(0);
  indices.push_back(2);
  indices.push_back(4);
  std::vector<int> indices_difference;
  floor_filter_->GetIndicesDifference(6, indices, &indices_difference);
  EXPECT_EQ(indices_difference.size(), 3);
  EXPECT_EQ(indices_difference[0], 1);
  EXPECT_EQ(indices_difference[1], 3);
  EXPECT_EQ(indices_difference[2], 5);
}

TEST_F(FloorFilterTest, FilterFloorCandidates) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 4;
  cloud.height = 1;
  cloud.points.push_back(pcl::PointXYZ(0, 0, 1));
  cloud.points.push_back(pcl::PointXYZ(1, 0, 1));
  cloud.points.push_back(pcl::PointXYZ(1, 0, -1));
  cloud.points.push_back(pcl::PointXYZ(1, 0, 0));
  std::vector<int> indices;
  floor_filter_->FilterFloorCandidates(0.1, 1, cloud, &indices);
  EXPECT_FALSE(std::find(indices.begin(), indices.end(), 1) == indices.end());
  EXPECT_FALSE(std::find(indices.begin(), indices.end(), 2) == indices.end());
  EXPECT_FALSE(std::find(indices.begin(), indices.end(), 3) == indices.end());
}

// TEST_F(FloorFilterTest, IntersectWithSightline) {
// }

TEST_F(FloorFilterTest, GetViewpointPoint) {
  pcl::PointXYZ viewpoint;
  EXPECT_TRUE(
      floor_filter_->GetViewpointPoint(sensor_transform_stamp_, &viewpoint));
  EXPECT_DOUBLE_EQ(viewpoint.x, sensor_pose_.getOrigin().x());
  EXPECT_DOUBLE_EQ(viewpoint.y, sensor_pose_.getOrigin().y());
  EXPECT_DOUBLE_EQ(viewpoint.z, sensor_pose_.getOrigin().z());
}

TEST_F(FloorFilterTest, GetSensorPlane) {
  Eigen::Hyperplane<float, 3> sensor_plane;
  EXPECT_TRUE(
      floor_filter_->GetSensorPlane(sensor_transform_stamp_, &sensor_plane));
  // The sensor plane should have an angle of 45 degrees to the
  // x-y-plane. That means the x and z components of the normal need
  // to be equal.
  EXPECT_DOUBLE_EQ(sensor_plane.normal()(0), sensor_plane.normal()(2));
  EXPECT_DOUBLE_EQ(sensor_plane.normal()(1), 0);
  double expected_offset = sqrt(0.5);
  EXPECT_NEAR(sensor_plane.offset(), expected_offset, 1e-6);
}

TEST_F(FloorFilterTest, IntersectWithSightLine) {
  Eigen::ParametrizedLine<float, 3> floor_line(
      Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 1, 0));
  pcl::PointXYZ interection_point;

  pcl::PointXYZ point_on_line(1, 0, 0);
  EXPECT_TRUE(
      floor_filter_->IntersectWithSightline(
          sensor_transform_stamp_, floor_line,
          point_on_line, &interection_point));
  EXPECT_DOUBLE_EQ(interection_point.x, 1.0);
  EXPECT_DOUBLE_EQ(interection_point.y, 0.0);
  EXPECT_DOUBLE_EQ(interection_point.z, 0.0);
  
  pcl::PointXYZ point_ahead_line(0.5, 0.0, 0.5);
  EXPECT_FALSE(
      floor_filter_->IntersectWithSightline(
          sensor_transform_stamp_, floor_line,
          point_ahead_line, &interection_point));

  pcl::PointXYZ point_behind_line(1.5, 0.0, -0.5);
  EXPECT_TRUE(
      floor_filter_->IntersectWithSightline(
          sensor_transform_stamp_, floor_line,
          point_behind_line, &interection_point));
  EXPECT_DOUBLE_EQ(interection_point.x, 1.0);
  EXPECT_DOUBLE_EQ(interection_point.y, 0.0);
  EXPECT_DOUBLE_EQ(interection_point.z, 0.0);

  pcl::PointXYZ point_behind_viewpoint(-0.5, 0.0, 1.5);
  EXPECT_FALSE(
      floor_filter_->IntersectWithSightline(
          sensor_transform_stamp_, floor_line,
          point_behind_viewpoint, &interection_point));
  
  Eigen::ParametrizedLine<float, 3> floor_line_not_on_sensor_plane(
      Eigen::Vector3f(2, 0, 0), Eigen::Vector3f(0, 1, 0));
  EXPECT_FALSE(
      floor_filter_->IntersectWithSightline(
          sensor_transform_stamp_, floor_line_not_on_sensor_plane,
          point_on_line, &interection_point));
}

TEST_F(FloorFilterTest, FindSensorPlaneIntersection) {
  Eigen::ParametrizedLine<float, 3> intersection_line;
  EXPECT_TRUE(
      floor_filter_->FindSensorPlaneIntersection(
          sensor_transform_stamp_, &intersection_line));
  Eigen::ParametrizedLine<float, 3> expected_line(
      Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 1, 0));
  Eigen::Vector3f origin_difference =
      intersection_line.origin() - expected_line.origin();
  EXPECT_TRUE(parsec_perception::geometry::VectorsParallel(
      origin_difference, expected_line.direction(), 1e-6));
  EXPECT_TRUE(parsec_perception::geometry::VectorsParallel(
      intersection_line.direction(), expected_line.direction(), 1e-6));
}

TEST_F(FloorFilterTest, GenerateCliffCloud) {
  Eigen::ParametrizedLine<float, 3> floor_line(
      Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 1, 0));
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  input_cloud.header.stamp = sensor_transform_stamp_;
  input_cloud.width = 3;
  input_cloud.height = 1;
  input_cloud.points.push_back(pcl::PointXYZ(1, 0, 0));
  input_cloud.points.push_back(pcl::PointXYZ(0.5, 0, 0.5));
  input_cloud.points.push_back(pcl::PointXYZ(2, 2, -1));
  std::vector<int> input_indices;
  input_indices.push_back(0);
  input_indices.push_back(1);
  input_indices.push_back(2);  
  pcl::PointCloud<pcl::PointXYZ> cliff_cloud;
  std::vector<int> cliff_indices;
  EXPECT_TRUE(
      floor_filter_->GenerateCliffCloud(
          floor_line, input_cloud, input_indices,
          &cliff_cloud, &cliff_indices));
  EXPECT_EQ(cliff_indices.size(), 1);
  EXPECT_EQ(cliff_indices[0], 2);
  EXPECT_EQ(cliff_cloud.points.size(), 1);
  EXPECT_DOUBLE_EQ(cliff_cloud[0].x, 1.0);
  EXPECT_DOUBLE_EQ(cliff_cloud[0].y, 1.0);
  EXPECT_DOUBLE_EQ(cliff_cloud[0].z, 0.0);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "floor_filter_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
