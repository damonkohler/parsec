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

#include "floor_filter/floor_filter.h"

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <ros/ros.h>

TEST(FindFloor, VectorsParallel) {
  floor_filter::FloorFilter floor_filter;
  Eigen::Vector3f vector_1(1, 0, 0);
  Eigen::Vector3f vector_2(-1, 0, 0);
  Eigen::Vector3f vector_3(2, 0, 0);
  Eigen::Vector3f vector_4(1, 1, 0);
  Eigen::Vector3f vector_5(0, 1, 0);
  EXPECT_TRUE(floor_filter.VectorsParallel(vector_1, vector_2, 1e-9));
  EXPECT_TRUE(floor_filter.VectorsParallel(vector_1, vector_3, 1e-9));
  EXPECT_TRUE(floor_filter.VectorsParallel(vector_2, vector_3, 1e-9));
  EXPECT_FALSE(floor_filter.VectorsParallel(vector_1, vector_4, 1e-9));
  EXPECT_FALSE(floor_filter.VectorsParallel(vector_1, vector_5, 1e-9));
  EXPECT_FALSE(floor_filter.VectorsParallel(vector_4, vector_5, 1e-9));
}

TEST(FindFloor, EuclideanDistance) {
  floor_filter::FloorFilter floor_filter;
  pcl::PointXYZ point_1(1, 0, 0);
  pcl::PointXYZ point_2(2, 0, 0);
  pcl::PointXYZ point_3(0, 1, 0);

  EXPECT_DOUBLE_EQ(floor_filter.EuclideanDistance(point_1, point_2), 1);
  EXPECT_DOUBLE_EQ(floor_filter.EuclideanDistance(point_1, point_3), sqrt(2));
}

TEST(FindFloor, IntersectLines) {
  floor_filter::FloorFilter floor_filter;
  Eigen::ParametrizedLine<float, 3>::VectorType origin(0, 0, 0);
  Eigen::ParametrizedLine<float, 3>::VectorType origin_y_1(0, 1, 0);
  Eigen::ParametrizedLine<float, 3>::VectorType origin_z_1(0, 0, 1);
  Eigen::ParametrizedLine<float, 3>::VectorType x_direction(1, 0, 0);
  Eigen::ParametrizedLine<float, 3>::VectorType y_direction(0, 1, 0);
  Eigen::ParametrizedLine<float, 3>::VectorType xy_direction(1, 1, 0);
  
  Eigen::ParametrizedLine<float, 3> x_axis(origin, x_direction);
  Eigen::ParametrizedLine<float, 3> y_axis(origin, y_direction);

  // The two lines intersect because they have the same origin
  Eigen::ParametrizedLine<float, 3>::VectorType intersection;
  EXPECT_TRUE(floor_filter.IntersectLines(x_axis, y_axis, &intersection));
  EXPECT_DOUBLE_EQ((intersection - origin).norm(), 0);

  // The two lines intersect in (-1, 0, 0)
  Eigen::ParametrizedLine<float, 3> xy_line(origin_y_1, xy_direction);
  Eigen::ParametrizedLine<float, 3>::VectorType correct_intersection(-1, 0, 0);
  EXPECT_TRUE(floor_filter.IntersectLines(x_axis, xy_line, &intersection));
  EXPECT_DOUBLE_EQ((intersection - correct_intersection).norm(), 0);

  // Two parallel lines cannot intersect
  Eigen::ParametrizedLine<float, 3> y_1_line(origin_y_1, x_direction);
  EXPECT_FALSE(floor_filter.IntersectLines(x_axis, y_1_line, &intersection));

  // Lines are skew
  Eigen::ParametrizedLine<float, 3> skew_line(origin_z_1, y_direction);
  EXPECT_FALSE(floor_filter.IntersectLines(x_axis, skew_line, &intersection));
}

TEST(FindFloor, IntersectPlanes) {
  floor_filter::FloorFilter floor_filter;
  Eigen::Hyperplane<float, 3>::VectorType origin(0, 0, 0);
  Eigen::Hyperplane<float, 3>::VectorType x_direction(1, 0, 0);
  Eigen::Hyperplane<float, 3>::VectorType y_direction(0, 1, 0);
  Eigen::Hyperplane<float, 3>::VectorType z_direction(0, 0, 1);
  Eigen::ParametrizedLine<float, 3> y_axis(origin, y_direction);

  Eigen::Hyperplane<float, 3> xy_plane(z_direction, origin);
  Eigen::Hyperplane<float, 3> xz_plane(y_direction, origin);
  Eigen::ParametrizedLine<float, 3> intersection_line;
  EXPECT_TRUE(floor_filter.IntersectPlanes(xy_plane, xz_plane, &intersection_line));
  Eigen::ParametrizedLine<float, 3>::VectorType intersection_point;
  EXPECT_TRUE(floor_filter.IntersectLines(intersection_line, y_axis, &intersection_point));
  EXPECT_DOUBLE_EQ((intersection_point - origin).norm(), 0);
  EXPECT_TRUE(floor_filter.VectorsParallel(intersection_line.direction(), x_direction, 1e-6));
}

TEST(FindFloor, IndicesDifference) {
  floor_filter::FloorFilter floor_filter;
  std::vector<int> indices;
  indices.push_back(0);
  indices.push_back(2);
  indices.push_back(4);
  std::vector<int> indices_difference;
  floor_filter.GetIndicesDifference(6, indices, &indices_difference);
  EXPECT_EQ(indices_difference.size(), 3);
  EXPECT_EQ(indices_difference[0], 1);
  EXPECT_EQ(indices_difference[1], 3);
  EXPECT_EQ(indices_difference[2], 5);
}

int main(int argc, char *argv[]) {
  // TODO(moesenle): pull out vector math functions into a separate
  // class and avoid ros init.
  ros::init(argc, argv, "floor_filter_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
