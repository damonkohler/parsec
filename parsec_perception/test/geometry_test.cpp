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

#include "parsec_perception/geometry.h"

#include <gtest/gtest.h>
#include <Eigen/Geometry>

TEST(GeometryTest, VectorsParallel) {
  Eigen::Vector3f vector_1(1, 0, 0);
  Eigen::Vector3f vector_2(-1, 0, 0);
  Eigen::Vector3f vector_3(2, 0, 0);
  Eigen::Vector3f vector_4(1, 1, 0);
  Eigen::Vector3f vector_5(0, 1, 0);
  EXPECT_TRUE(
      parsec_perception::geometry::VectorsParallel(vector_1, vector_2, 1e-9));
  EXPECT_TRUE(
      parsec_perception::geometry::VectorsParallel(vector_1, vector_3, 1e-9));
  EXPECT_TRUE(
      parsec_perception::geometry::VectorsParallel(vector_2, vector_3, 1e-9));
  EXPECT_FALSE(
      parsec_perception::geometry::VectorsParallel(vector_1, vector_4, 1e-9));
  EXPECT_FALSE(
      parsec_perception::geometry::VectorsParallel(vector_1, vector_5, 1e-9));
  EXPECT_FALSE(
      parsec_perception::geometry::VectorsParallel(vector_4, vector_5, 1e-9));
}

TEST(GeometryTest, IntersectLines) {
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
  EXPECT_TRUE(parsec_perception::geometry::IntersectLines(
      x_axis, y_axis, &intersection));
  EXPECT_DOUBLE_EQ((intersection - origin).norm(), 0);

  // The two lines intersect in (-1, 0, 0)
  Eigen::ParametrizedLine<float, 3> xy_line(origin_y_1, xy_direction);
  Eigen::ParametrizedLine<float, 3>::VectorType correct_intersection(-1, 0, 0);
  EXPECT_TRUE(parsec_perception::geometry::IntersectLines(
      x_axis, xy_line, &intersection));
  EXPECT_DOUBLE_EQ((intersection - correct_intersection).norm(), 0);

  // Two parallel lines cannot intersect
  Eigen::ParametrizedLine<float, 3> y_1_line(origin_y_1, x_direction);
  EXPECT_FALSE(parsec_perception::geometry::IntersectLines(
      x_axis, y_1_line, &intersection));

  // Lines are skew
  Eigen::ParametrizedLine<float, 3> skew_line(origin_z_1, y_direction);
  EXPECT_FALSE(parsec_perception::geometry::IntersectLines(
      x_axis, skew_line, &intersection));
}

TEST(GeometryTest, IntersectPlanes) {
  Eigen::Hyperplane<float, 3>::VectorType origin(0, 0, 0);
  Eigen::Hyperplane<float, 3>::VectorType x_direction(1, 0, 0);
  Eigen::Hyperplane<float, 3>::VectorType y_direction(0, 1, 0);
  Eigen::Hyperplane<float, 3>::VectorType z_direction(0, 0, 1);
  Eigen::ParametrizedLine<float, 3> y_axis(origin, y_direction);

  Eigen::Hyperplane<float, 3> xy_plane(z_direction, origin);
  Eigen::Hyperplane<float, 3> xz_plane(y_direction, origin);
  Eigen::ParametrizedLine<float, 3> intersection_line;
  EXPECT_TRUE(parsec_perception::geometry::IntersectPlanes(
      xy_plane, xz_plane, &intersection_line));
  Eigen::ParametrizedLine<float, 3>::VectorType intersection_point;
  EXPECT_TRUE(parsec_perception::geometry::IntersectLines(
      intersection_line, y_axis, &intersection_point));
  EXPECT_DOUBLE_EQ((intersection_point - origin).norm(), 0);
  EXPECT_TRUE(parsec_perception::geometry::VectorsParallel(
      intersection_line.direction(), x_direction, 1e-6));
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
