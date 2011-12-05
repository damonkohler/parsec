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

#include <ros_check/ros_check.h>

namespace parsec_perception {

namespace geometry {

bool IntersectLines(
    const Eigen::ParametrizedLine<float, 3> &line1, const Eigen::ParametrizedLine<float, 3> &line2,
    Eigen::ParametrizedLine<float, 3>::VectorType *intersection_point) {
  Eigen::ParametrizedLine<float, 3>::VectorType z_direction(0, 0, 1);
  // If one of our lines is parallel to the z axis we needed to use a
  // different formula.
  // TODO(moesenle): handle the case when one of the lines is parallel
  // to the z axis.
  CHECK(!VectorsParallel(line1.direction(), z_direction, 1e-6));
  CHECK(!VectorsParallel(line2.direction(), z_direction, 1e-6));
  Eigen::ParametrizedLine<float, 3>::Scalar divisor =
      line1.direction()(0) * line2.direction()(1) - line1.direction()(1) * line2.direction()(0);
  // If magnitude_divisor is zero, the two lines definitely don't
  // intersect.
  if (fabs(divisor) < 1e-6) {
    return false;
  }
  // Check if the lines are skew, i.e. if the minimum distance between the two
  // lines is > 0.
  double distance;
  if (!LineToLineDistance(line1, line2, &distance) || distance > 0.001) {
    return false;
  }
  // This formular can be derived from setting the two line equations
  // equal and solving the resulting equation system.
  Eigen::ParametrizedLine<float, 3>::Scalar magnitude =
      (line1.origin()(1) + line2.origin()(0) * line2.direction()(1)
       - line2.origin()(1) * line2.direction()(0)
       - line1.origin()(0) * line2.direction()(1))
      / divisor;
  *intersection_point = line1.origin() + line1.direction() * magnitude;
  return true;
}

bool LineToLineDistance(
    const Eigen::ParametrizedLine<float, 3> &line1, const Eigen::ParametrizedLine<float, 3> &line2,
    double *distance) {
  Eigen::ParametrizedLine<float, 3>::VectorType normal = line1.direction().cross(line2.direction());
  if (normal.norm() < 1e-6) {
    return false;
  }
  *distance = (normal / normal.norm()).dot(line2.origin() - line1.origin());
  return true;
}

bool IntersectPlanes(
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
  Eigen::Hyperplane<float, 3>::Scalar n1_n1  = plane1.normal().dot(plane1.normal());
  Eigen::Hyperplane<float, 3>::Scalar n2_n2  = plane2.normal().dot(plane2.normal());
  Eigen::Hyperplane<float, 3>::Scalar n1_n2  = plane1.normal().dot(plane2.normal());
  Eigen::Hyperplane<float, 3>::Scalar determinant =
      n1_n1 * n2_n2 - (n1_n2 * n1_n2);
  Eigen::Hyperplane<float, 3>::Scalar c1 =
      (plane1.offset() * n2_n2 - plane2.offset() * n1_n2) / determinant;
  Eigen::Hyperplane<float, 3>::Scalar c2 =
      (plane2.offset() * n1_n1 - plane1.offset() * n1_n2) / determinant;
  Eigen::ParametrizedLine<float, 3>::VectorType origin =
    c1 * plane1.normal() + c2 * plane2.normal() + direction / direction.norm();
  *intersection = Eigen::ParametrizedLine<float, 3>(origin, direction);
  return true;
}

}  // namespace geometry

}  // namespace parsec_perception
