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

#ifndef PARSEC_PERCEPTION_GEOMETRY_H
#define PARSEC_PERCEPTION_GEOMETRY_H

#include <Eigen/Geometry>

namespace parsec_perception {

namespace geometry {

/**
 * Checks if two vectors are (almost) parallel with respect to a
 * maximal angle between them.
 *
 * Public for testing only.
 */
template<typename T>
bool VectorsParallel(
    const T& vector1, const T& vector2, double angle_threshold) {
  return fabs(vector1.dot(vector2) / (vector1.norm() * vector2.norm()))
      >= cos(angle_threshold);
}

/**
 * Calculates the intersection between two lines in
 * three-dimensional space. Note: the two lines must not be parallel
 * to the z axis.
 *
 * Public for testing.
 *
 * @return true if a valid intersection could be found, false otherwise
 */
bool IntersectLines(
    const Eigen::ParametrizedLine<float, 3> &line1,
    const Eigen::ParametrizedLine<float, 3> &line2,
    Eigen::ParametrizedLine<float, 3>::VectorType *interection_point);

/**
 * Calculates the minimum distance between the two lines. If the
 * lines are parallel, returns false.
 *
 * Public for testing.
 */
bool LineToLineDistance(const Eigen::ParametrizedLine<float, 3> &line1,
                        const Eigen::ParametrizedLine<float, 3> &line2,
                        double *distance);

/**
 * Intersects two planes and returns the intersecting line. If the
 * calculation fails, i.e. if the two planes are paralle, return
 * false.
 *
 * Public for testing.
 */
bool IntersectPlanes(const Eigen::Hyperplane<float, 3> &plane1,
                     const Eigen::Hyperplane<float, 3> &plane2,
                     Eigen::ParametrizedLine<float, 3> *intersection);

}  // namespace geometry

}  // namespace parsec_perception

#endif  // PARSEC_PERCEPTION_GEOMETRY_H
