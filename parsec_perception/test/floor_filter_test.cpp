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
#include <Eigen/Geometry>

#include <ros/ros.h>

TEST(FindFloor, IndicesDifference) {
  parsec_perception::FloorFilter floor_filter;
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
