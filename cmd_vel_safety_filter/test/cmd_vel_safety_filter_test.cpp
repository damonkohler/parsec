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

#include "cmd_vel_safety_filter/cmd_vel_safety_filter.h"

#include <ros/ros.h>

#include <gtest/gtest.h>

class CmdVelSafetyFilterTest : public testing::Test {
 public:
  CmdVelSafetyFilterTest() : filter_(NULL) {}
  virtual void SetUp() {
    ros::NodeHandle node_handle("~");
    node_handle.setParam("radius", 0.25);
    node_handle.setParam("max_acceleration", 1.0);
    node_handle.setParam("stop_distance", 0.25);
    filter_ = new cmd_vel_safety_filter::CmdVelSafetyFilter(ros::NodeHandle("~"));
  }
  virtual void TearDown() {
    delete filter_;
    filter_ = NULL;
  }
 protected:
  cmd_vel_safety_filter::CmdVelSafetyFilter *filter_;
};

TEST_F(CmdVelSafetyFilterTest, FindPointsInDirection) {
  std::vector<tf::Point> points;
  points.push_back(tf::Point(2, 0, 0));
  points.push_back(tf::Point(0, 2, 0));
  points.push_back(tf::Point(0, 0, 2));
  points.push_back(tf::Point(2, 2, 0));

  std::vector<tf::Point> points_x;
  filter_->FindPointsInDirection(
      btVector3(1, 0, 0), 0.5, points, &points_x);
  EXPECT_EQ(points_x.size(), 1);
  EXPECT_DOUBLE_EQ(points_x[0].x(), 2);
  EXPECT_DOUBLE_EQ(points_x[0].y(), 0);
  EXPECT_DOUBLE_EQ(points_x[0].z(), 0);

  std::vector<tf::Point> points_y;
  filter_->FindPointsInDirection(
      btVector3(0, 1, 0), 0.5, points, &points_y);
  EXPECT_EQ(points_y.size(), 1);
  EXPECT_DOUBLE_EQ(points_y[0].x(), 0);
  EXPECT_DOUBLE_EQ(points_y[0].y(), 2);
  EXPECT_DOUBLE_EQ(points_y[0].z(), 0);

  std::vector<tf::Point> points_z;
  filter_->FindPointsInDirection(
      btVector3(0, 0, 1), 0.5, points, &points_z);
  EXPECT_EQ(points_z.size(), 1);
  EXPECT_DOUBLE_EQ(points_z[0].x(), 0);
  EXPECT_DOUBLE_EQ(points_z[0].y(), 0);
  EXPECT_DOUBLE_EQ(points_z[0].z(), 2);

  std::vector<tf::Point> points_xy;
  filter_->FindPointsInDirection(
      btVector3(1, 1, 0), 0.5, points, &points_xy);
  EXPECT_EQ(points_xy.size(), 1);
  EXPECT_DOUBLE_EQ(points_xy[0].x(), 2);
  EXPECT_DOUBLE_EQ(points_xy[0].y(), 2);
  EXPECT_DOUBLE_EQ(points_xy[0].z(), 0);
}

TEST_F(CmdVelSafetyFilterTest, FilterCmdVelPointInFront) {
  std::vector<tf::Point> points;
  points.push_back(tf::Point(0.75, 0, 0));
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  geometry_msgs::Twist filtered_cmd_vel;
  EXPECT_TRUE(filter_->FilterCmdVel(cmd_vel, points, &filtered_cmd_vel));
  EXPECT_DOUBLE_EQ(filtered_cmd_vel.linear.x, 0.5);
}

TEST_F(CmdVelSafetyFilterTest, FilterCmdVelPointLeft) {
  std::vector<tf::Point> points;
  points.push_back(tf::Point(0, 0.75, 0));
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  geometry_msgs::Twist filtered_cmd_vel;
  EXPECT_TRUE(filter_->FilterCmdVel(cmd_vel, points, &filtered_cmd_vel));
  EXPECT_DOUBLE_EQ(filtered_cmd_vel.linear.x, 1.0);
}

TEST_F(CmdVelSafetyFilterTest, FilterCmdVelPointRight) {
  std::vector<tf::Point> points;
  points.push_back(tf::Point(0, -0.75, 0));
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  geometry_msgs::Twist filtered_cmd_vel;
  EXPECT_TRUE(filter_->FilterCmdVel(cmd_vel, points, &filtered_cmd_vel));
  EXPECT_DOUBLE_EQ(filtered_cmd_vel.linear.x, 1.0);
}

TEST_F(CmdVelSafetyFilterTest, FilterCmdVelPointFarAway) {
  std::vector<tf::Point> points;
  points.push_back(tf::Point(10.0, 0, 0));
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  geometry_msgs::Twist filtered_cmd_vel;
  EXPECT_TRUE(filter_->FilterCmdVel(cmd_vel, points, &filtered_cmd_vel));
  EXPECT_DOUBLE_EQ(filtered_cmd_vel.linear.x, 1.0);
}

TEST_F(CmdVelSafetyFilterTest, FilterCmdVelPointInFrontFast) {
  std::vector<tf::Point> points;
  points.push_back(tf::Point(0.75, 0, 0));
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 10.0;
  geometry_msgs::Twist filtered_cmd_vel;
  EXPECT_TRUE(filter_->FilterCmdVel(cmd_vel, points, &filtered_cmd_vel));
  EXPECT_DOUBLE_EQ(filtered_cmd_vel.linear.x, 0.05);
}

TEST_F(CmdVelSafetyFilterTest, FilterCmdVelPointInCollision) {
  std::vector<tf::Point> points;
  points.push_back(tf::Point(0.25, 0, 0));
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 10.0;
  geometry_msgs::Twist filtered_cmd_vel;
  EXPECT_TRUE(filter_->FilterCmdVel(cmd_vel, points, &filtered_cmd_vel));
  EXPECT_DOUBLE_EQ(filtered_cmd_vel.linear.x, 0.0);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "priority_mux_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
