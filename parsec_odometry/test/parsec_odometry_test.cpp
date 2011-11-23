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

#include "parsec_odometry/parsec_odometry.h"

#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>

// Tests if all fields required for a correct odometry message stay
// valid after applying an identity transform.
TEST(ParsecOdometry, CorrectOdometryWithIdentity) {
  nav_msgs::Odometry uncorrected_odometry;
  uncorrected_odometry.header.frame_id = "odom";
  uncorrected_odometry.child_frame_id = "base_link";
  uncorrected_odometry.pose.pose.position.x = 1.0;
  uncorrected_odometry.pose.pose.position.y = 2.0;
  uncorrected_odometry.pose.pose.position.z = 3.0;  
  uncorrected_odometry.pose.pose.orientation.x = 0.0;
  uncorrected_odometry.pose.pose.orientation.y = 0.0;
  uncorrected_odometry.pose.pose.orientation.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.w = 1;
  tf::Transform identity = tf::Transform::getIdentity();
  parsec_odometry::ParsecOdometry parsec_odometry;
  nav_msgs::Odometry corrected_odometry;
  parsec_odometry.CorrectOdometry(
      uncorrected_odometry, identity, &corrected_odometry);
  EXPECT_DOUBLE_EQ(uncorrected_odometry.pose.pose.position.x,
                   corrected_odometry.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(uncorrected_odometry.pose.pose.position.y,
                   corrected_odometry.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(uncorrected_odometry.pose.pose.position.z,
                   corrected_odometry.pose.pose.position.z);
  EXPECT_DOUBLE_EQ(uncorrected_odometry.pose.pose.orientation.x,
                   corrected_odometry.pose.pose.orientation.x);
  EXPECT_DOUBLE_EQ(uncorrected_odometry.pose.pose.orientation.y,
                   corrected_odometry.pose.pose.orientation.y);
  EXPECT_DOUBLE_EQ(uncorrected_odometry.pose.pose.orientation.z,
                   corrected_odometry.pose.pose.orientation.z);
  EXPECT_DOUBLE_EQ(uncorrected_odometry.pose.pose.orientation.w,
                   corrected_odometry.pose.pose.orientation.w);
  EXPECT_EQ(uncorrected_odometry.header.frame_id,
            corrected_odometry.header.frame_id);
  EXPECT_EQ(uncorrected_odometry.child_frame_id,
            corrected_odometry.child_frame_id);
}

// Tests that identity odometry corrected with a transform results in
// the transform.
TEST(ParsecOdometry, CorrectIdentityOdometryWithTransform) {
  nav_msgs::Odometry uncorrected_odometry;
  uncorrected_odometry.pose.pose.orientation.w = 1;
  tf::Transform transform(btQuaternion(btVector3(0, 0, 1), M_PI / 2),
                          btVector3(1, 0, 0));
  parsec_odometry::ParsecOdometry parsec_odometry;
  nav_msgs::Odometry corrected_odometry;
  parsec_odometry.CorrectOdometry(
      uncorrected_odometry, transform, &corrected_odometry);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.x,
                   transform.getOrigin().x());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.y,
                   transform.getOrigin().y());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.z,
                   transform.getOrigin().z());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.x,
                   transform.getRotation().x());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.y,
                   transform.getRotation().y());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.z,
                   transform.getRotation().z());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.w,
                   transform.getRotation().w());
}

// Tests that odometry is transformed correctly
TEST(ParsecOdometry, CorrectyOdometryWithTransform) {
  nav_msgs::Odometry uncorrected_odometry;
  uncorrected_odometry.pose.pose.position.x = 1.0;
  uncorrected_odometry.pose.pose.position.y = 0.0;
  uncorrected_odometry.pose.pose.position.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.x = 0.0;
  uncorrected_odometry.pose.pose.orientation.y = 0.0;
  uncorrected_odometry.pose.pose.orientation.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.w = 1.0;
  tf::Transform transform(btQuaternion(btVector3(0, 0, 1), M_PI / 2),
                          btVector3(1, 0, 0));
  parsec_odometry::ParsecOdometry parsec_odometry;
  nav_msgs::Odometry corrected_odometry;
  parsec_odometry.CorrectOdometry(
      uncorrected_odometry, transform, &corrected_odometry);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.y, 1.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.x,
                   transform.getRotation().x());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.y,
                   transform.getRotation().y());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.z,
                   transform.getRotation().z());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.w,
                   transform.getRotation().w());
}

// Tests that odometry is transformed correctly
TEST(ParsecOdometry, CorrectyOdometryWithTransform2) {
  nav_msgs::Odometry uncorrected_odometry;
  uncorrected_odometry.pose.pose.position.x = 2.0;
  uncorrected_odometry.pose.pose.position.y = 0.0;
  uncorrected_odometry.pose.pose.position.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.x = 0.0;
  uncorrected_odometry.pose.pose.orientation.y = 0.0;
  uncorrected_odometry.pose.pose.orientation.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.w = 1.0;
  tf::Transform transform(btQuaternion(btVector3(0, 0, 1), M_PI / 2),
                          btVector3(1, 0, 0));
  parsec_odometry::ParsecOdometry parsec_odometry;
  nav_msgs::Odometry corrected_odometry;
  parsec_odometry.CorrectOdometry(
      uncorrected_odometry, transform, &corrected_odometry);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.x,
                   transform.getRotation().x());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.y,
                   transform.getRotation().y());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.z,
                   transform.getRotation().z());
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.w,
                   transform.getRotation().w());
}

// Test that calculating the correction transform without a correction
// offset is correct.
TEST(ParsecOdometry, CalculateCorrectionTransformNoOffset) {
  nav_msgs::Odometry uncorrected_odometry;
  uncorrected_odometry.pose.pose.position.x = 1.0;
  uncorrected_odometry.pose.pose.position.y = 0.0;
  uncorrected_odometry.pose.pose.position.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.x = 0.0;
  uncorrected_odometry.pose.pose.orientation.y = 0.0;
  uncorrected_odometry.pose.pose.orientation.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry last_odometry;
  last_odometry.pose.pose.position.x = 2.0;
  last_odometry.pose.pose.position.y = 0.0;
  last_odometry.pose.pose.position.z = 0.0;
  last_odometry.pose.pose.orientation.x = 0.0;
  last_odometry.pose.pose.orientation.y = 0.0;
  last_odometry.pose.pose.orientation.z = 1.0;
  last_odometry.pose.pose.orientation.w = 0.0;

  parsec_odometry::ParsecOdometry parsec_odometry;

  tf::Transform offset = tf::Transform::getIdentity();
  tf::Transform correction;
  parsec_odometry.CalculateCorrectionTransform(
      last_odometry, offset, &correction);
  EXPECT_DOUBLE_EQ(correction.getOrigin().x(), 2);
  EXPECT_DOUBLE_EQ(correction.getOrigin().y(), 0);
  EXPECT_DOUBLE_EQ(correction.getOrigin().z(), 0);
  EXPECT_DOUBLE_EQ(correction.getRotation().x(), 0);
  EXPECT_DOUBLE_EQ(correction.getRotation().y(), 0);
  EXPECT_DOUBLE_EQ(correction.getRotation().z(), 1);
  EXPECT_DOUBLE_EQ(correction.getRotation().w(), 0);

  nav_msgs::Odometry corrected_odometry;
  parsec_odometry.CorrectOdometry(
      uncorrected_odometry, correction, &corrected_odometry);
  
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.w, 0.0);
}

// Tests that calculation of the correction transform including offset
// is correct
TEST(ParsecOdometry, CalculateCorrectionTransform) {
  nav_msgs::Odometry uncorrected_odometry;
  uncorrected_odometry.pose.pose.position.x = 1.0;
  uncorrected_odometry.pose.pose.position.y = 0.0;
  uncorrected_odometry.pose.pose.position.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.x = 0.0;
  uncorrected_odometry.pose.pose.orientation.y = 0.0;
  uncorrected_odometry.pose.pose.orientation.z = 0.0;  
  uncorrected_odometry.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry last_odometry;
  last_odometry.pose.pose.position.x = 2.0;
  last_odometry.pose.pose.position.y = 0.0;
  last_odometry.pose.pose.position.z = 0.0;
  last_odometry.pose.pose.orientation.x = 0.0;
  last_odometry.pose.pose.orientation.y = 0.0;
  last_odometry.pose.pose.orientation.z = 1.0;
  last_odometry.pose.pose.orientation.w = 0.0;

  parsec_odometry::ParsecOdometry parsec_odometry;

  tf::Transform offset = tf::Transform(
      btQuaternion(0, 0, 0, 1),
      btVector3(1, 0, 0));
  tf::Transform correction;
  parsec_odometry.CalculateCorrectionTransform(
      last_odometry, offset, &correction);
  EXPECT_DOUBLE_EQ(correction.getOrigin().x(), 1);
  EXPECT_DOUBLE_EQ(correction.getOrigin().y(), 0);
  EXPECT_DOUBLE_EQ(correction.getOrigin().z(), 0);
  EXPECT_DOUBLE_EQ(correction.getRotation().x(), 0);
  EXPECT_DOUBLE_EQ(correction.getRotation().y(), 0);
  EXPECT_DOUBLE_EQ(correction.getRotation().z(), 1);
  EXPECT_DOUBLE_EQ(correction.getRotation().w(), 0);
  
  // Odometry * offset for comparing the result
  nav_msgs::Odometry corrected_last_odometry;
  parsec_odometry.CorrectOdometry(
      last_odometry, offset, &corrected_last_odometry);

  nav_msgs::Odometry corrected_odometry;
  parsec_odometry.CorrectOdometry(
      uncorrected_odometry, correction, &corrected_odometry);

  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(corrected_odometry.pose.pose.orientation.w, 0.0);
}

void CheckCalculateCorrectionTransform(
    const tf::Transform &transform, const sensor_msgs::PointCloud2 &cloud,
    double max_linear_error, double max_rotational_error) {
  Eigen::Matrix4f transform_matrix;
  pcl_ros::transformAsMatrix(transform, transform_matrix);
  sensor_msgs::PointCloud2 transformed_cloud;
  pcl_ros::transformPointCloud(transform_matrix, cloud,
                               transformed_cloud);

  parsec_odometry::ParsecOdometry parsec_odometry;
  tf::Transform calculated_transform;
  EXPECT_TRUE(
      parsec_odometry.CalculateLaserCorrectionTransform(
          cloud, transformed_cloud, &calculated_transform));

  EXPECT_NEAR(transform.getOrigin().x(),
              calculated_transform.getOrigin().x(),
              max_linear_error);
  EXPECT_NEAR(transform.getOrigin().y(),
              calculated_transform.getOrigin().y(),
              max_linear_error);
  EXPECT_NEAR(transform.getOrigin().z(),
              calculated_transform.getOrigin().z(),
              max_linear_error);
  EXPECT_NEAR(transform.getRotation().x(),
              calculated_transform.getRotation().x(),
              max_rotational_error);
  EXPECT_NEAR(transform.getRotation().y(),
              calculated_transform.getRotation().y(),
              max_rotational_error);
  EXPECT_NEAR(transform.getRotation().z(),
              calculated_transform.getRotation().z(),
              max_rotational_error);
  EXPECT_NEAR(transform.getRotation().w(),
              calculated_transform.getRotation().w(),
              max_rotational_error);
}

TEST(ParsecOdometry, CalculateLaserCorrectionTransform) {
  for (int i = 0; i < 2; i++) {
    std::stringstream filename;
    filename << "test/scan_cloud_" << i << ".pcd";
    sensor_msgs::PointCloud2 laser_cloud;
    ASSERT_EQ(pcl::PCDReader().read(filename.str(), laser_cloud), 0);
    {
      SCOPED_TRACE("x: 0.1");
      CheckCalculateCorrectionTransform(
          btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.1, 0, 0)),
          laser_cloud, 1e-3, 1e-3);
    }
    {
      SCOPED_TRACE("x: -0.1");
      CheckCalculateCorrectionTransform(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.1, 0, 0)),
        laser_cloud, 4e-2, 1e-3);
    }
    {
      SCOPED_TRACE("y: 0.1");
      CheckCalculateCorrectionTransform(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0, 0.1, 0)),
        laser_cloud, 1e-3, 1e-3);
    }
    {
      SCOPED_TRACE("y: -0.1");
      CheckCalculateCorrectionTransform(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0, -0.1, 0)),
        laser_cloud, 1e-3, 1e-3);
    }
      {
      SCOPED_TRACE("y: 0.1");
      CheckCalculateCorrectionTransform(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0, 0.1, 0)),
        laser_cloud, 1e-3, 1e-3);
    }
    {
      SCOPED_TRACE("az: 22.5 Degrees");
      CheckCalculateCorrectionTransform(
        btTransform(btQuaternion(btVector3(0, 0, 1), M_PI / 8)
                    , btVector3(0.0, 0, 0)),
        laser_cloud, 1e-3, 1e-3);
    }
    {
      SCOPED_TRACE("az: -22.5 Degrees");
      CheckCalculateCorrectionTransform(
        btTransform(btQuaternion(btVector3(0, 0, 1), -M_PI / 8)
                    , btVector3(0.0, 0, 0)),
        laser_cloud, 1e-3, 1e-3);
    }
  }
}

int main(int argc, char *argv[]) {
  // The NodeHandle class wants us to initialize ros although we don't
  // really use it in the tests.
  ros::init(argc, argv, "parsec_odometry_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
