/*
 * Copyright 2011 Google Inc.
 * Author: moesenle@google.com (Lorenz Moesenlechner)
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class TfOdometryRelay {
 public:
  TfOdometryRelay(const ros::NodeHandle &nh);

 private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  ros::Publisher odom_publisher_;
  ros::Rate odom_publish_rate_;
  ros::Duration velocity_averaging_interval_;
  std::string odom_frame_;
  std::string base_frame_;
  boost::thread odom_publisher_thread;

  void odomPublisher();
};

TfOdometryRelay::TfOdometryRelay(const ros::NodeHandle &nh)
  : nh_(nh),
    tf_(nh),
    odom_publish_rate_(10) {
  odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  
  double odom_publish_rate;
  nh_.param<double>("odom_publish_rate", odom_publish_rate, 10);
  odom_publish_rate_ = ros::Rate(odom_publish_rate);

  double velocity_averaging_interval;
  nh_.param<double>("velocity_averaging_interval", velocity_averaging_interval, 0.15);
  velocity_averaging_interval_ = ros::Duration(velocity_averaging_interval);

  nh_.param<std::string>("odom_frame", odom_frame_, "odom");
  nh_.param<std::string>("base_frame", base_frame_, "base_link");
  
  odom_publisher_thread = boost::thread(boost::bind(&TfOdometryRelay::odomPublisher, this));
}

void TfOdometryRelay::odomPublisher() {
  while (nh_.ok()) {
    try {
      geometry_msgs::Twist twist;
      tf::Point point(0, 0, 0);
      tf::StampedTransform odom_transform;

      tf_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), odom_transform);
      tf_.lookupTwist(base_frame_, odom_frame_, odom_frame_, point, base_frame_,
        odom_transform.stamp_, velocity_averaging_interval_, twist);

      nav_msgs::Odometry odom;
      odom.header.stamp = odom_transform.stamp_;
      odom.header.frame_id = odom_transform.frame_id_;
      odom.child_frame_id = odom_transform.child_frame_id_;
      odom.pose.pose.position.x = odom_transform.getOrigin().getX();
      odom.pose.pose.position.y = odom_transform.getOrigin().getY();
      odom.pose.pose.position.z = odom_transform.getOrigin().getZ();
      btQuaternion q = odom_transform.getRotation();
      odom.pose.pose.orientation.x = q.getX();
      odom.pose.pose.orientation.x = q.getY();
      odom.pose.pose.orientation.x = q.getZ();
      odom.pose.pose.orientation.x = q.getW();
      odom.twist.twist = twist;

      odom_publisher_.publish(odom);
      
      odom_publish_rate_.sleep();
    } catch(tf::TransformException &e) {
      ROS_WARN("LookupException %s", e.what());
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tf_odometry_relay");
  ros::NodeHandle nh("~");
  TfOdometryRelay relay(nh);

  ros::spin();

  return 0;
}
