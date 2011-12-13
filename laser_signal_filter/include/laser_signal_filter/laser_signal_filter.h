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

#ifndef LASER_SIGNAL_FILTER_LASER_SIGNAL_FILTER_H
#define LASER_SIGNAL_FILTER_LASER_SIGNAL_FILTER_H

#include <ros/ros.h>

#include <parsec_msgs/LaserTiltProfile.h>
#include <parsec_msgs/LaserTiltSignal.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_signal_filter {

class LaserSignalFilter {
 public:
  LaserSignalFilter(const ros::NodeHandle &node_handle);
  void EnableSignals(bool increasing, bool decreasing);

 private:
  sensor_msgs::LaserScan::ConstPtr last_scan_;
  parsec_msgs::LaserTiltProfile::ConstPtr last_profile_;
  parsec_msgs::LaserTiltSignal::ConstPtr last_signal_;
  ros::NodeHandle node_handle_;
  ros::Subscriber signal_subscriber_;
  ros::Subscriber profile_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Publisher scan_republisher_;
  bool increasing_enabled_;
  bool decreasing_enabled_;

  void SignalCallback(const parsec_msgs::LaserTiltSignal::ConstPtr &signal);
  void ProfileCallback(const parsec_msgs::LaserTiltProfile::ConstPtr &callback);
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
  bool MaybeRepublishScan(const sensor_msgs::LaserScan::ConstPtr &scan);
  bool IsEnabled(const ros::Time &time);
};

}  // namespace laser_signal_filter

#endif  // LASER_SIGNAL_FILTER_LASER_SIGNAL_FILTER_H
