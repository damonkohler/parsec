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

#include "laser_signal_filter/laser_signal_filter.h"

namespace laser_signal_filter {

LaserSignalFilter::LaserSignalFilter(const ros::NodeHandle &node_handle)
    : node_handle_(node_handle),
      increasing_enabled_(false),
      decreasing_enabled_(false) {
  signal_subscriber_ = node_handle_.subscribe<parsec_msgs::LaserTiltSignal>(
      "signal", 10, boost::bind(&LaserSignalFilter::SignalCallback, this, _1));
  profile_subscriber_ = node_handle_.subscribe<parsec_msgs::LaserTiltProfile>(
      "profile", 10, boost::bind(&LaserSignalFilter::ProfileCallback, this, _1));
  scan_subscriber_ = node_handle_.subscribe<sensor_msgs::LaserScan>(
      "scan", 10, boost::bind(&LaserSignalFilter::ScanCallback, this, _1));
  scan_republisher_ = node_handle_.advertise<sensor_msgs::LaserScan>(
      "filtered_scan", 10);
}

void LaserSignalFilter::EnableSignals(bool increasing, bool decreasing) {
  increasing_enabled_ = increasing;
  decreasing_enabled_ = decreasing;
}

void LaserSignalFilter::SignalCallback(
    const parsec_msgs::LaserTiltSignal::ConstPtr &signal) {
  last_signal_ = signal;
  if (last_scan_) {
    MaybeRepublishScan(last_scan_);
    last_scan_.reset();
  }
}

void LaserSignalFilter::ProfileCallback(
    const parsec_msgs::LaserTiltProfile::ConstPtr &profile) {
  last_profile_ = profile;
}

void LaserSignalFilter::ScanCallback(
    const::sensor_msgs::LaserScan::ConstPtr &scan) {
  if (!MaybeRepublishScan(scan)) {
    last_scan_ = scan;
  } else {
    last_scan_.reset();
  }
}

bool LaserSignalFilter::MaybeRepublishScan(
    const sensor_msgs::LaserScan::ConstPtr &scan) {
  if (IsEnabled(scan->header.stamp)) {
    scan_republisher_.publish(scan);
    return true;
  }
  return false;
}

bool LaserSignalFilter::IsEnabled(const ros::Time &time) {
  if (increasing_enabled_ && decreasing_enabled_) {
    return true;
  }
  if (!last_signal_ || !last_profile_) {
    return false;
  }
  if (increasing_enabled_ && last_signal_->signal ==
      parsec_msgs::LaserTiltSignal::ANGLE_INCREASING) {
    return time >= last_signal_->header.stamp &&
        time <= last_signal_->header.stamp + ros::Duration(
            last_profile_->increasing_duration);
  }
  if (decreasing_enabled_ && last_signal_->signal ==
      parsec_msgs::LaserTiltSignal::ANGLE_DECREASING) {
    return time >= last_signal_->header.stamp &&
        time <= last_signal_->header.stamp + ros::Duration(
            last_profile_->decreasing_duration);
  }
  return false;
}

}  // namespace laser_signal_filter
