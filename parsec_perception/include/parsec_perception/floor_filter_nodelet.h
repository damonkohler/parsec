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

#ifndef PARSEC_PERCEPTION_FLOOR_FILTER_NODELET_H
#define PARSEC_PERCEPTION_FLOOR_FILTER_NODELET_H

#include <vector>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <pcl/ModelCoefficients.h>

#include "parsec_perception/floor_filter.h"

namespace parsec_perception {

class FloorFilterNodelet : public nodelet::Nodelet {
 public:
  FloorFilterNodelet()
      : Nodelet(), floor_filter_(NULL) {}
  ~FloorFilterNodelet() {
    delete floor_filter_;
  }

 private:
  FloorFilter *floor_filter_;
  virtual void onInit();
};

}  // namespace parsec_perception

#endif  // PARSEC_PERCEPTION_FLOOR_FILTER_NODELET_H
