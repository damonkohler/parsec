/*
 *  Created on: May 11, 2009
 *      Author: duhadway
 */

#ifndef BUILD_MAP_H_
#define BUILD_MAP_H_

#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include "gridfastslam/gridslamprocessor.h"
#include "sensor/sensor_base/sensor.h"

namespace build_map
{

class BuildMap
{
 public:
  BuildMap();
  ~BuildMap();

  bool processBag(std::string bag_path);
  bool saveMap(std::string file);

 private:
  void tfHandler(tf::tfMessage::ConstPtr m);

  GMapping::GridSlamProcessor* gsp_;
  GMapping::RangeSensor* gsp_laser_;
  GMapping::OdometrySensor* gsp_odom_;
  GMapping::ScanMatcher matcher_;
  tf::Transformer tf_;

  bool inverted_laser_;
  bool first_scan_;
  int scan_count_;

  std::string base_frame_;
  std::string laser_frame_;
  std::string odom_frame_;
  std::string laser_topic_;

  std::queue<sensor_msgs::LaserScan> scans_;
  std::queue<sensor_msgs::LaserScan> bad_scans_;

  bool initialize(const sensor_msgs::LaserScan& scan);
  bool addScan(const sensor_msgs::LaserScan& scan);
  bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
  void updateMap(const sensor_msgs::LaserScan& scan);

  // Parameters used by GMapping
  double maxUrange_;
  double maxrange_;
  double sigma_;
  int kernelSize_;
  double lstep_;
  double astep_;
  int iterations_;
  double lsigma_;
  double ogain_;
  int lskip_;
  double srr_;
  double srt_;
  double str_;
  double stt_;
  double linearUpdate_;
  double angularUpdate_;
  double resampleThreshold_;
  int particles_;
  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;
  double delta_;
  double llsamplerange_;
  double llsamplestep_;
  double lasamplerange_;
  double lasamplestep_;
};

}

#endif /* BUILD_MAP_H_ */
