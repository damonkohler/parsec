/*
 *  Created on: May 11, 2009
 *      Author: duhadway
 */


#include <boost/foreach.hpp>

#include "gmapping_offline/gmapping_offline.h"
#include "ros/param.h"
#include "rosbag/bag.h"
#include "rosbag/exceptions.h"
#include "rosbag/query.h"
#include "rosbag/view.h"

using namespace std;
using namespace ros;

namespace build_map
{

BuildMap::BuildMap() :
  gsp_(NULL),
  gsp_laser_(NULL),
  gsp_odom_(NULL),
  tf_(),
  inverted_laser_(false),
  first_scan_(true),
  scan_count_(0)
{
  tf_.setExtrapolationLimit(ros::Duration(0.03));

  gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  ROS_ASSERT(gsp_);
  
  param::param("/gmapping_offline/inverted_laser", inverted_laser_, false);
  param::param("/gmapping_offline/base_frame", base_frame_, string("base_link"));
  param::param("/gmapping_offline/laser_frame", laser_frame_, string("base_laser"));
  param::param("/gmapping_offline/odom_frame", odom_frame_, string("odom"));
  param::param("/gmapping_offline/laser_topic", laser_topic_, string("/base_scan"));

  // Parameters used by GMapping itself
  param::param("/gmapping_offline/maxUrange", maxUrange_, 80.0);
  param::param("/gmapping_offline/sigma", sigma_, 0.05);
  param::param("/gmapping_offline/kernelSize", kernelSize_, 1);
  param::param("/gmapping_offline/lstep", lstep_, 0.05);
  param::param("/gmapping_offline/astep", astep_, 0.05);
  param::param("/gmapping_offline/iterations", iterations_, 5);
  param::param("/gmapping_offline/lsigma", lsigma_, 0.075);
  param::param("/gmapping_offline/ogain", ogain_, 3.0);
  param::param("/gmapping_offline/lskip", lskip_, 0);
  param::param("/gmapping_offline/srr", srr_, 0.1);
  param::param("/gmapping_offline/srt", srt_, 0.2);
  param::param("/gmapping_offline/str", str_, 0.1);
  param::param("/gmapping_offline/stt", stt_, 0.2);
  param::param("/gmapping_offline/linearUpdate", linearUpdate_, 1.0);
  param::param("/gmapping_offline/angularUpdate", angularUpdate_, 0.5);
  param::param("/gmapping_offline/resampleThreshold", resampleThreshold_, 0.5);
  param::param("/gmapping_offline/particles", particles_, 30);
  param::param("/gmapping_offline/xmin", xmin_, -40.0);
  param::param("/gmapping_offline/ymin", ymin_, -40.0);
  param::param("/gmapping_offline/xmax", xmax_, 40.0);
  param::param("/gmapping_offline/ymax", ymax_, 40.0);
  param::param("/gmapping_offline/delta", delta_, 0.1);
  param::param("/gmapping_offline/llsamplerange", llsamplerange_, 0.01);
  param::param("/gmapping_offline/llsamplestep", llsamplestep_, 0.01);
  param::param("/gmapping_offline/lasamplerange", lasamplerange_, 0.005);
  param::param("/gmapping_offline/lasamplestep", lasamplestep_, 0.005);
}

BuildMap::~BuildMap()
{
  if (gsp_)
    delete gsp_;
  if (gsp_laser_)
    delete gsp_laser_;
  if (gsp_odom_)
    delete gsp_odom_;
}

bool BuildMap::initialize(const sensor_msgs::LaserScan& scan)
{
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<btTransform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;
  try
  {
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
//    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
    return false;
  }
  double yaw,pitch,roll;
  btMatrix3x3 mat =  laser_pose.getBasis();
  mat.getEulerZYX(yaw, pitch, roll);

  GMapping::OrientedPoint gmap_pose(laser_pose.getOrigin().x(),
                                    laser_pose.getOrigin().y(),
                                    yaw);

  ROS_INFO("laser's pose wrt base: %.3f %.3f %.3f",
            laser_pose.getOrigin().x(),
            laser_pose.getOrigin().y(),
            yaw);

  // The laser must be called "FLASER"
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         scan.ranges.size(),
                                         scan.angle_increment,
                                         gmap_pose,
                                         0.0,
                                         scan.range_max);
  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor("odom");
  ROS_ASSERT(gsp_odom_);

  double maxrange = scan.range_max;

  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);

  gsp_->setMatchingParameters(maxUrange_, maxrange, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,time(NULL));


  // Set up matcher
  double* laser_angles = new double[scan.ranges.size()];
  double theta = scan.angle_min;
  for(unsigned int i=0; i<scan.ranges.size(); i++)
  {
    laser_angles[i]=theta;
    theta += scan.angle_increment;
  }

  matcher_.setLaserParameters(scan.ranges.size(), laser_angles, gsp_laser_->getPose());
  delete[] laser_angles;

  matcher_.setlaserMaxRange(scan.range_max);
  matcher_.setusableRange(maxUrange_);
  matcher_.setgenerateMap(true);

  ROS_INFO("Initialization complete");

  return true;
}

bool BuildMap::addScan(const sensor_msgs::LaserScan& scan)
{
  GMapping::OrientedPoint gmap_pose;
  if(!getOdomPose(gmap_pose, scan.header.stamp))
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  if (inverted_laser_) {
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  } else {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  gsp_->processScan(reading);

  return true;
}


bool BuildMap::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (btTransform(btQuaternion(0,0,0),
                                           btVector3(0,0,0)), t, base_frame_);
  tf::Stamped<btTransform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_DEBUG("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw,pitch,roll;
  odom_pose.getBasis().getEulerZYX(yaw, pitch, roll);
  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

void BuildMap::tfHandler(tf::tfMessage::ConstPtr m) {
  for (unsigned int i=0; i < m->transforms.size(); i++) {
    tf::StampedTransform trans;
    tf::transformStampedMsgToTF(m->transforms[i], trans);
    tf_.setTransform(trans);
  }

  // Try to process any scans that failed last time
  while (!bad_scans_.empty()) {
    sensor_msgs::LaserScan scan = bad_scans_.front();
    bad_scans_.pop();

    if (first_scan_) {
      if (initialize(scan))
        first_scan_ = false;
      else
        continue;
    }
    ROS_DEBUG("Processing bad scan");
    addScan(scan);
  }

  while (!scans_.empty()) {
    sensor_msgs::LaserScan scan = scans_.front();
    scans_.pop();

    if (first_scan_) {
      if (initialize(scan))
        first_scan_ = false;
      else
        continue;
    }

    static int count = 0;
    count++;
    ROS_INFO("Processing %d/%d\t%d%%", count, scan_count_, (int)(100.0 * count / scan_count_));

    if (!addScan(scan)) {
      bad_scans_.push(scan);
    }
  }
}

bool BuildMap::processBag(string file_path) {
  try {
    rosbag::Bag bag(file_path);
    rosbag::View view(bag, rosbag::TopicQuery(laser_topic_));
    scan_count_ = view.size();
    view.addQuery(bag, rosbag::TopicQuery("/tf_message"));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
      if (scan != NULL) {
        scans_.push(*scan);
      }

      tf::tfMessage::ConstPtr transform = m.instantiate<tf::tfMessage>();
      if (transform != NULL) {
        tfHandler(transform);
      }
    }
    bag.close();
  } catch (rosbag::BagException exception) {
    return false;
  }

  return true;
}

bool BuildMap::saveMap(string file)
{
  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];

  /// @todo Dynamically determine bounding box for map
  GMapping::Point wmin(xmin_, ymin_);
  GMapping::Point wmax(xmax_, ymax_);

  GMapping::Point center;
  center.x=(wmin.x + wmax.x) / 2.0;
  center.y=(wmin.y + wmax.y) / 2.0;

  GMapping::ScanMatcherMap smap(center, wmin.x, wmin.y, wmax.x, wmax.y, delta_);

  GMapping::IntPoint imin = smap.world2map(wmin);
  GMapping::IntPoint imax = smap.world2map(wmax);

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f", n->pose.x, n->pose.y, n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher_.invalidateActiveArea();
    matcher_.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher_.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  FILE* out = fopen(file.c_str(), "w");
  fprintf(out, "P5\n# CREATOR: build_map.cpp %.3f m/pix\n%d %d\n255\n", delta_, imax.x - imin.x, imax.y - imin.y);

  for(int x = 0; x < smap.getMapSizeX(); x++) {
    for(int y = 0; y < smap.getMapSizeY(); y++) {
      GMapping::IntPoint p(imin.x + x, imin.y + y);
      double occ=smap.cell(p);
//      occ = (occ < 0) ? 0.5 : occ;
//      fputc(255-(int)(occ*254),out);
      if (occ < 0) {
        fputc(206, out);
      } else if (occ < 0.1) { //occ [0,0.1)
        fputc(254, out);
      } else if (occ > 0.65) { //occ (0.65,1]
        fputc(000, out);
      } else { //occ [0.1,0.65]
        fputc(206, out);
      }
    }
  }

  fclose(out);

  return true;
}

}

int main(int argc, char** argv){
  if (argc < 3) {
    printf("USAGE: build_map BAG MAP\n");
    return(1);
  }

  ros::init(argc, argv, "buildmap");
  build_map::BuildMap bm;

  if (! bm.processBag(argv[1])) {
    return(1);
  }

  if (!bm.saveMap(argv[2])) {
    return(1);
  }

  return(0);
}
