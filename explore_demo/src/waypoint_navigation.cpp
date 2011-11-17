// Copyright 2011 Google Inc.
// Author: duhadway@google.com (Charles DuHadway)
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


#include "explore_demo/waypoint_navigation.h"
#include <geometry_msgs/PoseArray.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace explore;
using namespace navfn;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std;

namespace waypoint_nav {

double sign(double x){
  return x < 0.0 ? -1.0 : 1.0;
}

WaypointNav::WaypointNav() :
  node_(),
  move_base_client_("move_base") {
  ros::NodeHandle private_nh("~");

  goals_publisher_ = node_.advertise<geometry_msgs::PoseArray>("goals", 10);
  marker_publisher_ = node_.advertise<Marker>("visualization_marker",10);
  marker_array_publisher_ = node_.advertise<MarkerArray>("visualization_marker_array",10);

  add_goal_service_ = private_nh.advertiseService("add_goal", &WaypointNav::addGoalCallback, this);
  remove_goal_service_ = private_nh.advertiseService("remove_goal", &WaypointNav::removeGoalCallback, this);
  select_goal_service_ = private_nh.advertiseService("select_goal", &WaypointNav::selectGoalCallback, this);

  private_nh.param("planner_frequency", planner_frequency_, 1.0);
  private_nh.param("distance_threshold", goal_distance_threshold_, 0.15);
}

WaypointNav::~WaypointNav() {
}

bool WaypointNav::addGoalCallback(AddGoal::Request &req, AddGoal::Response &res)
{
  goals_mutex_.lock();
  goals_.push_back(req.goal);
  goals_mutex_.unlock();
  return true;
}

bool WaypointNav::removeGoalCallback(RemoveGoal::Request &req, RemoveGoal::Response &res)
{
  goals_mutex_.lock();
  removeGoalInList(req.goal, goals_);
  goals_mutex_.unlock();
  return true;
}

bool WaypointNav::selectGoalCallback(SelectGoal::Request &req, SelectGoal::Response &res)
{
  goals_mutex_.lock();
  removeGoalInList(req.goal, goals_);
  goals_.push_front(req.goal);
  goals_mutex_.unlock();
  return true;
}

template <class CONTAINER>
void WaypointNav::publishGoals(const ros::Publisher &publisher, const CONTAINER &goals) {
  if (publisher.getNumSubscribers()) {
    geometry_msgs::PoseArray poses;
    vector<Pose> goals_vec(goals.begin(), goals.end());
    poses.set_poses_vec(goals_vec);
    publisher.publish(poses);
  }
}

void WaypointNav::publishGoal(const ros::Publisher &publisher,
                              const Pose &goal) {
  if (publisher.getNumSubscribers()) {
    publisher.publish(goal);
  }
}

void WaypointNav::makePlan() {

  goals_mutex_.lock();
  if (goals_.size() == 0) {
    goals_mutex_.unlock();
    return;
  }

  Pose next_goal = goals_.front();
  goals_mutex_.unlock();

  if (!goalsNear(current_goal_, next_goal, goal_distance_threshold_)) {
    current_goal_ = next_goal;

    move_base_msgs::MoveBaseGoal move_goal;
    move_goal.target_pose.pose = next_goal;
    move_base_client_.sendGoal(move_goal, boost::bind(&WaypointNav::reachedGoal, this, _1, _2, next_goal));
  }

  publishGoals(goals_publisher_, goals_);
  publishGoals(failed_goals_publisher_, failed_goals_);
}

template <class CONTAINER>
bool WaypointNav::goalInList(const geometry_msgs::Pose& goal,
                             const CONTAINER& goals){
  return goalNearList(goal, goal_distance_threshold_, goals);
}

template <class CONTAINER>
bool WaypointNav::removeGoalInList(const geometry_msgs::Pose& goal,
                                   CONTAINER& goals){
  return removeGoalNearList(goal, goal_distance_threshold_, goals);
}

bool WaypointNav::goalsNear(const geometry_msgs::Pose &a,
                            const geometry_msgs::Pose &b,
                            double dist_threshold) {
  double x_diff = a.position.x - b.position.x;
  double y_diff = a.position.y - b.position.y;
  double dist = x_diff * x_diff + y_diff * y_diff;
  return (dist < (dist_threshold * dist_threshold));
}

template <class CONTAINER>
bool WaypointNav::goalNearList(const geometry_msgs::Pose& goal,
                           double dist_threshold,
                           const CONTAINER& goals) {
  for (typename CONTAINER::const_iterator it = goals.begin(); it != goals.end(); ++it) {
    if (goalsNear(goal, *it, dist_threshold)) {
      return true;
    }
  }
  return false;
}

template <class CONTAINER>
bool WaypointNav::removeGoalNearList(const Pose& goal,
                                     double dist_threshold,
                                     CONTAINER& goals) {
  for (typename CONTAINER::iterator it = goals.begin(); it != goals.end(); ++it) {
    if (goalsNear(goal, *it, dist_threshold)) {
      goals.erase(it);
      return true;
    }
  }
  return false;
}

void WaypointNav::reachedGoal(const actionlib::SimpleClientGoalState& status,
                              const move_base_msgs::MoveBaseResultConstPtr& result,
                              geometry_msgs::Pose goal) {

  ROS_DEBUG("Reached goal");
  if(status == actionlib::SimpleClientGoalState::ABORTED){
    failed_goals_.insert(goal);
    ROS_DEBUG("Adding current goal to failed goals.");
  }
}

void WaypointNav::execute() {
  while (! move_base_client_.waitForServer(ros::Duration(5,0)))
    ROS_WARN("Waiting to connect to move_base server");

  ROS_INFO("Connected to move_base server");

  // This call sends the first goal, and sets up for future callbacks.
  makePlan();

  ros::Rate r(planner_frequency_);
  while (node_.ok()) {
    makePlan();
    r.sleep();
  }

  move_base_client_.cancelAllGoals();
}

void WaypointNav::spin() {
  ros::spinOnce();
  boost::thread t(boost::bind( &WaypointNav::execute, this));
  ros::spin();
  t.join();
}

}

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_nav");

  waypoint_nav::WaypointNav nav;
  nav.spin();

  return(0);
}
