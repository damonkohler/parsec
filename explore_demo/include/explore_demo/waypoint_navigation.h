/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Google Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef WAYPOINT_NAVIGATION_H_
#define WAYPOINT_NAVIGATION_H_

#include <list>
#include <set>
#include <string>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <boost/thread/mutex.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <explore/AddGoal.h>
#include <explore/RemoveGoal.h>
#include <explore/SelectGoal.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <navfn/navfn_ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace waypoint_nav {

/**
 * @class WaypointNav
 * @brief A class that maintains a sequence of waypoints to follow.
 */
class WaypointNav {
public:
  /**
   * @brief  Constructor
   */
  WaypointNav();

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~WaypointNav();


  /**
   * @brief Waits for and executes waypoint commands.
   */
  void execute();

  /**
   * @brief Runs explore in a new thread and process ROS callbacks in the
   * calling thread by calling ros::spin.
   */
  void spin();

private:

  /**
   * @brief  Make a global plan.
   */
  void makePlan();

  /**
   * @brief Publish a list of goal positions.
   * @param publisher The publisher to use.
   * @param goals The list of goals.
   */
  template <class CONTAINER>
  void publishGoals(const ros::Publisher &publisher,
                    const CONTAINER &goals);

  /**
   * @brief  Publish the goal to which the robot is currently navigating.
   * @param  goal The goal to publish.
   */
  void publishGoal(const ros::Publisher &publisher,
                   const geometry_msgs::Pose& goal);

  /**
   * @ brief Publish markers for visualizing current navigation state.
   * @ publish_current_goal When true a marker highlighting the current_goal will be
   * published as well.
   * @ current_goal The goal the robot is currently pursuing.
   */
  void publishMarkers(bool publish_current_goal,
                      const geometry_msgs::Pose& current_goal);

  /**
   * @brief Callback indicating that the move_base action client is finished.
   */
  void reachedGoal(const actionlib::SimpleClientGoalState& status,
                   const move_base_msgs::MoveBaseResultConstPtr& result,
                   geometry_msgs::Pose goal);

  /**
   * @brief Callback indicating a custom goal should be added to the list of
   * current goals.
   */
  bool addGoalCallback(explore::AddGoal::Request &req, explore::AddGoal::Response &res);

  /**
   * @brief Callback indicating that this goal should removed from the list of
   * considered goals.
   */
  bool removeGoalCallback(explore::RemoveGoal::Request &req, explore::RemoveGoal::Response &res);

  /**
   * @brief Callback indicating that this goal should be selected as the next goal.
   */
  bool selectGoalCallback(explore::SelectGoal::Request &req, explore::SelectGoal::Response &res);

  /*
   * @brief Tests if two goals are near each other.
   */
  bool goalsNear(const geometry_msgs::Pose &a,
                 const geometry_msgs::Pose &b,
                 double dist_threshold);
  /**
   * @brief Tests if goal is near a goal in goals.
   */
  template <class CONTAINER>
  bool goalInList(const geometry_msgs::Pose& goal,
                  const CONTAINER& goals);

  /**
   * @brief Tests if goal is near a goal in goals.
   */
  template <class CONTAINER>
  bool goalNearList(const geometry_msgs::Pose& goal,
                    double dist_threshold,
                    const CONTAINER& goals);

  /**
   * @brief Removes goal from goals.
   */
  template <class CONTAINER>
  bool removeGoalInList(const geometry_msgs::Pose& goal,
                        CONTAINER& goals);

  /**
   * @brief Removes goal from goals.
   */
  template <class CONTAINER>
  bool removeGoalNearList(const geometry_msgs::Pose& goal,
                          double dist_threshold,
                          CONTAINER& goals);

  ros::NodeHandle node_;
  std::string robot_base_frame_;

  // move_base action client.
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

  // Distance threshold at which two goals are considered unique.
  double goal_distance_threshold_;

  geometry_msgs::Pose current_goal_;
  std::list<geometry_msgs::Pose> goals_;
  // Protects user_goals.
  boost::mutex goals_mutex_;

  struct PoseCompare {
    bool operator()(const geometry_msgs::Pose &a,
                    const geometry_msgs::Pose &b) {
      return (&a < &b);
    }
  };

  std::set<geometry_msgs::Pose, PoseCompare> failed_goals_;

  ros::ServiceServer add_goal_service_;
  ros::ServiceServer remove_goal_service_;
  ros::ServiceServer select_goal_service_;

  ros::Publisher goals_publisher_;
  ros::Publisher failed_goals_publisher_;
  ros::Publisher marker_publisher_;
  ros::Publisher marker_array_publisher_;

  // Frequency at which goals are updated.
  double planner_frequency_;

};

}

#endif
