/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <ros/ros.h>
#include <std_msgs/Time.h>

void PongCallback(const std_msgs::Time::ConstPtr &time_msg) {
  ROS_INFO("Received pong %lf", (ros::Time::now() - time_msg->data).toSec());
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "parsec_pinger");
  ros::NodeHandle nh;
  ros::Publisher pinger = nh.advertise<std_msgs::Time>("ping", 10);
  ros::Subscriber pong = nh.subscribe<std_msgs::Time>("pong", 10, &PongCallback);
  std_msgs::Time time_msg;
  ros::Rate ping_rate(1);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok()) {
    time_msg.data = ros::Time::now();
    pinger.publish(time_msg);
    ping_rate.sleep();
  }
  
  return 0;
}
