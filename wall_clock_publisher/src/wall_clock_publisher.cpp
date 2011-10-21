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

/**
 * Make sure that each client gets the current time as soon as it
 * connects. Updates are then sent in the main thread at the specified
 * rate.
 */
void onClientConnect(const ros::SingleSubscriberPublisher &client_pub)
{
  std_msgs::Time time;
  time.data = ros::Time::now();
  client_pub.publish(time);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "wall_time_publisher");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinners(1);

  double publish_rate;
  // Per default, we publish only once every 5 minutes
  nh.param("publish_rate", publish_rate, 1.0/300.0);
  ros::Rate rate(publish_rate);

  ros::Publisher wall_clock_pub = nh.advertise<std_msgs::Time>("/wall_clock", 1,
    static_cast<const ros::SubscriberStatusCallback &>(onClientConnect));
  
  std_msgs::Time time;

  spinners.start();
  while(nh.ok())
  {
    time.data = ros::Time::now();
    wall_clock_pub.publish(time);
    rate.sleep();
  }
  
  return 0;
}
