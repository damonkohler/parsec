///////////////////////////////////////////////////////////////////////////////
// priority_mux is a ROS topic multiplexer that prioritize input topics.
//
// Copyright (C) 2011, Google
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////


#include <cstdio>
#include <vector>
#include <list>
#include "ros/console.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using namespace topic_tools;

static ros::NodeHandle *g_node = NULL;
static string g_output_topic;
static ros::Publisher g_pub;
static bool g_advertised = false;

struct sub_info_t
{
  std::string topic_name;
  ros::Time last_received;
  ros::Subscriber *sub;
  ShapeShifter* msg;
};

static vector<struct sub_info_t> g_subs;
static size_t g_selected = 0;
static double g_timeout_sec = 1.0;

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg, size_t idx)
{
  if (!g_advertised)
  {
    ROS_INFO("advertising");
    g_pub = msg->advertise(*g_node, g_output_topic, 10);
    g_advertised = true;
  }

  ros::Time n = ros::Time::now();
  sub_info_t& current_sub = g_subs[idx];
  current_sub.last_received = n;
  if (idx <= g_selected) {
    g_selected = idx;
    g_pub.publish(msg);
  } else {
    bool any_recent_higher_priority_topics = false;
    for (int i=0; i < idx; ++i) {
      const sub_info_t& sub = g_subs[i];
      ros::Duration interval = n - sub.last_received;
      if (interval.toSec() <= g_timeout_sec) {
        any_recent_higher_priority_topics = true;
        break;
      }
    }

    if (!any_recent_higher_priority_topics) {
      g_selected = idx;
      g_pub.publish(msg);
    }
  }
}

int main(int argc, char **argv)
{
  vector<string> args;
  ros::removeROSArgs(argc, (const char**)argv, args);

  if (args.size() < 3)
  {
    printf("\nusage: priority_mux OUT_TOPIC IN_TOPIC1 [IN_TOPIC2 [...]]\n\n");
    return 1;
  }
  std::string topic_name;
  if(!getBaseName(args[1], topic_name))
    return 1;
  ros::init(argc, argv, topic_name + string("_mux"),
            ros::init_options::AnonymousName);
  vector<string> topics;
  for (unsigned int i = 2; i < args.size(); i++) {
    string topic = args[i];
    topics.push_back(args[i]);
  }
  ros::NodeHandle n;
  g_node = &n;
  g_output_topic = args[1];

  ros::NodeHandle mux_nh("priority_mux"), pnh("~");
  mux_nh.param("timeout", g_timeout_sec, 1.0);

  for (size_t i = 0; i < topics.size(); i++)
  {
    struct sub_info_t sub_info;
    sub_info.msg = new ShapeShifter;
    sub_info.topic_name = ros::names::resolve(topics[i]);
    sub_info.sub = new ros::Subscriber(n.subscribe<ShapeShifter>(sub_info.topic_name, 10, boost::bind(in_cb, _1, i)));
    sub_info.last_received = ros::TIME_MIN;
    g_subs.push_back(sub_info);
  }
  g_selected = 0; // select the highest priority topic to start

  ros::spin();
  for (vector<struct sub_info_t>::iterator it = g_subs.begin();
       it != g_subs.end();
       ++it)
  {
    if (it->sub)
      it->sub->shutdown();
    delete it->sub;
    delete it->msg;
  }

  g_subs.clear();
  return 0;
}
