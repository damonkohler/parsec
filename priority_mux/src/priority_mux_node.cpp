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

#include "priority_mux/priority_mux.h"

namespace priority_mux {

bool Initialize(ros::NodeHandle &node_handle, PriorityMux &priority_mux) {
  XmlRpc::XmlRpcValue incoming_topics;
  if (!node_handle.getParam("incoming_topics", incoming_topics)) {
    ROS_FATAL("Parameter not found: incoming_topics");
    return false;
  }
  if (incoming_topics.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_FATAL("Parameter needs to be a list: incoming_topics");
    return false;
  }
  for (int i = 0; i < incoming_topics.size(); i++) {
    if (incoming_topics[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_FATAL("Parameter must be a string: incoming_topics[%d]", i);
      return false;
    }
    ROS_INFO("Adding topic: %s", static_cast<std::string>(incoming_topics[i]).c_str());
    priority_mux.AddTopic(incoming_topics[i]);
  }
  return true;
}

}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "priority_mux");
  ros::NodeHandle node_handle("~");
  priority_mux::PriorityMux priority_mux(node_handle);
  Initialize(node_handle, priority_mux);
  ros::spin();
  return 0;
}
