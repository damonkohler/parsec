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

#ifndef RCCONSOLE_EXCLUDE_MESSAGE_FILTER_H
#define RCCONSOLE_EXCLUDE_MESSAGE_FILTER_H

#include "rcconsole/log_filter.h"

namespace rcconsole {

class ExcludeMessageFilter : public LogFilter {
 protected:
  virtual bool FilterLog(const rosgraph_msgs::Log &log);
};

}  // namespace rcconsole

#endif  // RCCONSOLE_EXCLUDE_MESSAGE_FILTER_H
