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

#ifndef RCCONSOLE_LOG_FILE_PRINTER_H
#define RCCONSOLE_LOG_FILE_PRINTER_H

#include <fstream>
#include <string>

#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>

#include "rcconsole/log_stream_printer.h"

namespace rcconsole {

class LogFilePrinter : public nodelet::Nodelet {
 public:
  virtual void onInit();
  
 private:
  static const std::string kDefaultFormatString;
  std::ofstream file_;
  boost::shared_ptr<LogStreamPrinter> stream_printer_;
};

}  // namespace rcconsole

#endif  // RCCONSOLE_LOG_FILE_PRINTER_H
