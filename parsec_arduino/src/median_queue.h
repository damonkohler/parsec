// Copyright 2011 Google Inc.
// Author: whess@google.com (Wolfgang Hess)
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

#ifndef PARSECLIB_MEDIAN_QUEUE_
#define PARSECLIB_MEDIAN_QUEUE_

#include <stdlib.h>

template<typename T, int N>
class MedianQueue {
 public:
  MedianQueue(T initial_value) {
    for (size_t i = 0; i != N; ++i) {
      values_[i] = initial_value;
    }
  }

  void PushValue(T new_value) {
    for (size_t i = 1; i != N; ++i) {
      values_[i - 1] = values_[i];
    }
    values_[N - 1] = new_value;
  }

  int GetMedian() const {
    for (int i = 0;; ++i) {
      size_t less_or_equal = 0, greater_or_equal = 0;
      for (int j = 0; j != N; ++j) {
        less_or_equal += (values_[j] <= values_[i]);
        greater_or_equal += (values_[j] >= values_[i]);
      }
      if (less_or_equal >= (N + 1) / 2 && greater_or_equal >= (N + 1) / 2) {
        return values_[i];
      }
    }
  }

 private:
  T values_[N];
};

#endif  // PARSECLIB_MEDIAN_QUEUE_
