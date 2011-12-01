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

#include "median_queue.h"

#include <cstdlib>

#include <gtest/gtest.h>

TEST(MedianQueueTest, HappyCase) {
  MedianQueue<int, 5> q(100);
  EXPECT_EQ(100, q.GetMedian());
  q.PushValue(150);
  EXPECT_EQ(100, q.GetMedian());
  q.PushValue(160);
  EXPECT_EQ(100, q.GetMedian());
  q.PushValue(170);
  EXPECT_EQ(150, q.GetMedian());
  q.PushValue(90);
  EXPECT_EQ(150, q.GetMedian());
  q.PushValue(-8000);
  EXPECT_EQ(150, q.GetMedian());
  q.PushValue(-8000);
  EXPECT_EQ(90, q.GetMedian());
}

TEST(MedianQueueTest, QueueSizeOne) {
  MedianQueue<int, 1> q(100);
  EXPECT_EQ(100, q.GetMedian());
  for (int i = -10; i != 10; ++i) {
    q.PushValue(i);
    EXPECT_EQ(i, q.GetMedian());
  }
}

TEST(MedianQueueTest, HappyCaseEvenQueueSize) {
  MedianQueue<int, 4> q(100);
  EXPECT_EQ(100, q.GetMedian());
  q.PushValue(150);
  q.PushValue(150);
  q.PushValue(200);
  EXPECT_EQ(150, q.GetMedian());
}

TEST(MedianQueueTest, QueueDoesNotCrash) {
  MedianQueue<int, 5> q(0);
  srand(1234);
  for (int i = 0; i != 1000; ++i) {
    q.PushValue(rand());
    EXPECT_LE(0, q.GetMedian());
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
