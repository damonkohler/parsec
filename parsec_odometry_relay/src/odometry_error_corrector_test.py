#!/usr/bin/env python
#
# Copyright (C) 2011 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import roslib; roslib.load_manifest('parsec_odometry_relay')

import unittest
import odometry_error_corrector

import nav_msgs.msg as nav_msgs


class OdometryErrorCorrectorTest(unittest.TestCase):
  
  def test_find_enclosing_odometry_indices(self):
    error_corrector = odometry_error_corrector.OdometryErrorCorrector(0.1, 0.1)

    odometry_message_1 = nav_msgs.Odometry()
    odometry_message_1.header.stamp = 0

    odometry_message_2 = nav_msgs.Odometry()
    odometry_message_2.header.stamp = 1

    odometry_message_3 = nav_msgs.Odometry()
    odometry_message_3.header.stamp = 2
    
    error_corrector.add_odometry_message(odometry_message_1)
    error_corrector.add_odometry_message(odometry_message_2)
    error_corrector.add_odometry_message(odometry_message_3)

    self.assertEqual(
        error_corrector._find_enclosing_odometry_message_indices(0.5), (0, 1))
    self.assertEqual(
        error_corrector._find_enclosing_odometry_message_indices(1), (1, 2))
    self.assertEqual(
        error_corrector._find_enclosing_odometry_message_indices(1.5), (1, 2))

 
if __name__ == '__main__':
  unittest.main()
