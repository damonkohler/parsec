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

"""Calls the servo calibration routine."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import sys

import roslib; roslib.load_manifest('parsec_calibration')
import rospy

from parsec_calibration import servo_calibration_routine


def main():
  if len(rospy.myargv()) < 5:
    sys.stdout.write('Usage: %s <minimum angle> <maximum angle> <increasing duration> <decreasing duration>\n' %
                     rospy.myargv()[0])
    return
  rospy.init_node('calibrate_tilting_servo')
  calibration_routine = servo_calibration_routine.ServoCalibrationRoutine(
      float(rospy.myargv()[1]), float(rospy.myargv()[2]), float(rospy.myargv()[3]), float(rospy.myargv()[4]))
  calibration_routine.run()


if __name__ == '__main__':
  main()
