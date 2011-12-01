#!/usr/bin/env python
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

"""Converts messages from the map topic to compressed images and publishes them."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import io
from PIL import Image

import roslib; roslib.load_manifest('map_to_compressed_image')
import rospy

import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs


_DEFAULT_IMAGE_FORMAT = 'png'
_UNKNOWN_COLOR = 128
_OCCUPIED_COLOR = 0
_FREE_COLOR = 255


class MapToCompressedImage(object):
  """Converts publishes map messages as compressed images.

  Incoming map messages are converted into images of the specified
  format. Please note that the output image is not transformed. The
  first entry in the map data corresponds to the top left pixel in the
  image, the second entry to the pixel (1, 0) and so on.
  """

  def __init__(self):
    self._format = rospy.get_param('~format', _DEFAULT_IMAGE_FORMAT)
    self._map_subscriber = rospy.Subscriber(
        'map', nav_msgs.OccupancyGrid, self.on_map_callback)
    self._image_metadata_publisher = rospy.Publisher(
        '~metadata', nav_msgs.MapMetaData)
    self._image_publisher = rospy.Publisher(
        '~compressed_map', sensor_msgs.CompressedImage)

  def on_map_callback(self, message):
    image = sensor_msgs.CompressedImage(
      header=message.header)
    image.format = self._format
    image.data = self.generate_image(
      self._format, message.info.width, message.info.height, message.data)
    self._image_publisher.publish(image)
    self._image_metadata_publisher.publish(message.info)

  def generate_image(self, format, width, height, data):
    input_data = io.BytesIO()
    for value in data:
      if value == -1:
        # map value unknown
        input_data.write(chr(_UNKNOWN_COLOR))
      elif value > 0:
        input_data.write(chr(_OCCUPIED_COLOR))
      else:
        input_data.write(chr(_FREE_COLOR))
    image = Image.frombuffer(
        'L', (width, height), input_data.getvalue(), 'raw', 'L', 0, 1)
    compressed_data = io.BytesIO()
    image.save(compressed_data, format)
    return compressed_data.getvalue()


def main():
  rospy.init_node('map_to_compressed_image')
  converter = MapToCompressedImage()
  rospy.spin()


if __name__ == '__main__':
  main()
