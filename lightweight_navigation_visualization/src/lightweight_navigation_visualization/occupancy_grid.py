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

"""Provides functions for resizing and converting occupancy grids."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import io
from PIL import Image

import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs


DEFAULT_COLOR_UNKNOWN = 128
DEFAULT_COLOR_OCCUPIED = 0
DEFAULT_COLOR_FREE = 1


def _occupancy_to_bytes(
    data,
    color_unknown=DEFAULT_COLOR_UNKNOWN,
    color_free=DEFAULT_COLOR_FREE,
    color_occupied=DEFAULT_COLOR_OCCUPIED):
  for value in data:
    if value == -1:
      yield chr(color_unknown)
    elif value == 0:
      yield chr(color_free)
    else:
      yield chr(color_occupied)


def _bytes_to_occupancy(
    data,
    color_unknown=DEFAULT_COLOR_UNKNOWN,
    color_free=DEFAULT_COLOR_FREE,
    color_occupied=DEFAULT_COLOR_OCCUPIED):
  for value in data:
    if value == color_unknown:
      yield -1
    elif value == color_free:
      yield 0
    else:
      yield 100


def occupancy_grid_to_image(
    occupancy_grid,
    color_unknown=DEFAULT_COLOR_UNKNOWN,
    color_free=DEFAULT_COLOR_FREE,
    color_occupied=DEFAULT_COLOR_OCCUPIED):
  data_stream = io.BytesIO()
  for value in _occupancy_to_bytes(occupancy_grid.data, color_unknown,
                                   color_free, color_occupied):
    data_stream.write(value)
  return Image.fromstring(
      'L', (occupancy_grid.info.width, occupancy_grid.info.height),
      data_stream.getvalue())


def image_to_occupancy_grid_data(
    image,
    color_unknown=DEFAULT_COLOR_UNKNOWN,
    color_free=DEFAULT_COLOR_FREE,
    color_occupied=DEFAULT_COLOR_OCCUPIED):
  return _bytes_to_occupancy(
      image.getdata(), color_unknown, color_free, color_occupied)


def scale_occupancy_grid(occupancy_grid, resolution):
  """Scales an occupancy grid message.

  Takes an occupancy grid message, scales it to have the new size and
  returns the scaled grid.

  Parameters:
    occupancy_grid: the occupancy grid message to scale
    resolution: the resolution the scaled occupancy grid should have
  """
  image = occupancy_grid_to_image(occupancy_grid)
  scaling_factor = occupancy_grid.info.resolution / resolution
  new_width = int(occupancy_grid.info.width * scaling_factor)
  new_height = int(occupancy_grid.info.height * scaling_factor)
  resized_image = image.resize((new_width, new_height))
  result = nav_msgs.OccupancyGrid()
  result.header = occupancy_grid.header
  result.info.map_load_time = occupancy_grid.info.map_load_time
  result.info.resolution = resolution
  result.info.width = new_width
  result.info.height = new_height
  result.info.origin = occupancy_grid.info.origin
  result.data = list(image_to_occupancy_grid_data(resized_image))
  return result
