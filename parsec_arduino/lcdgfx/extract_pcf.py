#!/usr/bin/env python
# Copyright 2011 Google Inc. All Rights Reserved.
# Author: whess@google.com (Wolfgang Hess)
# Extract a 6x8 font from a pcf-file, rotate it and output as a C array to
# be suitable for display on an LCD.

import struct
import sys

lsbint32 = struct.Struct('<i')
msbint32 = struct.Struct('>i')
PCF_BYTE_MASK = 4
msb = False

fontfile = open('clR6x8.pcf')

def pread(offset, size):
  fontfile.seek(offset)
  assert fontfile.tell() == offset
  data = fontfile.read(size)
  assert len(data) == size
  return data

def read_lsbint32(offset):
  return lsbint32.unpack(pread(offset, 4))[0]

def read_format(offset):
  if read_lsbint32(offset) & PCF_BYTE_MASK:
    global msb
    msb = True
  
def read_int32(offset):
  return (msbint32 if msb else lsbint32).unpack(pread(offset, 4))[0]  

def extract_glyph(offset, padding):
  assert padding == 4
  data = [0, 0, 0, 0, 0, 0]
  for line in xrange(8):
    origline = read_int32(offset + line * padding) >> (32 - 6)
    for col in xrange(6):
      data[col] |= int(((origline >> (5 - col)) & 1) != 0) << line
  sys.stdout.write('{0x%02x, ' % data[0])
  sys.stdout.write('0x%02x, ' % data[1])
  sys.stdout.write('0x%02x, ' % data[2])
  sys.stdout.write('0x%02x, ' % data[3])
  sys.stdout.write('0x%02x, ' % data[4])
  sys.stdout.write('0x%02x},' % data[5])

assert pread(0, 4) == '\1fcp'
for toc_entry in (8 + 16 * i for i in xrange(read_lsbint32(4))):
  size = read_lsbint32(toc_entry + 8)
  offset = read_lsbint32(toc_entry + 12)
  if read_lsbint32(toc_entry) == 8:  # Type is PCF_BITMAPS
    read_format(offset)
    glyph_count = read_int32(offset + 4)
    glyph_offset = [read_int32(offset + 8 + 4 * i) for i in xrange(glyph_count)]
    assert glyph_count == 128
    sys.stdout.write('const unsigned char font_data[128][6] = {');
    format3 = read_lsbint32(offset) & 3
    padding = 2 ** format3
    # bitmap_size = read_int32(offset + 8 + 4 * glyph_count + 4 * format3)
    for byte_offset in glyph_offset:
      sys.stdout.write('\n    ')
      extract_glyph(offset + 24 + 4 * glyph_count + byte_offset, padding)
    sys.stdout.write('};\n')
