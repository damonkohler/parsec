#!/usr/bin/env python
# Copyright 2011 Google Inc. All Rights Reserved.
# Author: whess@google.com (Wolfgang Hess)


heart_large = [
  '  xx xx  ',
  ' xxxxxxx ',
  'xxxxxxxxx',
  ' xxxxxxx ',
  '  xxxxx  ',
  '   xxx   ',
  '    x    ',
  '         ']

heart_small = [
  '         ',
  '  xx xx  ',
  ' xxxxxxx ',
  '  xxxxx  ',
  '   xxx   ',
  '    x    ',
  '         ',
  '         ']

def dump_bits(pattern):
  bits = [0] * len(pattern[0])
  for i in xrange(8):
    for j in xrange(len(pattern[0])):
       bits[j] |= ((1 << i) if pattern[i][j] != ' ' else 0)
  for data in bits:
    print '0x%02x,' % data,
  print


dump_bits(heart_large)
dump_bits(heart_small)
    
