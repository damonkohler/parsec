#!/usr/bin/env python

import roslib; roslib.load_manifest('display_parsec_time_offsets')

import rospy
import sys

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def odom_callback(data):
    offset = data.header.stamp - rospy.Time.now()
    print 'odom offset', offset.to_sec()

def laser_callback(data):
    offset = data.header.stamp - rospy.Time.now()
    print 'laser offset', offset.to_sec()
    
rospy.init_node('display_time_offsets', anonymous=True)

odom_subscriber = None
laser_subscriber = None

if 'odom' in sys.argv[1:]:
    odom_subscriber = rospy.Subscriber('/odom', Odometry, odom_callback)
if 'laser' in sys.argv[1:]:
    laser_subscriber = rospy.Subscriber('/base_scan', LaserScan, laser_callback)

if odom_subscriber or laser_subscriber:
    rospy.spin()
else:
    print 'Usage: %r [odom] [laser]' % sys.argv[0]

