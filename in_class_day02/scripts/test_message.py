#!/usr/bin/env python

""" This script explores publishing ROS messages in ROS using Python """

from std_msgs.msg import Header
from geometry_msgs.msg import Point

import rospy

from geometry_msgs.msg import PointStamped

rospy.init_node('test_message')

my_hdr = Header(stamp=rospy.Time.now(), frame_id="odom")
my_point = Point(1.0, 2.0, 3.0)

my_pt_stamped = PointStamped(header=my_hdr, point=my_point)

print my_pt_stamped

