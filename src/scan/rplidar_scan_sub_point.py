#!/usr/bin/env python  
""" 

- Version 1.0 2015/3/11

this code subscribe to rplidar_scan_pub
    

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify it

This programm is tested on kuboki base turtlebot. 

"""

import rospy 
from rplidar_python.msg import rplidar_point
from sensor_msgs.msg import LaserScan

def callback(data):
 point_arry=eval(data.point)
 print data
 print 'data length:',len(point_arry),'\n'


def listener():
 print "start listening to rplidar"
 rospy.init_node('rplidar_point_client', anonymous=False)
 rospy.Subscriber("rplidar_point", rplidar_point, callback)
 rospy.spin()

if __name__ == '__main__':
 listener()

 
