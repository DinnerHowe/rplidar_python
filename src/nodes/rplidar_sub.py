#!/usr/bin/env python  
""" 

- Version 1.0 2015/10/27

this code subscribe to rplidar_driver_add_pub
    

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy 
from rplidar_python.msg import rplidar_point

def callback(data):
 
 point_arry=eval(data.point)
 print data, len(point_arry)

def listener():
 print "start listening to rplidar"
 rospy.init_node('rplidar_driver_sub', anonymous=False)
 rospy.Subscriber("rplidar_driver", rplidar_point, callback)
 rospy.spin()

if __name__ == '__main__':
 listener()

 
