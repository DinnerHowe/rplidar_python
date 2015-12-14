#!/usr/bin/env python  
""" 

- Version 1.0 2015/3/11

this code subscribe to rplidar_scan_pub
angle

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify it

This programm is tested on kuboki base turtlebot. 

"""

import rospy 
from rplidar_python.msg import *
from sensor_msgs.msg import LaserScan

def callback(data):
 dic=[]
 point_arry=data.angle
 for i in range(len(data.angle)):
  dic.append([data.angle[i],data.distance[i]])
 #print point_arry
 #print '\nangle data lenth:',len(point_arry),data.frame,'\n'
 print '\nangle,distance data:\n'
 for i in range(len(dic)):
  print dic[i]
 print len(dic),data.frame,'\n'


def listener():
 print "start listening to rplidar"
 rospy.init_node('rplidar_data_angle_client', anonymous=False)
 rospy.Subscriber("rplidar_data", rplidar_data, callback)
 rospy.spin()

if __name__ == '__main__':
 listener()

 
