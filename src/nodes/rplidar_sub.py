#!/usr/bin/env python  
""" 

- Version 1.0 2015/10/27

this code subscribe to rplidar_driver_add_pub
    

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy,Queue
from rplidar_python.msg import rplidar_point


def callback_xy(data):
 x_list,y_list=[],[]
 x_data=Queue.Queue()
 y_data=Queue.Queue()
 point_arry=eval(data.point)
 #print point_arry, len(point_arry)
 for i in range(len(point_arry)):
  x_list.append(point_arry[i][0])
  y_list.append(point_arry[i][1])
 print '\nx\n',x_list,'\ny\n',y_list
 x_data.put(x_list)
 y_data.put(y_list)

def callback_polor(data):
 angle_list,distance_list=[],[]
 angle_data=Queue.Queue()
 distance_data=Queue.Queue()
 point_arry=eval(data.point)
 #print point_arry, len(point_arry)
 for i in range(len(point_arry)):
  angle_list.append(point_arry[i][0])
  distance_list.append(point_arry[i][1])
 print '\nangle\n',angle_list,'\ndistance\n',distance_list
 angle_data.put(angle_list)
 distance_data.put(distance_list)

def listener():
 print "start listening to rplidar"
 rospy.init_node('rplidar_driver_sub', anonymous=False)
 rospy.Subscriber("rplidar_driver_xy", rplidar_point, callback_xy)
 rospy.Subscriber("rplidar_driver", rplidar_point, callback_polor)
 rospy.spin()

if __name__ == '__main__':
 listener()

 
