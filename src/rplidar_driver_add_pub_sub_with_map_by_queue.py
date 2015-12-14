#!/usr/bin/env python  
""" 
- Version 1.0 2015/17/11

sub and generate map

this code subscribe to rplidar_driver_add_pub by casting array form txt
    

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy,Queue,threading,numpy
from rplidar_python.msg import rplidar_point
from sensor_msgs.msg import LaserScan


def data_gen(x_list=[],ylist=[]):
 rospy.init_node('rplidar_driver_xy_sub', anonymous=False)
 #rospy.Subscriber("rplidar_driver_xy", rplidar_point, callback_xy) #xy
 rospy.Subscriber('scan', LaserScan, callback_polo) #polo


def init():
 ax.set_title('rplidar map')
 ax.set_xlabel('x value in meter')
 ax.set_ylabel('y value in meter')
 ax.set_xlim([-5, 5])
 ax.set_ylim([-5, 5])
 line.set_data(x, y)
 return line,

x_data=Queue.Queue()
y_data=Queue.Queue()

x_list,y_list=[],[]
x_polo,y_polo=[],[]
x=[]
y=[]

fig, ax = plt.subplots()
line, = ax.plot([],[],linestyle="none",marker=".",markersize=3,markerfacecolor="blue")
ax.grid()

#polo
def callback_polo(data):

 distance=list(data.ranges)
 angular=data.angle_max
 angular_increase=data.angle_increment
 for i in range(len(distance)):
  x_polo.append(distance[i]*numpy.sin(angular))
  y_polo.append(distance[i]*numpy.cos(angular))
  angular+=angular_increase
 x_data.put(x_polo)
 y_data.put(y_polo)

 x=x_data.get()
 y=y_data.get()
 ax.figure.canvas.draw()
 line.set_data(x,y)
 return line,

def callback_xy(data):
 point_arry=eval(data.point)
 print point_arry, len(point_arry)
 for i in range(len(point_arry)):
  x_list.append(point_arry[i][0]/1000.0)
  y_list.append(point_arry[i][1]/1000.0)
 x_data.put(x_list)
 y_data.put(y_list)

 x=x_data.get()
 y=y_data.get()
 ax.figure.canvas.draw()
 line.set_data(x,y)
 return line,

#ani = animation.FuncAnimation(fig, callback_xy, data_gen, blit=False, interval=100,repeat=False, init_func=init) #xy

ani = animation.FuncAnimation(fig, callback_polo, data_gen, blit=False, interval=100, repeat=False, init_func=init) #polo

ax.relim()
ax.autoscale_view()

plt.show()

