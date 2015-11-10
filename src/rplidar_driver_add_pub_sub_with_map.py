#!/usr/bin/env python  
""" 

- Version 1.0 2015/10/29

sub and generate map

this code subscribe to rplidar_driver_add_pub by threading
    

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy,numpy,threading
from rplidar_python.msg import rplidar_point
from geometry_msgs.msg import Point
import matplotlib.pyplot  as plt

#method of defining a thread

#class ros_sub(threading,Thread):
# a thread for subscriber
 #def __init__(self):
  #rospy.Subscriber("rplidar_driver", rplidar_point, self.callback)



class listener:

 def data_range(self,datas):
  self.point=Point()
  x=[]
  y=[]
  for i in range(len(datas)):
   x.append(datas[i][0]/1000.0)
   y.append(datas[i][1]/1000.0)
  return [x,y]

 def init_xy_plot(self):
  plt.ion()
  self.figure,self.ax = plt.subplots() #plt.figure(figsize=(6, 6),dpi=160,facecolor="w",edgecolor="k")
  self.ax = self.figure.add_subplot(111)
  self.line2d, = self.ax.plot([],[],linestyle="none",marker=".",markerfacecolor="blue"),#markersize=3)

  self.ax.set_title('rplidar map')
  self.ax.set_xlabel('x value in meter')
  self.ax.set_ylabel('y value in meter')
  self.ax.set_xlim([-5, 5])
  self.ax.set_ylim([-5, 5])
  self.ax.grid()

 def callback(self,data):
  self.point_arry=eval(data.point)
  self.clouds=self.data_range(self.point_arry)
  #self.update_xy_plot()
  #self.update_xy_plot(self.clouds)
  #print self.clouds
  return self.clouds

 def update_xy_plot(self,data):
  self.line2d.set_xdata(data[0])
  self.line2d.set_ydata(data[1])

 def __init__(self):
  print "start listening to rplidar"
  rospy.init_node('rplidar_driver_sub', anonymous=False)
  self.init_xy_plot()

  self.ax.relim()
  self.ax.autoscale_view()
  self.figure.canvas.draw()

if __name__ == '__main__':
 listener()
 rospy.spin()
 
