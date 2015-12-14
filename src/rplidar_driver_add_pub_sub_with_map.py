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

### the method of defining a thread (unfinished)

#class ros_sub(threading,Thread):
# a thread for subscriber
 #def __init__(self):
  #rospy.Subscriber("rplidar_driver", rplidar_point, self.callback)
 #def callback(self,data):
  #self.point_arry=eval(data.point)
  #self.clouds=self.data_range(self.point_arry)
  #self.update_xy_plot()
  #self.update_xy_plot(self.clouds)
  #print self.clouds
  #return self.clouds
 #def update_xy_plot(self,data):
  #self.line2d.set_xdata(data[0])
  #self.line2d.set_ydata(data[1])


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
  self.figure,self.ax = plt.subplots() 
  self.ax = self.figure.add_subplot(111)
  self.line2d, = self.ax.plot([],[],marker=".",markerfacecolor="blue",lw=2)

  self.ax.set_title('rplidar map')
  self.ax.set_xlabel('x value in meter')
  self.ax.set_ylabel('y value in meter')
  self.ax.set_xlim([-5, 5])
  self.ax.set_ylim([-5, 5])
  self.ax.grid()


 def __init__(self):
  print "start listening to rplidar"
  rospy.init_node('rplidar_driver_sub', anonymous=False)
  self.init_xy_plot()

  plt.show()
if __name__ == '__main__':
 listener()
 rospy.spin()
 
