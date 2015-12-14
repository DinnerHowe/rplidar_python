#!/usr/bin/env python  
""" 

- Version 1.0 2015/10/30

sub and generate map

this code subscribe to rplidar_driver_add_pub by casting array form txt
    

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy,numpy,threading,getpass,time,Queue
from rplidar_python.msg import rplidar_point
from geometry_msgs.msg import Point
import matplotlib.pyplot  as plt
import matplotlib.animation as animation




class listener:

 def init_xy_plot(self):
  print " init_xy_plot "
  plt.ion()
  self.figure = plt.figure()

  self.ax = self.figure.add_subplot(111)

  self.line, = self.ax.plot([],[],
                                   linestyle="none",
                                   marker=".",
                                   #markersize=3,
                                   markerfacecolor="blue")

  self.ax.set_title('rplidar map')
  self.ax.set_xlabel('x value in meter')
  self.ax.set_ylabel('y value in meter')
  self.ax.set_xlim([-5, 5])
  self.ax.set_ylim([-5, 5])
  self.ax.grid()


 def get_data(self):
  self.account=getpass.getuser()
  self.array,self.matrixs,matrix=[],[],[]
  with open('/home/%s/mapdata/rplidar_frame.txt'%self.account,'r') as f:
  #with open('/home/%s/mapdata/frame_1.txt'%self.account,'r') as f: #for testing
   for line in f:
    if line[0]=='[':
     self.array=eval(line)
     print '\ncurrent:',len(eval(line))
     matrix=self.data_range(self.array)
     print 'matrix:',len(matrix),'x',len(matrix[0]),'y',len(matrix[1])

     #self.matrixs.append(matrix)
     #print 'matrixs:',len(self.matrixs),'\n'
  #return self.matrixs

 def data_range(self,datas):
  self.point=Point()
  x=[]
  y=[]
  for i in range(len(datas)):
   x.append(datas[i][0]/1000.0)
   y.append(datas[i][1]/1000.0)
  return [x,y]


 def update_xy_plot(self,data):

  self.line.set_xdata(data[0])
  self.line.set_ydata(data[1])
  #self.scatter=self.ax.scatter(data[0],data[1],alpha=0.5)# do not clear priorry data
  self.ax.relim()
  self.ax.autoscale_view()
  self.figure.canvas.draw()

 def __init__(self):
  print "start listening to rplidar"
  rospy.init_node('rplidar_driver_sub', anonymous=False)
  self.coulds=[]
  self.init_xy_plot()
  self.clouds=[]

  th=threading.Thread(target=self.get_data)
  th.start()

  self.coulds.append(self.get_data())

  #print 'lenth:',len(self.coulds)

  try:
   for i in range(len(self.coulds)):
    self.update_xy_plot(self.coulds[i])
    time.sleep(0.2)
  except KeyboardInterrupt:
   rospy.loginfo("CTRL-c pressed, exiting...")
   pass


if __name__ == '__main__':
 listener()
 
