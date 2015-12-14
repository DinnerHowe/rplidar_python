#!/usr/bin/env python
# -*- coding:utf-8 -*-
""" 
- Version 1.0 2015/3/11

before run this code pls install parse library and also install construct lib for protocol

pub data in laserscan type

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

from reference import *
import serial,numpy,os,time,rospy,getpass,collections
from types import *  
from rplidar_python.msg import *
from std_msgs.msg import String

class driver:

 def defination(self,maxlen=360):
  self.ResponseType={measurement:'measurement',devinfo:'devinfo',devhealth:'devhealth'}
  self.ResponseStatus={status_ok:'status_ok',status_warning:'status_warning',status_error:'status_error'}
  self.ResponseMode={SINGLE:'SINGLE', MULTI:'MULTI',UNDEFINED_f:'UNDEFINED',UNDEFINED_s:'UNDEFINED'}
  self.raw_data=collections.deque(maxlen=maxlen)

  self.coordinates=[]
  self.angle=[]
  self.distance=[]
  #self.last=0.0
  self.circle=0
  self.count=0 #for saving

  self.rplidar_point=rplidar_point()
  self.rplidar_data=rplidar_data()
  self.rplidar_parameters=rplidar_parameters()

  self.accout=getpass.getuser()

  rospy.loginfo( 'building topics') #Publisher
  self.pub_point = rospy.Publisher('rplidar_point', rplidar_point, queue_size=20)
  self.pub_data=rospy.Publisher('rplidar_data',rplidar_data,queue_size=10)
  self.pub_parameters=rospy.Publisher('rplidar_parameters',rplidar_parameters,queue_size=10)

 def port_finder(self,i,thread):
  while thread and i<10:
   try:
    rospy.loginfo('evaluating /dev/ttyUSB%d'%i)
    test = serial.Serial("/dev/ttyUSB%d"%i,115200)
   except IOError, e:
    rospy.loginfo(e)
    i+=1
    continue
   except KeyboardInterrupt ,e:
    rospy.loginfo(e)
    break
   else:
    thread = False
   finally:
    rospy.loginfo( 'round %d port checking done'%i)
  return [i,thread]

 def __init__(self,thread=True,i=0):
  rospy.loginfo( 'building node rplidar_tester')
  rospy.init_node('rplidar_tester', anonymous=False)

  rospy.loginfo( 'perparing for system parameters')
  self.defination()

  rospy.loginfo( 'start connecting to port')
  port=self.port_finder(i,thread)

  if port[1]==False:
   self.port = serial.Serial("/dev/ttyUSB%d"%port[0],115200)
   self.port.flushInput()# discarding all flush input buffer contents
   self.port.flushOutput()# discarding all flush input buffer contents
   self.port.setDTR(0)
   rospy.loginfo('clear buffer done\n\n\n\n')

   health=self.device_health()
   rospy.loginfo('%s'%self.ResponseStatus[health.status])
   rospy.loginfo('device_health done\n\n\n\n')

   if health.status!=status_ok:
    self.driver_reset()
    rospy.loginfo('driver_reset done\n\n\n\n')

   self.current=rospy.Time.now()
   self.rplidar_points()
   rospy.loginfo('rplidar_points done \n\n\n\n')


 #发送命令
 def command(self,com):
  rospy.loginfo('sending commands')
  command=com
  cmd=command_format.build(Container(sync_byte=sync_byte, cmd_flag=command))
  self.port.write(cmd)
  rospy.loginfo('connect device on port: %s'%self.port.portstr)

 #返回头字节
 def header_check(self):
  rospy.loginfo('start evaluating header')
  stamp=time.time()
  time_out=1
  #waiting for response
  while time.time() < stamp+time_out:
   if self.port.inWaiting() < response_header_format.sizeof():
    time.sleep(0.01)

   else:
    _str = self.port.read(response_header_format.sizeof()) 
    response_str=response_header_format.parse(_str)

    rospy.loginfo('return data stream header checking result:\n')

    rospy.loginfo('\ninitial response bytes(0XA5 0X5A): %s %s\n'%(hex(response_str.sync_byte1).upper(),hex(response_str.sync_byte2).upper()))
    rospy.loginfo('response_size: %s\n'%hex(response_str.response.response_size))
    rospy.loginfo('response_data: %s'%hex(response_str.response.response_data))
    rospy.loginfo('response_mode: %s\n'%self.ResponseMode[response_str.response.response_mode])
    rospy.loginfo('response_type: %s\n'%self.ResponseType[response_str.response_type])

    if response_str.sync_byte1 != sync_byte1 or response_str.sync_byte2 != sync_byte2:
     rospy.loginfo( 'unexpect response header')
     os.system('rosnode kill cmd_tester')
    else:
     return response_str.response_type
  rospy.loginfo("time out") 

   
 #硬件状态
 def device_health(self):
  rospy.loginfo('##device_health##   %s'%hex(3))
  cmd = get_device_health
  self.command(cmd)
  if self.header_check()==devhealth:
   _str = self.port.read(response_device_health_format.sizeof()) 
   response_str=response_device_health_format.parse(_str)
   rospy.loginfo('rplidar device health:\n%s'%response_str)
   rospy.loginfo('command for device health: %s'%hex(cmd))
   return response_str
  else:
   rospy.loginfo('command for devhealth error or return value error')
   os.system('rosnode kill cmd_tester')

 # reset
 def driver_reset(self):
  rospy.loginfo('## reset device ##')
  cmd = reset
  self.command(cmd)
  self.port.setDTR(level=1)
  time.sleep(0.2)  

 def rplidar_points(self):
  rospy.loginfo('##rplidar single scan##   %s'%hex(5))
  cmd = scan
  self.command(cmd)

  if self.header_check()==measurement:
   while not rospy.is_shutdown():
    while self.port.inWaiting()< response_device_point_format.sizeof():
     time.sleep(0.001)
     
    _str = self.port.read(response_device_point_format.sizeof())
    self.raw_data.append(_str)

    coordinate=self.OutputCoordinate(self.raw_data.popleft()) 
    self.x=coordinate[1]*numpy.sin(coordinate[0])
    self.y=coordinate[1]*numpy.cos(coordinate[0])

    self.coordinates.append([self.x,self.y])
    self.angle.append(coordinate[0])
    self.distance.append(coordinate[1])

    if self.synbit: #start a new circle?

     self.duration=(rospy.Time.now().secs-self.current.secs)+(rospy.Time.now().nsecs-self.current.nsecs)*(10**(-9))
     self.current=rospy.Time.now()
     self.circle+=1

    # rplidar_parameters
     self.rplidar_parameters.frame=self.circle
     self.rplidar_parameters.angle_max=numpy.pi-numpy.radians(0)
     self.rplidar_parameters.angle_min=numpy.pi-numpy.radians(359.0)
     self.rplidar_parameters.angle_increment=(self.rplidar_parameters.angle_max-self.rplidar_parameters.angle_min)/len(self.coordinates)
     self.rplidar_parameters.time_increment=self.duration/float(len(self.coordinates))
     self.rplidar_parameters.scan_time=self.duration
     self.rplidar_parameters.range_min=0.15
     self.rplidar_parameters.range_max=6.0
     print self.rplidar_parameters
     self.pub_parameters.publish(self.rplidar_parameters)

    # rplidar_point
     self.rplidar_point.frame=self.circle
     self.rplidar_point.point=str(self.coordinates)
     print self.coordinates,'\n'
     print 'coordinates len:',len(self.coordinates),self.circle,'\n'
     self.pub_point.publish(self.rplidar_point)
     self.coordinates=[]#empty stack

    # rplidar_data
     self.rplidar_data.frame=self.circle
     self.rplidar_data.angle=self.angle
     self.rplidar_data.distance=self.distance

     print 'angle length:',len(self.rplidar_data.angle),'distance length:',len(self.rplidar_data.distance),'point length:',self.rplidar_data.frame,'\n'

     self.pub_data.publish(self.rplidar_data)
     self.angle=[]
     self.distance=[]
    #self.save(self.coordinates)

    else:
     continue

  else:
   rospy.loginfo('command for rplidar single scan error or return value error')
   os.system('rosnode kill cmd_tester')

 def save(self,data):
  if self.circle%300==1:
   self.count+=1
  filename = open('/home/%s/mapdata/rplidar_test.txt'%self.accout,'a')
  try:
   filename.writelines('circle number: %d\npioints data:\n'%self.circle)
   filename.writelines(str(data))
   filename.writelines('\n\n')
  finally:
   filename.close( )

 def OutputCoordinate(self,raw):

  response=response_device_point_format.parse(raw)

  self.synbit=response.quality.syncbit
  self.synbit_inverse=response.quality.syncbit_inverse
  self.quality=response.quality.quality

  self.check_bit=response.angle_q6_lower.check_bit_c
  self.angular=((response.angle_q6_higher<<7)|response.angle_q6_lower.angle_q6_lower)/64.0
  self.dis=response.distance_q2/4.0

  return [self.angular,self.dis/1000]

   
if __name__ == "__main__":
 try:
  rospy.loginfo("initialization system")
  driver()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("unknown_detector node terminated.")
