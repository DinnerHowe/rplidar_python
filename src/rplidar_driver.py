#!/usr/bin/env python
# -*- coding:utf-8 -*-
""" 
- Version 1.0 2015/17/11
before run this code pls install parse library and also install construct lib for protocol

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

from reference import *
import serial,numpy,os,time,rospy,getpass
from types import *
import collections 


class driver:

 def stop_device(self):
  rospy.loginfo('## stop device ##')
  cmd = stop
  self.command(cmd)
  self.port.setDTR(1)
  self.port.close()

 def defination(self,maxlen=360):
  self.ResponseType={measurement:'measurement',devinfo:'devinfo',devhealth:'devhealth'}
  self.ResponseStatus={status_ok:'status_ok',status_warning:'status_warning',status_error:'status_error'}
  self.ResponseMode={SINGLE:'SINGLE', MULTI:'MULTI',UNDEFINED_f:'UNDEFINED',UNDEFINED_s:'UNDEFINED'}
  self.raw_data=collections.deque(maxlen=maxlen)
  self.coordinates=collections.deque(maxlen=maxlen)
  self.circle=0
  self.accout=getpass.getuser()

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
  rospy.loginfo( 'perparing for system parameters')
  self.defination()

  rospy.loginfo( 'building node cmd_tester')
  rospy.init_node('rplidar_driver', anonymous=False)

  rospy.loginfo( 'start connecting to port')
  port=self.port_finder(i,thread)

  if port[1]==False:
   self.port = serial.Serial("/dev/ttyUSB%d"%port[0],115200)
   
   self.port.flushInput()# discarding all flush input buffer contents
   self.port.setDTR(0)
   rospy.loginfo('clear buffer done\n\n\n\n')

   health=self.device_health()
   rospy.loginfo('%s'%self.ResponseStatus[health.status])
   rospy.loginfo('device_health done\n\n\n\n')

   if health.status!=status_ok:
    self.driver_reset()
    rospy.loginfo('driver_reset done\n\n\n\n')

   rospy.loginfo('starting subscribing sensor data')
   self.rplidar_points()
   rospy.loginfo('rplidar_points done \n\n\n\n')
   self.stop_device()

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
  self.port.setDTR(1)
  time.sleep(0.2)  

 def rplidar_points(self):

  rospy.loginfo('##rplidar single scan##   %s'%hex(5))
  cmd = scan
  #coordinates=[]
  self.command(cmd)

  if self.header_check()==measurement:
   #while KeyboardInterrupt:
   while not rospy.is_shutdown():
    while self.port.inWaiting()< response_device_point_format.sizeof():
     time.sleep(0.001)
    _str = self.port.read(response_device_point_format.sizeof())
    self.raw_data.append(_str) 
    coordinate=self.OutputCoordinate(cmd,self.raw_data.popleft() )
    #print coordinate
    self.coordinates.append(coordinate)
    if self.synbit:
     print self.coordinates,'\n',len(self.coordinates),'\n'
     self.save(self.coordinates)
     self.coordinates.clear()
    else:
     self.circle+=1
     continue

  else:
   rospy.loginfo('command for rplidar single scan error or return value error')
   os.system('rosnode kill cmd_tester')


 def save(self,data):
  save_data=list(data)
  filename = open('/home/%s/mapdata/rplidar_frame.txt'%self.accout,'a')
  try:
   filename.writelines('circle number: %d\npioints data:\n'%self.circle)
   filename.writelines(str(save_data))
   filename.writelines('\n\n')
  finally:
   filename.close()

 def OutputCoordinate(self,cmd,raw):


  response=response_device_point_format.parse(raw)

  self.synbit=response.quality.syncbit
  self.synbit_inverse=response.quality.syncbit_inverse
  self.quality=response.quality.quality

  self.check_bit=response.angle_q6_lower.check_bit_c
  self.angular=((response.angle_q6_higher<<7)|response.angle_q6_lower.angle_q6_lower)/64.0
  self.distance=response.distance_q2/4.0

  self.x=self.distance*numpy.sin(self.angular)
  self.y=self.distance*numpy.cos(self.angular)

  #rospy.loginfo('scan data: %s'%response)
  #print "\ncoordinate is (x,y): ",self.x,self.y,'\n'

  return [self.x,self.y]

   
if __name__ == "__main__":
 try:
  rospy.loginfo("initialization system")
  driver()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("unknown_detector node terminated.")

   
