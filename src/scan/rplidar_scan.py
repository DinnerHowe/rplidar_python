#!/usr/bin/env python
# -*- coding:utf-8 -*-
""" 
- Version 1.0 2015/3/11

before run this code pls install parse library and also install construct lib for protocol

pub data in laserscan type ver 1.0

多个frame发一次

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

from reference import *
import serial,numpy,os,time,rospy,getpass,collections
from types import *  
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class driver:

 def defination(self,maxlen=360):
  self.ResponseType={measurement:'measurement',devinfo:'devinfo',devhealth:'devhealth'}
  self.ResponseStatus={status_ok:'status_ok',status_warning:'status_warning',status_error:'status_error'}
  self.ResponseMode={SINGLE:'SINGLE', MULTI:'MULTI',UNDEFINED_f:'UNDEFINED',UNDEFINED_s:'UNDEFINED'}
  self.raw_data=collections.deque(maxlen=maxlen)

  self.LaserScan=LaserScan()
  self.LaserScan.header.seq=0
  self.accout=getpass.getuser()

  rospy.loginfo( 'building topics') #Publisher
  self.pub_data=rospy.Publisher('/scan',LaserScan,queue_size=10)

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
  self.rplidar_matrix()

  rospy.loginfo( 'start connecting to port')
  port=self.port_finder(i,thread)
  if port[1]==False:
   self.port = serial.Serial("/dev/ttyUSB%d"%port[0],115200)
   self.port.flushInput()# discarding all flush input buffer contents
   self.port.flushOutput()# discarding all flush input buffer contents
   self.port.setDTR(level=0)
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
  rospy.loginfo('device_health  %s'%hex(3))
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
  rospy.loginfo(' reset device ')
  cmd = reset
  self.command(cmd)
  self.port.setDTR(level=1)
  time.sleep(0.2)  


 def rplidar_matrix(self):
  self.frame={}
  self.frames={}
  self.circle=0
  self.ranges=[]
  self.intensities=[]
  for i in range(360):
   self.frames['%s.0'%i]=[[],0.0]
   self.frame['%s.0'%i]=[]
   self.ranges.append(None)
   self.intensities.append(0.0)

 def rplidar_points(self):
  rospy.loginfo('rplidar single scan   %s'%hex(5))
  cmd = scan
  self.command(cmd)

  if self.header_check()==measurement:
   while not rospy.is_shutdown():

  # checking buff
    while self.port.inWaiting()< response_device_point_format.sizeof():
     time.sleep(0.001)

  # raw data
    self.PolorCoordinate=[]
    _str = self.port.read(response_device_point_format.sizeof())
    self.raw_data.append(_str)
    #print 'raw data queue lenth test in:',len(self.raw_data)
    self.PolorCoordinate=self.OutputCoordinate(self.raw_data.popleft())
    #print 'raw data queue lenth test out:',len(self.raw_data)

  # classified one frame into matrix
    scanangle=round(self.PolorCoordinate[0])
    if scanangle==360.0:
     scanangle=0.0
    if str(scanangle) in self.frame:
     self.frame[str(scanangle)].append(self.PolorCoordinate[1])
     #print len(self.frame[str(scanangle)])
    else:
     print 'rotation angle error, sample angle not in matrix'
     break

  # start a new circle? 
    if self.synbit:
     self.circle+=1
     #classify data into matrix
     for i in self.frames:
      if self.frame[i]:
       self.frames[i][1]+=1.0
       self.frames[i][0].extend(self.frame[i])
       #print len(self.frames[i][0])
      else:
       pass
      #clear 
      for i in range(360):
       self.frame['%s.0'%i]=[]
    else:
     pass

  # publish?
    if self.circle==40:
     for i in self.frames:
      #print i,self.frames[i][0],type(self.frames[i][0])
      self.ranges[int(float(i))]=(numpy.mean(self.frames[i][0]))
      self.intensities[int(float(i))]=self.frames[i][1]
     self.lidar_publisher(self.ranges,self.intensities)    
    else:
     pass
  else:
   rospy.loginfo('command for rplidar single scan error or return value error')
   os.system('rosnode kill cmd_tester')
  self.rplidar_matrix() 

 def lidar_publisher(self,ranges,intensities):
  self.duration=(rospy.Time.now().secs-self.current.secs)+(rospy.Time.now().nsecs-self.current.nsecs)*(10**(-9))
  self.current=rospy.Time.now()
 # header
  self.LaserScan.header.stamp=rospy.Time.now()
  self.LaserScan.header.seq+=1
  self.LaserScan.header.frame_id='laser'
 # rplidar_parameters
  self.LaserScan.angle_max=numpy.pi-numpy.radians(0)
  self.LaserScan.angle_min=numpy.pi-numpy.radians(359.0)
  self.LaserScan.angle_increment=-numpy.radians(1.0)
  self.LaserScan.time_increment=self.duration/50.0
  self.LaserScan.scan_time=self.duration
  self.LaserScan.range_min=0.15
  self.LaserScan.range_max=6.0
 # rplidar_ranges/intensities
  self.LaserScan.ranges=ranges
  self.LaserScan.intensities=intensities
  self.pub_data.publish(self.LaserScan)

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
