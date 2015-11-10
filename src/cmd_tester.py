#!/usr/bin/env python
# -*- coding:utf-8 -*-
""" 
- Version 2.0 2015/22/10-23/10   2015/26/10

before run this code pls install parse library and also install construct lib for protocol

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

from reference import *

import serial,numpy,os,time,rospy



class printting:

 def __init__(self,thread=True,i=0):
  self.ResponseType={measurement:'measurement',devinfo:'devinfo',devhealth:'devhealth'}
  self.ResponseStatus={status_ok:'status_ok',status_warning:'status_warning',status_error:'status_error'}
  self.ResponseMode={SINGLE:'SINGLE', MULTI:'MULTI',UNDEFINED_f:'UNDEFINED',UNDEFINED_s:'UNDEFINED'}

  rospy.init_node('cmd_tester', anonymous=False)
  rospy.loginfo( 'start connecting to port')
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

  if thread==False:
   self.port = serial.Serial("/dev/ttyUSB%d"%i,115200)
   
   self.port.flushInput()# discarding all flush input buffer contents
   self.port.setDTR(level=0)

   rospy.loginfo('clear buffer done\n\n\n\n')

   hardware=self.device_info_print()
   rospy.loginfo('device_info_print done\n\n\n\n')

   health=self.device_health_print()
   rospy.loginfo('%s'%self.ResponseStatus[health.status])
   rospy.loginfo('device_health_print done\n\n\n\n')

   if health.status!=status_ok:
    self.device_reset_print()
    rospy.loginfo('device_reset_print done\n\n\n\n')

   
   rospy.loginfo('starting subscribing sensor data')
   #while not rospy.is_shutdown():
   self.port.flushInput()# discarding all flush input buffer contents

   self.device_point_print()
   rospy.loginfo('device_point_print done \n\n\n\n')


   self.device_stop_print()
   rospy.loginfo('device_stop_print done \n\n\n\n')


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

 # 硬件信息
 def device_info_print(self):
  rospy.loginfo('##rplidar device_info##   %s'%hex(20))
  cmd = get_device_info
  self.command(cmd)
  if self.header_check()==devinfo:
   _str=self.port.read(response_device_info_format.sizeof())
   response_str=response_device_info_format.parse(_str)
   rospy.loginfo('device info:\n%s'%response_str)
   rospy.loginfo('command for device info: %s'%hex(cmd))
   return response_str
  else:
   rospy.loginfo('command for devinfo error or return value error')
   os.system('rosnode kill cmd_tester')
   
 #硬件状态
 def device_health_print(self):
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
 def device_reset_print(self):
  rospy.loginfo('## reset device ##')
  cmd = reset
  self.command(cmd)
  self.port.setDTR(level=1)
  time.sleep(0.2)  

 # stop
 def device_stop_print(self):
  rospy.loginfo('## stop device ##')
  cmd = stop
  self.command(cmd)
  self.port.setDTR(level=1)
  self.port.close()

 #单次扫描返回的数据
 def device_point_print(self):

  rospy.loginfo('##rplidar single scan##   %s'%hex(5))
  cmd = scan
  #cmd = force_scan
  coordinates=[]
  self.command(cmd)

  if self.header_check()==measurement:
   while self.port.inWaiting()< response_device_point_format.sizeof():
    time.sleep(0.001)
   _str = self.port.read(response_device_point_format.sizeof()) 
   coordinate=self.OutputCoordinate(cmd,_str)
   coordinates.append(coordinate)

  else:
   rospy.loginfo('command for rplidar single scan error or return value error')
   os.system('rosnode kill cmd_tester')



 def OutputCoordinate(self,cmd,_str):

  response=response_device_point_format.parse(_str)

  self.synbit=response.quality.syncbit
  self.synbit_inverse=response.quality.syncbit_inverse
  self.quality=response.quality.quality

  self.check_bit=response.angle_q6_lower.check_bit_c
  self.angular=((response.angle_q6_higher<<7)|response.angle_q6_lower.angle_q6_lower)/64.0
  self.distance=response.distance_q2/4.0

  self.x=self.distance*numpy.sin(self.angular)
  self.y=self.distance*numpy.cos(self.angular)

  rospy.loginfo('scan data: %s'%response)
  print "\n    syncbit:=1 表示新的一圈 360 度扫描的开始;\n    syncbit_inverse:0;\n    check_bit: 校验位,永远为 1;\n    quality: 采样点信号质量 与激光接收信号质量相关;\n    angle_q6: 15位， 测距点相对于 RPLIDAR 朝向夹角;\n    distance_q2: 16位， 测距点相对于 RPLIDAR 的距离(毫米单位) ;\n"

  rospy.loginfo('command for scan data: %s'%hex(cmd))

  print "\ncoordinate is (x,y): ",self.x,self.y,'\n'

  return [self.x,self.y]

   
if __name__ == "__main__":
 try:
  rospy.loginfo("initialization system")
  printting()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("unknown_detector node terminated.")

   
