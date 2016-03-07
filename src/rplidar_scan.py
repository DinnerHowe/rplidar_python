#!/usr/bin/env python
# -*- coding:utf-8 -*-
""" 
- Version 5.0 2015/11/12

before run this code pls install parse library and also install construct lib for protocol

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 
"""

from reference import *
import serial,numpy,os,time,rospy,getpass,collections,math
import list_ports_linux
from list_ports_linux import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class driver:

 def stop_device(self):
  rospy.loginfo('## stop device ##')
  cmd = stop
  self.command(cmd)
  self.port.setDTR(1)
  self.port.close()

 def defination(self,maxlen=360):
  self.maxlen=maxlen
  self.ResponseType={measurement:'measurement',devinfo:'devinfo',devhealth:'devhealth'}
  self.ResponseStatus={status_ok:'status_ok',status_warning:'status_warning',status_error:'status_error'}
  self.ResponseMode={SINGLE:'SINGLE', MULTI:'MULTI',UNDEFINED_f:'UNDEFINED',UNDEFINED_s:'UNDEFINED'}

  self.raw_data=collections.deque(maxlen=self.maxlen)

  self.LaserScan=LaserScan()
  self.LaserScan.header.seq=0
  self.accout=getpass.getuser()

  rospy.loginfo( 'building topics') #Publisher
  self.pub_data=rospy.Publisher('/scan',LaserScan,queue_size=10)

 def port_finder(self,trigger):
  ports = list(list_ports_linux.comports())
  for port in ports:
   if port[1]=='CJ9011A' or port[1]=='CP2102 USB to UART Bridge Controller':
    trigger = True
    rospy.loginfo( 'rplidar connect on port: %s'%port[0])
    return [port,trigger]
   else:
    port=[]
  return[port,trigger]

 def __init__(self,trigger=False):
  rospy.loginfo( 'building node rplidar')
  rospy.init_node('rplidar', anonymous=False)
  rospy.loginfo( 'perparing for system parameters')
  self.defination()
  self.not_start=True
  self.rplidar_matrix()

  rospy.loginfo( 'start connecting to port')
  port=self.port_finder(trigger)
  if port[1]:
   self.port = serial.Serial("%s"%port[0][0],115200)
   self.port.flushInput()# discarding all flush input buffer contents

   rospy.loginfo('clear buffer done\n\n\n\n')

   health=self.device_health()
   rospy.loginfo('%s'%self.ResponseStatus[health.status])
   rospy.loginfo('device_health done\n\n\n\n')

   if health.status!=status_ok:
    self.driver_reset()
    rospy.loginfo('driver_reset done\n\n\n\n')
   self.port.setDTR(0)
   self.current=rospy.Time.now()
   self.rplidar_points()
   rospy.loginfo('rplidar_points done \n\n\n\n')
  else:
   rospy.loginfo('cannot find rplidar please connect rplidar on')
   pass 

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
  self.port.setDTR(1)
  time.sleep(0.001)  

 # start scanning
 def rplidar_matrix(self):
  self.frame={}
  self.ranges,self.intensive=[],[]
  for i in range(360):
   self.frame['%s.0'%i]=[]
   self.ranges.append(float('inf'))
   self.intensive.append(0.0)

 def rplidar_points(self):
  rospy.loginfo('rplidar single scan   %s'%hex(5))
  cmd = scan
  self.command(cmd)

  a=0
  
  if self.header_check()==measurement:
   while not rospy.is_shutdown():
    # checking buff len
    while self.port.inWaiting()< response_device_point_format.sizeof():
     time.sleep(0.001)
     
    _str = self.port.read(response_device_point_format.sizeof()) 
    response=response_device_point_format.parse(_str)
    self.synbit=response.quality.syncbit

    # start a new circle?
    if self.synbit and self.not_start:
      self.not_start=False

    # fill up raw data
    if not self.not_start:
     self.raw_data.append(_str)
     
    # release data
    if self.synbit and not self.not_start:
     self.data_buff=list(self.raw_data)
     self.raw_data.clear()
     
     for i in range(len(self.data_buff)):
      self.PolorCoordinate=self.OutputCoordinate(self.data_buff[i])
      #print "###",self.PolorCoordinate
      self.angle=self.PolorCoordinate[0]
      if str(self.angle) in self.frame:
       if not math.isinf(self.PolorCoordinate[1]):
        self.intensive[int(self.angle)]=self.PolorCoordinate[2]
        self.frame[str(self.angle)].append(self.PolorCoordinate[1])
        self.ranges[int(self.angle)]=numpy.mean(self.frame[str(self.angle)])
       else:
        pass
      else:
       pass
            
     self.lidar_publisher(self.ranges,self.intensive)
     self.rplidar_matrix() 
  else:
   rospy.loginfo('command for rplidar single scan error or return value error')
   os.system('rosnode kill cmd_tester')


 def OutputCoordinate(self,raw):
  response=response_device_point_format.parse(raw)

  inten=response.quality.quality
  angular=(response.angle_q6>>angle_shift)/64.0
  angle= round(angular)

  if response.distance_q2!=0:
   dis=response.distance_q2/4.0/1000.0
  else:
   dis=float('inf')

  return [angle,dis,inten]


 def lidar_publisher(self,ranges,intensive):
  self.duration=(rospy.Time.now().secs-self.current.secs)+(rospy.Time.now().nsecs-self.current.nsecs)*(10**(-9))
  self.current=rospy.Time.now()
  # header
  self.LaserScan.header.stamp=rospy.Time.now()
  self.LaserScan.header.seq+=1
  try:
   self.LaserScan.header.frame_id=rospy.get_param("~frame_id")
  except:
   self.LaserScan.header.frame_id="/camera_depth_frame" #default laser
      
  # rplidar_parameters
  self.LaserScan.angle_max=numpy.pi-numpy.radians(0.0)
  self.LaserScan.angle_min=numpy.pi-numpy.radians(359.0)
  self.LaserScan.angle_increment=-numpy.radians(1.0)
  self.LaserScan.time_increment=self.duration/360
  self.LaserScan.scan_time=self.duration
  try:
   self.LaserScan.range_min=rospy.get_param("range_min")#0.15
   self.LaserScan.range_max=rospy.get_param("range_max")#6.0
  except:
   self.LaserScan.range_min=0.15
   self.LaserScan.range_max=6.0
  # rplidar_ranges
  self.LaserScan.ranges=ranges
  self.LaserScan.intensities=intensive
  self.pub_data.publish(self.LaserScan)

if __name__ == "__main__":
 try:
  rospy.loginfo("initialization system")
  driver()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("unknown_detector node terminated.")

