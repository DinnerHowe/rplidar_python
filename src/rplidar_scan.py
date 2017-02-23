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
# from list_ports_linux import *
from sensor_msgs.msg import LaserScan
from threading import Lock
import copy

deque_len = 360
raw_data = collections.deque(maxlen=deque_len)


class driver:

 def stop_device(self):
  rospy.loginfo('## stop device ##')
  cmd = stop
  self.command(cmd)
  self.port.setDTR(1)
  self.port.close()

 def defination(self):
  global deque_len
  self.maxlen = deque_len
  self.ResponseType={measurement:'measurement',devinfo:'devinfo',devhealth:'devhealth'}
  self.ResponseStatus={status_ok:'status_ok',status_warning:'status_warning',status_error:'status_error'}
  self.ResponseMode={SINGLE:'SINGLE', MULTI:'MULTI',UNDEFINED_f:'UNDEFINED',UNDEFINED_s:'UNDEFINED'}

  self.default_params()

  # self.LaserScan=LaserScan()
  # self.LaserScan.header.seq=0
  self.seq = 0
  self.accout=getpass.getuser()

  self.locker = Lock()

  rospy.loginfo( 'building topics') #Publisher
  if not rospy.has_param('~scan_topic'):
   rospy.set_param('~scan_topic','/scan')
  self.scan_topic=rospy.get_param('~scan_topic')

  if not rospy.has_param('~rate'):
   rospy.set_param('~rate', 10.0)
  self.frequency = rospy.get_param('~rate')

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
  self.frame=self.frame_default.copy()
  self.ranges= [i for i in self.ranges_default]
  self.intensive= [i for i in self.intensive_default]

 def default_params(self):
  self.frame_default = {}
  self.ranges_default = []
  self.intensive_default = []
  for i in range(361):
   self.frame_default['%s.0' % i] = []
   self.ranges_default.append(float('inf'))
   self.intensive_default.append(0.0)

 def rplidar_points(self):
  rospy.loginfo('rplidar single scan   %s'%hex(5))
  cmd = scan
  self.command(cmd)
  if self.header_check()==measurement:
   rate = rospy.Rate(self.frequency)
   while self.port.inWaiting() < response_device_point_format.sizeof():
    time.sleep(0.001)
   while not rospy.is_shutdown():
    _str = self.port.read(response_device_point_format.sizeof())
    response=response_device_point_format.parse(_str)
    synbit=response.quality.syncbit
    # start a new circle?
    if synbit and self.not_start:
      self.not_start=False
    # fill up raw data
    if not self.not_start:
     global raw_data
     raw_data.append(copy.deepcopy(_str))
     # release data
     if synbit:
      data_buff=list(raw_data)
      raw_data.clear()
      for i in range(len(data_buff)):
       PolorCoordinate=self.OutputCoordinate(data_buff[i])
       angle=PolorCoordinate[0]
       if str(angle) in self.frame:
        if not math.isinf(PolorCoordinate[1]):
         self.intensive[int(angle)]=PolorCoordinate[2]
         self.frame[str(angle)].append(copy.deepcopy(PolorCoordinate[1]))
         self.ranges[int(angle)]=round(numpy.mean(self.frame[str(angle)]),4)
       else:
        rospy.loginfo(str(angle)+str(self.frame))
      self.lidar_publisher(copy.deepcopy(self.ranges),copy.deepcopy(self.intensive))
      rate.sleep()
      self.port.flushOutput()
      self.rplidar_matrix()
      # self.frame = {}
      # self.ranges, self.intensive = [], []

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
  _Scan = LaserScan()
  _Scan.header.stamp=rospy.Time.now()
  _Scan.header.seq = self.seq
  self.seq +=1
  try:
   _Scan.header.frame_id=rospy.get_param("~frame_id")
  except:
   _Scan.header.frame_id="/camera_depth_frame" #default laser

  # rplidar_parameters
  _Scan.angle_max=numpy.pi-numpy.radians(0.0)
  _Scan.angle_min=numpy.pi-numpy.radians(359.0)
  _Scan.angle_increment=-numpy.radians(1.0)
  _Scan.time_increment=self.duration/360
  _Scan.scan_time=self.duration
  try:
   _Scan.range_min=rospy.get_param("range_min")#0.15
   _Scan.range_max=rospy.get_param("range_max")#6.0
  except:
   _Scan.range_min=0.15
   _Scan.range_max=6.0
  # rplidar_ranges
  _Scan.ranges=ranges
  _Scan.intensities=intensive
  if _Scan != LaserScan():
   pub_data = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
   pub_data.publish(_Scan)

if __name__ == "__main__":
 try:
  rospy.loginfo("initialization system")
  driver()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("unknown_detector node terminated.")

