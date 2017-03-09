#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""

before run this code pls install parse library and also install construct lib for protocol

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.
"""

from reference import *
import serial
import numpy
import rospy
import getpass
import collections
import math
from sensor_msgs.msg import LaserScan
import copy
import function
import subprocess

raw_data = collections.deque(maxlen=100)
para_data = collections.deque(maxlen=360)
compress_data = collections.deque(maxlen=10)
reset = False
start = False

class ClearParams:
    def __init__(self):
        rospy.logwarn('clearing parameters')
        rospy.delete_param('~scan_topic')
        rospy.delete_param('~rplidar_scan_rate')
        rospy.delete_param('~rplidar_pub_rate')
        rospy.delete_param('~rplidar_port_name')
        rospy.delete_param('~rplidar_frame')
        rospy.delete_param('~range_min')
        rospy.delete_param('~range_max')

class driver:
    def __init__(self):
        self.defination()
        self.begin()
        rospy.Timer(rospy.Duration(self.scan_frequency), self.Read_Data)
        rospy.Timer(rospy.Duration(self.scan_frequency/5.0), self.Resolve_Data)
        rospy.Timer(rospy.Duration(self.pub_frequency), self.lidar_publisher)
        rospy.spin()

    def defination(self):
        self.ResponseType = {measurement: 'measurement', devinfo: 'devinfo', devhealth: 'devhealth'}
        self.ResponseStatus = {status_ok: 'status_ok', status_warning: 'status_warning', status_error: 'status_error'}
        self.ResponseMode = {SINGLE: 'SINGLE', MULTI: 'MULTI', UNDEFINED_f: 'UNDEFINED', UNDEFINED_s: 'UNDEFINED'}

        # function.default_params(self)
        self.seq = 0
        self.accout = getpass.getuser()

        if not rospy.has_param('~scan_topic'):
            rospy.set_param('~scan_topic', '/scan')
        self.scan_topic = rospy.get_param('~scan_topic')

        if not rospy.has_param('~rplidar_scan_rate'):
            rospy.set_param('~rplidar_scan_rate', 0.0001)
        self.scan_frequency = rospy.get_param('~rplidar_scan_rate')

        if not rospy.has_param('~rplidar_pub_rate'):
            rospy.set_param('~rplidar_pub_rate', 0.01)
        self.pub_frequency = rospy.get_param('~rplidar_pub_rate')

        if not rospy.has_param('~rplidar_port_name'):
            rospy.set_param('~rplidar_port_name', 'CP2102 USB to UART Bridge Controller')
        self.rplidar_port_name = rospy.get_param('~rplidar_port_name')

        if not rospy.has_param('~rplidar_frame'):
            rospy.set_param('~rplidar_frame', '/camera_depth_frame')
        self.rplidar_frame = rospy.get_param('~rplidar_frame')

        if not rospy.has_param('~range_min'):
            rospy.set_param('~range_min', 0.15)
        self.range_min = rospy.get_param('~range_min')

        if not rospy.has_param('~range_max'):
            rospy.set_param('~range_max', 6.0)
        self.range_max = rospy.get_param('~range_max')

        finder = function.port_finder(False, self.rplidar_port_name)
        self.find_port = finder[1]
        if self.find_port:
            self.port_name = finder[0][1]
            self.port = serial.Serial("%s" % finder[0][0], 115200)

    def begin(self):
        global start
        if self.find_port:
            self.port.setDTR(1)
            rospy.logwarn("connect port: %s" %self.port_name)
            # self.port.flushInput()  # discarding all flush input buffer contents
            # self.port.flushOutput()
            health = function.device_health(self.port)
            try:
                rospy.loginfo('health status: %s'%self.ResponseStatus[health.status])
            except:
                rospy.logwarn('health status: %s' %health)
                rospy.signal_shutdown('health check error')
                self.port.close()
            if health != None:
                if health.status != status_ok:
                    function.driver_reset(self.port)
                else:
                    self.port.setDTR(0)
                    cmd = scan
                    function.send_command(self.port, cmd)
                    if function.header_check(self.port) == measurement:
                        start = True
                    else:
                        rospy.logerr('header check error,response type not measurement')
                        start = False
                        rospy.signal_shutdown('header check error')
        else:
            rospy.logwarn('Can NOT find rplidar please check rplidar connection')
            rospy.logwarn('Shut Down Progress')
            rospy.signal_shutdown('None connection')

    def Read_Data(self, event):
        global start
        if start:
            current = rospy.Time.now()
            global raw_data
            function.rplidar_points(self.port, raw_data)
            self.duration = (rospy.Time.now().secs - current.secs) + (rospy.Time.now().nsecs - current.nsecs) * (10 ** (-9))

    def Resolve_Data(self, event):
        global raw_data
        global reset
        global para_data
        if len(raw_data)>0:
            reset = False
            # print 'raw data length', len(raw_data)
            _str = raw_data.pop()
            response = response_device_point_format.parse(_str)
            synbit = response.quality.syncbit
            syncbit_inverse = response.quality.syncbit_inverse
            # release data
            if synbit and not syncbit_inverse:
                data_buff = list(para_data)
                ranges = function.range_matrix()
                intensive = function.intensive_matrix()
                for i in range(len(data_buff)):
                    PolorCoordinate = function.OutputCoordinate(data_buff[i])
                    if PolorCoordinate[0] >= 360:
                        print PolorCoordinate[0]
                        angle = PolorCoordinate[0]%360
                    else:
                        angle = PolorCoordinate[0]
                    if not math.isinf(PolorCoordinate[1]):
                        print angle
                        if math.isinf(ranges[int(angle)]):
                            ranges[int(angle)] = round(PolorCoordinate[1], 4)
                            intensive[int(angle)] = PolorCoordinate[2]
                        else:
                            ranges[int(angle)] = function.fusion([ranges[int(angle)], intensive[int(angle)]], PolorCoordinate)
                            intensive[int(angle)] = PolorCoordinate[2]
                global compress_data
                compress_data.append([ranges, []])
                para_data.clear()
                para_data.append(_str)
            elif not synbit and syncbit_inverse:
                para_data.append(_str)
                pass
            else:
                rospy.logerr('buff error!!')
                # para_data.clear()
                # raw_data.clear()
                if not reset:
                    reset=True
            if reset:
                global start
                start = False
                rospy.logwarn('resetting system')
                function.stop_device(self.port)
                self.begin()

    def lidar_publisher(self, event):
        global compress_data
        if len(compress_data) > 0:
            # header
            _Scan = LaserScan()
            _Scan.header.stamp = rospy.Time.now()
            _Scan.header.seq = self.seq
            self.seq += 1
            _Scan.header.frame_id = self.rplidar_frame
            # rplidar_parameters
            _Scan.angle_max = numpy.pi - numpy.radians(0.0)
            _Scan.angle_min = numpy.pi - numpy.radians(360.0)
            _Scan.angle_increment = -numpy.radians(1.0)
            _Scan.time_increment = self.duration / 360
            _Scan.scan_time = self.duration
            _Scan.range_min = self.range_min
            _Scan.range_max = self.range_max
            # rplidar_ranges
            [ranges, intensive] = compress_data.pop()
            _Scan.ranges = ranges
            _Scan.intensities = intensive
            if _Scan != LaserScan():
                print 'pub',len(compress_data)
                pub_data = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
                pub_data.publish(_Scan)

