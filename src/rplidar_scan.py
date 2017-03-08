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
import time
import rospy
import getpass
import collections
import math
import list_ports_linux
from sensor_msgs.msg import LaserScan
import copy

raw_data = collections.deque(maxlen=1)
para_data = collections.deque(maxlen=360)
reset = False
start = False

class ClearParams:
    def __init__(self):
        rospy.logwarn('clearing parameters')
        rospy.delete_param('~scan_topic')
        rospy.delete_param('~rplidar_rate')
        rospy.delete_param('~rplidar_frame')
        rospy.delete_param('~range_min')
        rospy.delete_param('~range_max')

class driver:
    def __init__(self):
        self.defination()
        self.rplidar_matrix()
        self.begin()
        rospy.Timer(rospy.Duration(self.frequency), self.read)
        rospy.Timer(rospy.Duration(self.frequency), self.read)
        rospy.spin()


    def defination(self):
        self.ResponseType = {measurement: 'measurement', devinfo: 'devinfo', devhealth: 'devhealth'}
        self.ResponseStatus = {status_ok: 'status_ok', status_warning: 'status_warning', status_error: 'status_error'}
        self.ResponseMode = {SINGLE: 'SINGLE', MULTI: 'MULTI', UNDEFINED_f: 'UNDEFINED', UNDEFINED_s: 'UNDEFINED'}

        self.default_params()
        self.seq = 0
        self.accout = getpass.getuser()

        if not rospy.has_param('~scan_topic'):
            rospy.set_param('~scan_topic', '/scan')
        self.scan_topic = rospy.get_param('~scan_topic')
        if not rospy.has_param('~rplidar_rate'):
            rospy.set_param('~rplidar_rate', 0.001)
        self.frequency = rospy.get_param('~rplidar_rate')
        if not rospy.has_param('~rplidar_port_name'):
            rospy.set_param('~rplidar_port_name', 'CP2102 USB to UART Bridge Controller')
        self.rplidar_port_name = rospy.get_param('~rplidar_port_name')
        if not rospy.has_param('~rplidar_frame'):
            rospy.set_param('~rplidar_frame', '/camera_depth_framer')
        self.rplidar_frame = rospy.get_param('~rplidar_frame')
        if not rospy.has_param('~range_min'):
            rospy.set_param('~range_min', 0.15)
        self.range_min = rospy.get_param('~range_min')
        if not rospy.has_param('~range_max'):
            rospy.set_param('~range_max', 6.0)
        self.range_max = rospy.get_param('~range_max')


    def port_finder(self, trigger):
        ports = list(list_ports_linux.comports())
        for port in ports:
            if self.rplidar_port_name in port[1]:
                trigger = True
                rospy.logwarn('find rplidar connect on port: %s' % port[0])
                return [port, trigger]
            else:
                # port = []
                trigger = False
        return [port, trigger]

    def read(self, event):
        global start
        if start:
            self.rplidar_points()


    def resolve(self):
       global raw_data
        _str = raw_data.pop()
        response = response_device_point_format.parse(_str)
        synbit = response.quality.syncbit
        syncbit_inverse = response.quality.syncbit_inverse
        if synbit and not_start:
            not_start = False
        if not not_start:
            global para_data
            # release data
            if synbit and not syncbit_inverse:
                data_buff = list(para_data)
                para_data.clear()
                for i in range(len(data_buff)):
                    PolorCoordinate = self.OutputCoordinate(data_buff[i])
                    angle = PolorCoordinate[0]
                    if str(angle) in self.frame:
                        if not math.isinf(PolorCoordinate[1]):
                            # self.intensive[int(angle)] = PolorCoordinate[2]
                            self.frame[str(angle)].append(PolorCoordinate[1])
                            self.ranges[int(angle)] = round(numpy.mean(self.frame[str(angle)]), 4)
                            # self.frame[str(angle)].append(copy.deepcopy(PolorCoordinate[1]))
                            # self.ranges[int(angle)] = round(numpy.mean(self.frame[str(angle)]), 4)
                self.lidar_publisher(self.ranges)
                self.rplidar_matrix()
            elif not synbit and syncbit_inverse:
                self.rplidar_matrix()
                pass
            else:
                rospy.logerr('buff error!!')
                para_data.clear()
                if not reset:
                    reset=True
                    break
            para_data.append(_str)
        if reset:
            rospy.logwarn('resetting system')
            self.stop_device()
            self.rplidar_matrix()
            self.begin()

    def begin(self, trigger=False):
        finder = self.port_finder(trigger)
        if finder[1]:
            self.port = serial.Serial("%s" % finder[0][0], 115200)
            self.port.setDTR(1)
            rospy.logwarn("connect port: %s" % finder[0][1])
            self.port.flushInput()  # discarding all flush input buffer contents
            self.port.flushOutput()
            health = self.device_health()
            try:
                rospy.loginfo('health status: %s'%self.ResponseStatus[health.status])
            except:
                rospy.logwarn('health status: %s' %health)
                self.begin()
            if health != None:
                if health.status != status_ok:
                    self.driver_reset()
                else:
                    self.port.setDTR(0)
                    self.current = rospy.Time.now()
                    # self.rplidar_points(not_start)
                    cmd = scan
                    self.command(cmd)
                    if self.header_check() == measurement:
                        global start
                        start = True
                    else:
                        rospy.logerr('header check error,response type not measurement')
                        self.rplidar_matrix()
                        start = False
                        self.begin()

        else:
            rospy.loginfo('cannot find rplidar please connect rplidar on')
            self.stop_device()

    def lidar_publisher(self, ranges, intensive = []):
        duration = (rospy.Time.now().secs - self.current.secs) + (rospy.Time.now().nsecs - self.current.nsecs) * (10 ** (-9))
        self.current = rospy.Time.now()
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
        _Scan.time_increment = duration / 360
        _Scan.scan_time = duration
        _Scan.range_min = self.range_min
        _Scan.range_max = self.range_max
        # rplidar_ranges
        _Scan.ranges = ranges
        _Scan.intensities = intensive
        if _Scan != LaserScan():
            print 'pub'
            pub_data = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
            pub_data.publish(_Scan)
