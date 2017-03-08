#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify
"""
from reference import *
import time
import rospy

def rplidar_points(self):
    while self.port.inWaiting() < response_device_point_format.sizeof():
        time.sleep(0.001)
    global reset
    print 'loop'
    reset = False
    _str = self.port.read(response_device_point_format.sizeof())
    global raw_data
    raw_data.append(_str)

def stop_device(self):
    cmd = stop
    command(cmd)
    self.port.setDTR(1)
    self.port.close()

def command(self, com):
    rospy.loginfo('sending commands')
    command = com
    cmd = command_format.build(Container(sync_byte=sync_byte, cmd_flag=command))
    self.port.write(cmd)

def header_check(self):
    rospy.loginfo('evaluating header')
    stamp = time.time()
    time_out = 1
    while time.time() < stamp + time_out:
        if self.port.inWaiting() < response_header_format.sizeof():
            time.sleep(0.01)
        else:
            _str = self.port.read(response_header_format.sizeof())
            response_str = response_header_format.parse(_str)
            # rospy.loginfo('return data stream header checking result:\n')
            # rospy.loginfo('\ninitial response bytes(0XA5 0X5A): %s %s\n' % (hex(response_str.sync_byte1).upper(), hex(response_str.sync_byte2).upper()))
            # rospy.loginfo('response_size: %s'%hex(response_str.response.response_size))
            # rospy.loginfo('response_data: %s'%hex(response_str.response.response_data))
            # rospy.loginfo('response_mode: %s'%self.ResponseMode[response_str.response.response_mode])
            # rospy.loginfo('response_type: %s'%self.ResponseType[response_str.response_type])
            if response_str.sync_byte1 != sync_byte1 or response_str.sync_byte2 != sync_byte2:
                rospy.logerr('unexpect response header')
                return response_str.response_type
                # os.system('rosnode kill cmd_tester')
                # self.defination()
                # self.rplidar_matrix()
            else:
                return response_str.response_type
    rospy.loginfo("time out")

def device_health(self):
    cmd = get_device_health
    self.command(cmd)
    if self.header_check() == devhealth:
        _str = self.port.readline(response_device_health_format.sizeof())
        response_str = response_device_health_format.parse(_str)
        return response_str
    else:
        rospy.logwarn('command for devhealth error or return value error')
        return None

def driver_reset(self):
    cmd = reset
    self.command(cmd)
    self.port.setDTR(1)
    time.sleep(0.01)

def rplidar_matrix(self):
    self.frame = self.frame_default.copy()
    self.ranges = [i for i in self.ranges_default]
    # self.intensive = [i for i in self.intensive_default]

def OutputCoordinate(self, raw):
    response = response_device_point_format.parse(raw)
    inten = response.quality.quality
    angular = (response.angle_q6 >> angle_shift) / 64.0
    angle = round(angular)
    if response.distance_q2 != 0:
        dis = response.distance_q2 / 4.0 / 1000.0
    else:
        dis = float('inf')
    return [angle, dis, inten]


def default_params(self):
    self.frame_default = {}
    self.ranges_default = []
    # self.intensive_default = []
    for i in range(360):
        self.frame_default['%s.0' % i] = []
        self.ranges_default.append(float('inf'))
        # self.intensive_default.append(0.0)