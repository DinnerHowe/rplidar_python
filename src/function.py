#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify
"""
from reference import *
import time
import rospy
import list_ports_linux

ranges_default = []
for i in range(360):
    ranges_default.append(float('inf'))

def rplidar_points(port, raw_data):
    while port.inWaiting() < response_device_point_format.sizeof():
        time.sleep(0.001)
    _str = port.read(response_device_point_format.sizeof())
    raw_data.append(_str)

def stop_device(port):
    cmd = stop
    send_command(port, cmd)
    port.setDTR(1)
    # port.close()

def send_command(port, com):
    rospy.loginfo('sending commands')
    command = com
    cmd = command_format.build(Container(sync_byte=sync_byte, cmd_flag=command))
    port.write(cmd)

def header_check(port):
    rospy.loginfo('evaluating header')
    stamp = time.time()
    time_out = 1
    while time.time() < stamp + time_out:
        if port.inWaiting() < response_header_format.sizeof():
            time.sleep(0.01)
        else:
            _str = port.read(response_header_format.sizeof())
            response_str = response_header_format.parse(_str)
            if response_str.sync_byte1 != sync_byte1 or response_str.sync_byte2 != sync_byte2:
                rospy.logerr('unexpect response header')
            return response_str.response_type
    rospy.loginfo("time out")
    return None

def device_health(port):
    cmd = get_device_health
    send_command(port, cmd)
    if header_check(port) == devhealth:
        _str = port.readline(response_device_health_format.sizeof())
        response_str = response_device_health_format.parse(_str)
        return response_str
    else:
        rospy.logwarn('command for devhealth error or return value error')
        return None

def driver_reset(port):
    cmd = reset
    send_command(port, cmd)
    port.setDTR(1)
    time.sleep(0.01)

def OutputCoordinate(raw):
    response = response_device_point_format.parse(raw)
    inten = response.quality.quality
    angular = (response.angle_q6 >> angle_shift) / 64.0
    angle = round(angular)
    if response.distance_q2 != 0:
        dis = response.distance_q2 / 4.0 / 1000.0
    else:
        dis = float('inf')
    return [angle, dis, inten]

def range_matrix():
    global ranges_default
    return [i for i in ranges_default]

def range_matrix():
    global ranges_default
    return [i for i in ranges_default]

def intensive_matrix():
    global ranges_default
    return [0 for i in ranges_default]

def port_finder(trigger, port_name):
    ports = list(list_ports_linux.comports())
    for port in ports:
        if port_name in port[1]:
            trigger = True
            rospy.logwarn('find rplidar connect on port: %s' % port[0])
            return [port, trigger]
    return [None, trigger]

def fusion(origion_PolorCoordinate, current_PolorCoordinate):
    return round((origion_PolorCoordinate[0]*origion_PolorCoordinate[1] + current_PolorCoordinate[1]*current_PolorCoordinate[2])/float(current_PolorCoordinate[2] + origion_PolorCoordinate[1]), 4)