#!/usr/bin/env python
# -*- coding:utf-8 -*-
""" 
- Version 1.0 2015/22/10   

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
from construct import *


# cmds
#无应答
stop=0x25#离开扫描采样模式,进入空闲状态
reset=0x40#测距核心软重启
#多次应答
force_scan=0x21#请求进入扫描采样状态,强制数据输出
scan=0x20#请求进入扫描采样状态
#单次应答
get_device_info=0x50#获取设备序列号等信息
get_device_health=0x52#获取设备健康状态

#response
#返回值的尾字节
measurement = 0x81#当请求为0x02/0x21时的返回值的尾字节
devinfo = 0x4#当请求为0x50时的返回值的尾字节
devhealth = 0x6#当请求为0x52时的返回值的尾字节
#设备健康状态的返回值
status_ok = 0x0
status_warning = 0x1
status_error = 0x2


# struct
#各种返回值的格式

# serial data structure returned by GET_INFO (20 bytes)
response_device_info_format = Struct("info_format",
    ULInt8("model"),
    ULInt8("firmware_version_minor"),
    ULInt8("firmware_version_major"),
    ULInt8("hardware_version"),
    String("serial_number", 16)
)

# serial data structure returned by GET_HEALTH (3 bytes)
response_device_health_format = Struct("health_format",
    Enum(Byte("status"), 
            status_ok = status_ok,
            status_warning = status_warning,
            status_error = status_error),
    ULInt16("error_code")
)

# serial data structure returned by SCAN -- a single point (5 bytes)
response_device_point_format = Struct("point_format",
    BitStruct("byte0", 
                BitField("sync_quality", 6),
                Flag("syncbit_inverse"),
                Flag("syncbit")),
    BitStruct("byte1",
                BitField("angle_lowbyte", 7),
                Const(Flag("check_bit"), 1)), # check_bit must be 1
    ULInt8("angle_highbyte"),
    ULInt16("distance_q2")
)


# convert binary to hex string
toHex = lambda x:"".join([hex(ord(c))[2:].zfill(2) for c in x]).upper()
