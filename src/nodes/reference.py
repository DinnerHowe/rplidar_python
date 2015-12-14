#!/usr/bin/env python
# -*- coding:utf-8 -*-
""" 
- Version 1.0 2015/22/10   

this file base on rplidar protocol interface

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

#标识位
sync_byte=0xA5#报头字节
flag_has_pay_load=0x80#结尾字节

#response
sync_byte1=0xA5#返回报头字节
sync_byte2=0x5A#返回报头字节

#返回值类型的字节
measurement = 0x81#当请求为0x02/0x21时的返回值的尾字节
devinfo = 0x4#当请求为0x50时的返回值的尾字节
devhealth = 0x6#当请求为0x52时的返回值的尾字节

#设备健康状态的返回值
status_ok = 0x0
status_warning = 0x1
status_error = 0x2

#扫描模型
SINGLE = 0x0
MULTI = 0x1
UNDEFINED_f=0x2
UNDEFINED_s=0x3

# struct
# 命令格式
command_format = Struct("cmd_format",
 ULInt8("sync_byte"), # must be A5
 ULInt8("cmd_flag") # CMD
)

#返回报头文件格式
response_header_format = Struct("header_format",
 ULInt8("sync_byte1"), # must be A5
 ULInt8("sync_byte2"), # must be 5A
 BitStruct("response",
  BitField("response_size", 8),
  BitField("response_data", 22),
  BitField("response_mode",2),
  ),
 ULInt8("response_type"), # # to determine which kind of response it is
)

# 返回硬件参数格式 (20 bytes)
response_device_info_format = Struct("info_format",
 ULInt8("model"),
 ULInt16("firmware_version"),
 ULInt8("hardware_version"),
 String("serial_number", 16)
)

# 返回硬件状况格式 (3 bytes)
response_device_health_format = Struct("health_format",
 ULInt8("status"),
 ULInt16("error_code")
)

# 返回单次扫描格式 (5 bytes)
"""angular = angle_q6/64.0 Deg
   distance = distance_q2/4.0 mm"""

response_device_point_format = Struct("point_format",

 BitStruct("quality", 
  BitField("quality", 6),
  Flag("syncbit_inverse"),#扫描起始标志位的取反,始终有S̅ = ! S
  Flag("syncbit")),#扫描起始标志位,S=1 表示新的一圈 360 度扫描的开始
#测距点相对于 RPLIDAR 朝向夹角(角度表示,[0-360)。使用定点小数表示
 BitStruct("angle_q6_lower",
  BitField("angle_q6_lower", 7),
  Const(Flag("check_bit_c"), 1)), # check_bit must be 1
 ULInt8("angle_q6_higher"),

 ULInt16("distance_q2")
)




# 2进制字节转16进制字符
toHex = lambda x:"".join([hex(ord(c))[2:].zfill(2) for c in x]).upper()
