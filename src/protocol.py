#!/usr/bin/env python
""" 
- Version 1.0 2015/22/10   

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
from construct import *


# protocol
#cmd
sync_byte=0xA5
flag_has_pay_load=0x80

#response
sync_byte1=0xA5
sync_byte2=0x5A


# struct
command_format = Struct("cmd_format",
    ULInt8("sync_byte"), # must be RPLIDAR_CMD_SYNC_BYTE: A5
    ULInt8("cmd_flag") # one byte for CMD
)

response_header_format = Struct("header_format",
    ULInt8("sync_byte1"), # must be RPLIDAR_ANS_SYNC_BYTE1: A5
    ULInt8("sync_byte2"), # must be RPLIDAR_ANS_SYNC_BYTE2: 5A
    #ULInt32("size_q30_subtype"), # see _u32 size:30; _u32 subType:2;
    ULInt24("response_size"),
    BitStruct("response_mode", 
            Enum(BitField("mode", 2), SINGLE = 0x0, MULTI = 0x1, _default_ = "UNDEFINED"),
            Padding(6)),
    ULInt8("response_type") # one byte for message type
)
