#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
- Version 5.0 2015/11/12

before run this code pls install parse library and also install construct lib for protocol

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.
"""
import rospy
import rplidar_scan


if __name__ == "__main__":
    try:
        rospy.init_node('rplidar', anonymous=False)
        rospy.loginfo("initialization system")
        try:
            rplidar_scan.driver()
            rplidar_scan.ClearParams()
        except KeyboardInterrupt:
            rplidar_scan.ClearParams()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("unknown_detector node terminated.")
