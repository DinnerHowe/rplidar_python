#!/usr/bin/env python
""" permission_get.py 

- Version 2.0 2015/8/28

this file is used to aquire rfid sensor publisher communicate port permission 

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import os
import getpass
if __name__=='__main__':
 name=getpass.getuser()
 permission=os.popen('sudo usermod -a -G dialout %s'%name)
 print "please restart your computer"
