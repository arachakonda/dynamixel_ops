#!usr/bin/env python3
import rospy
import os
from dynamixel_ops.vars import *
from dynamixel_ops.comms import *


def check_limit(val, limit):
    if(val>limit):
        return 0
    if(val<=limit):
        return 1
    
def open_port(portHandler):
    # Open port
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        quit()

def close_port(portHandler,groupBulkRead):
    # Open port
    try:
       clearBR(groupBulkRead)
       portHandler.closePort()
       print("Succeeded to close the port")
    except:
        print("Failed to close the port")
        quit()

def close_port_SR(portHandler,groupSyncRead):
    # Open port
    try:
       clearSR(groupSyncRead)
       portHandler.closePort()
       print("Succeeded to close the port")
    except:
        print("Failed to open the port")
        quit()

def set_baudrate(portHandler, baudrate):
    # Set port baudrate
    try:
        portHandler.setBaudRate(baudrate)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        quit()

def calcVelProf(ros_freq, pos_inc):
    rev = pos_inc/POSITION_LIMIT
    rev_vel = rev*ros_freq
    vel_prof_max = rev_vel/REV_FACTOR
    return int(vel_prof_max)

