#!/usr/bin/env python3

#*******************************************************************************
# Copyright 2023 @ arachakonda.github.io
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************

#*******************************************************************************
# This example is written for DYNAMIXEL XC-330-M288-T series with U2D2.
# This example specifically demonstrates the position control of DYNAMIXEL while
# using velocity profile in position control mode(single rev) by using step-
# increments
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun dynamixel_ops readWrite_pos_current.py
#
# Open terminal #3 (run one of below commands at a time)
#
#
# Author: Ananth Rachakonda
#******************************************************************************/

#*******************************************************************************
# The flow of the code is as follows:
# 1) first open port using the portHandler object to communcate with U2D2
# 2) set the baudrate of that port to communcate with DYNAMIXELs
# 3) set operation mode of the DYNAMIXELs to POSITION_CONTROL_MODE
# 4) enable torque on the DYNAMIXELs
# 5) add bulkRead Parameters; note that you can add the starting address and
#    and read multiple parameters of the DYNAMIXEL based on length specified
# 6) create a message object posCurr
# 7) home the DYNAMIXELs
# 8) start the publisher for position and current
#******************************************************************************/
import os
import rospy
from dynamixel_sdk import *
from dynamixel_ops.msg import *
from dynamixel_ops.vars import *
from dynamixel_ops.utils import *
from dynamixel_ops.comms import *
import time
import threading  # Step 1: Import threading

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURR, LEN_PRESENT_DATA)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POS, LEN_GOAL_POS)
DXL_IDS = [1]
time.sleep(0.5)

dynamixel_lock = threading.Lock()  # Step 2: Create a global lock

def syncRead_dynamixelTelemetry(groupSyncRead, DXL_IDS, dT):
    #initialize IDs
    init_ids(dT, DXL_IDS)
    #perform the syncRead
    syncRead(groupSyncRead, packetHandler)
    #check if data is available
    checkDataAvailable(groupSyncRead, DXL_IDS, ADDR_PRESENT_CURR, LEN_PRESENT_DATA)
    #extract data from the syncRead
    extractSyncReadData(groupSyncRead, DXL_IDS, dT)

def print_data(pos, vel, curr):
    print("Telemetry Data: ")
    print(pos)
    print(vel)
    print(curr)
    print("------------------")
    pass

#for publishing Telemetry data in standard units
def convert_to_range(pulses, velocity, current):
    # Convert the inputs to lists for modification
    pulses_list = list(pulses)
    velocity_list = list(velocity)
    current_list = list(current)

    # Convert the pulses to radians
    for i in range(len(pulses_list)):
        if pulses_list[i] > 1048575:
            # Convert the number from 32-bit unsigned to 32-bit signed
            pulses_list[i] = -(4294967295 - pulses_list[i] + 1)
            
        pulses_list[i] = pulses_list[i] * 0.001533981

    # If velocity value is greater than 2047, then it is a negative number
    for i in range(len(velocity_list)):
        if velocity_list[i] > 2047:
            # Convert the number from 16-bit unsigned to 16-bit signed
            velocity_list[i] = -(4294967295 - velocity_list[i] + 1)
        # Convert the number to rad/sec
        velocity_list[i] = velocity_list[i] * 0.229 * 0.104719755

    # If the current value is greater than 2352, then it is a negative number
    for i in range(len(current_list)):
        if current_list[i] > 2352:
            # Convert the number from 16-bit unsigned to 16-bit signed
            current_list[i] = -(65535 - current_list[i] + 1)
        # Convert the number to mA
        current_list[i] = current_list[i] * 1

    return pulses_list, velocity_list, current_list

#for writing commands to dynamixel in dynamixel units
def convert_from_range(radians, velocity, current):
    # Convert the inputs to lists for modification
    pulses_list = list(radians)
    velocity_list = list(velocity)
    current_list = list(current)

    # Convert radian to pulses
    for i in range(len(pulses_list)):
        pulses_list[i] = int(pulses_list[i] * 651.8986469044037)

    # Convert velocity from rad/sec to 16-bit signed
    for i in range(len(velocity_list)):
        velocity_list[i] = int(velocity_list[i] * 4.363323129985824)
        if velocity_list[i] < 0:
            # Convert the number from 16-bit signed to 16-bit unsigned
            velocity_list[i] = velocity_list[i] + 4294967295

    # Convert current from mA to 16-bit signed
    for i in range(len(current_list)):
        current_list[i] = int(current_list[i] * 1)
        if current_list[i] < 0:
            # Convert the number from 16-bit signed to 16-bit unsigned
            current_list[i] = current_list[i] + 65536

    return pulses_list, velocity_list, current_list

class dynamixelTelemetryClass:
    #defined to get telemetry data from dynamixels
    def __init__(self, DXL_IDS=DXL_IDS):
        self.id = DXL_IDS
        self.angle = [0]*len(DXL_IDS)
        self.velocity = [0]*len(DXL_IDS)
        self.current = [0]*len(DXL_IDS)

    
def pub_dynamixelTelemetry(packetHandler, groupSyncRead, DXL_IDS, dynTel):
    pub_dynamixelTelemetry = rospy.Publisher('/dynamixel_ops/dynamixelTelemetry', dynamixelTelemetry, queue_size=1)
    pubObj = dynamixelTelemetry()
    rospy.init_node('pubw_pos_vel_curr', anonymous=True)
    ros_freq = 200
    rate = rospy.Rate(ros_freq) # 10hz
    while not rospy.is_shutdown():
        #read data from dynamixels

        with dynamixel_lock:  
            syncRead_dynamixelTelemetry(groupSyncRead, DXL_IDS, dynTel)
            print_data(dynTel.angle, dynTel.velocity, dynTel.current)
        pubObj.angle, pubObj.velocity, pubObj.current = convert_to_range(dynTel.angle, dynTel.velocity, dynTel.current)
        print_data(pubObj.angle, pubObj.velocity, pubObj.current)
        #publish data
        pub_dynamixelTelemetry.publish(pubObj)
        #sleep for enough time to match the frequency of publishing
        rate.sleep()

def pingDynamixels(packetHandler, DXL_IDS):
    pingable_ids = []
    for id in DXL_IDS:
        dxl_comm_result = COMM_TX_FAIL
        print("pinging ID:%03d" % id)
        model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, id)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ID:%03d] ping Failed" % id)
        elif dxl_comm_result == COMM_SUCCESS:
            pingable_ids.append(id)
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (id, model_number))
        
    time.sleep(3)
    return pingable_ids

def write_dynamixelCmdCallback(data):
    with dynamixel_lock:  # Use the lock
        dxl_error = 0
        dxl_comm_result = COMM_TX_FAIL
        dxl_addparam_result = False
        angle_list, velocity_list, current_list = convert_from_range(data.angle, data.velocity, data.current)
        command_list = angle_list
        for i in range(len(DXL_IDS)):
            #param_goal = [DXL_LOBYTE(command_list[i]), DXL_HIBYTE(command_list[i])]
            param_goal = [DXL_LOBYTE(DXL_LOWORD(command_list[i])), 
                        DXL_HIBYTE(DXL_LOWORD(command_list[i])), 
                        DXL_LOBYTE(DXL_HIWORD(command_list[i])),
                        DXL_HIBYTE(DXL_HIWORD(command_list[i]))]
            dxl_addparam_result = groupSyncWrite.addParam(DXL_IDS[i], param_goal)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % i)
            else:
                print("[ID:%03d] groupSyncWrite addparam success" % i)
        
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("failed to set command")
        else:
            print("succeeded to set command")
        groupSyncWrite.clearParam()

def main():
    global portHandler, packetHandler, groupSyncRead, DXL_IDS
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    #check if the dynamixels with ids in DXL_IDS are connected
    DXL_IDS = pingDynamixels(packetHandler, DXL_IDS)
    #set dynamixels to current control mode
    setOpModes(portHandler, packetHandler, DXL_IDS, EXTENDED_POSITION_CONTROL_MODE)
    #set homing offset
    setHomingOffset(portHandler, packetHandler, DXL_IDS)
    #add IDs for sync read of same data from multiple dynamixels
    add_SyncReadIDs(groupSyncRead, DXL_IDS)
    #sync read data from multiple dynamixels
    #enable dynamixel torques
    enable_torques(portHandler, packetHandler, DXL_IDS)
    try:
        dynTel = dynamixelTelemetryClass(DXL_IDS=DXL_IDS) 
        rospy.Subscriber('/dynamixel_ops/dynamixelCmd', dynamixelCmd, write_dynamixelCmdCallback)
        pub_dynamixelTelemetry(packetHandler, groupSyncRead, DXL_IDS, dynTel)
    except:
        print("ROS Node Terminated")
    
    #disable dynamixel torques
    disable_torques(portHandler, packetHandler, DXL_IDS)
    #close port
    close_port_SR(portHandler,groupSyncRead)


if __name__ == '__main__':
    main()