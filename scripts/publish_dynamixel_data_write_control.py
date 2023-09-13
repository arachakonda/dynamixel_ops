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

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
#Initialize GroupSyncRead instace for Current, Position and Velocity
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURR, LEN_PRESENT_DATA)
#Initialize GroupSyncWrite instance for Position only
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POS, LEN_GOAL_POS)
#the order of the list is [mcpf, mcpa, pip, dip]
DXL_IDS = [1]
time.sleep(0.5)

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

def convert_to_range(pulses, velocity, current):
    #convert the pulses to radians
    for i in range(len(pulses)):
        pulses[i] = pulses[i] * 0.001533981


    #if velocity value is greater than 2047, then it is a negative number
    for i in range(len(velocity)):
        if velocity[i] > 2047:
            #convert the number from 16-bit unsigned to 16-bit signed
            velocity[i] = (velocity[i] ^ 0xFFFF) + 1
        else:
            velocity[i] = velocity[i]
        #convert the number to rad/sec
        velocity[i] = velocity[i] * 0.229 * 0.104719755

    #if the current value is greater than 2352, then it is a negative number
    for i in range(len(current)):
        if current[i] > 2352:
            #convert the number from 16-bit unsigned to 16-bit signed
            current[i] = (current[i] ^ 0xFFFF) + 1
        else:
            current[i] = current[i]
        #convert the number to mA
        current[i] = current[i] * 2.69


    
def pub_dynamixelTelemetry(groupSyncRead, DXL_IDS, dynTel):
    pub_dynamixelTelemetry = rospy.Publisher('/dynamixel_ops/dynamixelTelemetry', dynamixelTelemetry, queue_size=10)
    rospy.init_node('pubw_pos_vel_curr', anonymous=True)
    ros_freq = 20
    rate = rospy.Rate(ros_freq) # 10hz
    while not rospy.is_shutdown():
        #read data from dynamixels
        syncRead_dynamixelTelemetry(groupSyncRead, DXL_IDS, dynTel)
        print_data(dynTel.pulses, dynTel.velocity, dynTel.current)
        convert_to_range(dynTel.pulses, dynTel.velocity, dynTel.current)
        #publish data
        pub_dynamixelTelemetry.publish(dynTel)
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
    dxl_error = 0
    dxl_comm_result = COMM_TX_FAIL
    dxl_addparam_result = False
    command_list = list(data.pulses)
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
    setOpModes(portHandler, packetHandler, DXL_IDS, POSITION_CONTROL_MODE)
    #add IDs for sync read of same data from multiple dynamixels
    add_SyncReadIDs(groupSyncRead, DXL_IDS)
    #sync read data from multiple dynamixels
    #enable dynamixel torques
    enable_torques(portHandler, packetHandler, DXL_IDS)
    try:
        dynTel = dynamixelTelemetry() 
        rospy.Subscriber('/dynamixel_ops/dynamixelCmd', dynamixelCmd, write_dynamixelCmdCallback)
        pub_dynamixelTelemetry(groupSyncRead, DXL_IDS, dynTel)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")
    
    #disable dynamixel torques
    disable_torques(portHandler, packetHandler, DXL_IDS)
    #close port
    close_port_SR(portHandler,groupSyncRead)


if __name__ == '__main__':
    main()