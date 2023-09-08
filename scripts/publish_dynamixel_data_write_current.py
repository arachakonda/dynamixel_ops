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
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURR, LEN_GOAL_CURR)
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
    print(pos)
    print(vel)
    print(curr)
    print("------------------")
    pass

    
def pub_dynamixelTelemetry(packetHandler, groupSyncRead, DXL_IDS, dynTel):
    pub_dynamixelTelemetry = rospy.Publisher('dynamixelTelemetry', dynamixelTelemetry, queue_size=10)
    rospy.init_node('pubw_pos_vel_curr', anonymous=True)
    ros_freq = 10
    rate = rospy.Rate(ros_freq) # 10hz
    while not rospy.is_shutdown():
        #read data from dynamixels
        syncRead_dynamixelTelemetry(groupSyncRead, DXL_IDS, dynTel)
        print_data(dynTel.angle, dynTel.velocity, dynTel.current)
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
    current_list = list(data.current)
    for i in range(len(DXL_IDS)):
        param_goal_current = [DXL_LOBYTE(current_list[i]), DXL_HIBYTE(current_list[i])]
        print(param_goal_current)
        dxl_addparam_result = groupSyncWrite.addParam(DXL_IDS[i], param_goal_current)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % i)
        else:
            print("[ID:%03d] groupSyncWrite addparam success" % i)
    
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("failed to set current")
    else:
        print("succeeded to set current")
    groupSyncWrite.clearParam()

    print(data)

def main():
    global portHandler, packetHandler, groupSyncRead, DXL_IDS
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    #check if the dynamixels with ids in DXL_IDS are connected
    DXL_IDS = pingDynamixels(packetHandler, DXL_IDS)
    #set dynamixels to current control mode
    setOpModes(portHandler, packetHandler, DXL_IDS, CURRENT_CONTROL_MODE)
    #add IDs for sync read of same data from multiple dynamixels
    add_SyncReadIDs(groupSyncRead, DXL_IDS)
    #sync read data from multiple dynamixels
    #enable dynamixel torques
    enable_torques(portHandler, packetHandler, DXL_IDS)
    try:
        dynTel = dynamixelTelemetry() 
        rospy.Subscriber('dynamixelCmdTopic', dynamixelCmd, write_dynamixelCmdCallback)
        pub_dynamixelTelemetry(packetHandler, groupSyncRead, DXL_IDS, dynTel)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")
    
    #disable dynamixel torques
    disable_torques(portHandler, packetHandler, DXL_IDS)
    #close port
    close_port_SR(portHandler,groupSyncRead)


if __name__ == '__main__':
    main()