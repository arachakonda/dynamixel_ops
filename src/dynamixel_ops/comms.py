#!/usr/bin/env python3
from dynamixel_sdk import *
from dynamixel_ops.msg import *
from dynamixel_ops.vars import *

def get_present_current(id, portHandler, packetHandler):
    dxl_present_curr, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_CURR)
    return dxl_present_curr

def get_present_velocity(id, portHandler, packetHandler):
    dxl_present_vel, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_VEL)
    return dxl_present_vel

def get_present_pos(id, portHandler, packetHandler):
    dxl_present_pos, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POS)
    return dxl_present_pos

def enable_torque(portHandler, packetHandler, id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d Torque is enabled" % id)

def enable_torques(portHandler, packetHandler, DXL_IDS):
    for id in DXL_IDS:
        enable_torque(portHandler, packetHandler, id)

def add_paramBR(groupBulkRead, id, param_addr, param_len):
    # Add parameter storage for Dynamixel#1 present position
    dxl_addparam_result = groupBulkRead.addParam(id, param_addr, param_len)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % id)
        quit()

def bulkRead(groupBulkRead, packetHandler):
    dxl_comm_result = groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

def syncRead(groupSyncRead, packetHandler):
    # Sync read the current positions and velocities of the motors
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        quit()

def checkDataAvailable(groupSyncRead, DXL_IDS, startAddr, dataLen):
    for id in DXL_IDS:
        dxl_getdata_result = groupSyncRead.isAvailable(id, startAddr, dataLen)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % (id))
            quit()


def extractData(groupBulkRead, id, param_addr, param_len):
    # Check if groupbulkread data of Dynamixel#1 is available
    dxl_getdata_result = groupBulkRead.isAvailable(id, param_addr, param_len)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkRead getdata failed" % id)
        quit()
    dxl_data = groupBulkRead.getData(id, param_addr, param_len)

    if param_addr == ADDR_PRESENT_VEL:
        if dxl_data > 2047:
            #take 2's complement of 32 bit number
            dxl_data = dxl_data - 4294967296
    if param_addr == ADDR_PRESENT_CURR:
        if dxl_data > 2047:
            #take 2's complement of 32 bit number
            dxl_data = dxl_data - 4294967296
    return dxl_data

def clearBR(groupBulkRead):
    groupBulkRead.clearParam()

def clearBW(groupBulkWrite):
    groupBulkWrite.clearParam()

def add_paramBW(groupBulkWrite, id, param_addr, param_len, param_val):
    # Allocate goal position value into byte
    if param_len == 4:
        param_val_array = [DXL_LOBYTE(DXL_LOWORD(param_val)), 
                            DXL_HIBYTE(DXL_LOWORD(param_val)), 
                            DXL_LOBYTE(DXL_HIWORD(param_val)), 
                            DXL_HIBYTE(DXL_HIWORD(param_val))]
    elif param_len == 2:
        param_val_array = [DXL_LOBYTE(param_val), 
                            DXL_HIBYTE(param_val)]
    
    elif param_len == 1:
        param_val_array=[param_val]

    # Add Dynamixel#1 parameter value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(id, param_addr, param_len, param_val_array)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % id)
        quit()

def bulkWrite(groupBulkWrite, packetHandler):
    # Bulkwrite goal position and LED value
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    
def disable_torque(portHandler, packetHandler, id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,  id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d Torque is disabled" % id)

def disable_torques(portHandler, packetHandler, DXL_IDS):
    for id in DXL_IDS:
        disable_torque(portHandler, packetHandler, id)

def setOpMode(portHandler, packetHandler, id, mode):
    disable_torque(portHandler, packetHandler, id)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OP_MODE, mode)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully shifted to mode %d" % (id,mode))

    enable_torque(portHandler, packetHandler, id)

def setOpModes(portHandler, packetHandler, DXL_IDS, mode):
    for id in DXL_IDS:
        setOpMode(portHandler, packetHandler, id, mode)


def setProfVel(portHandler, packetHandler, id, val):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_PROF_VEL, val)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
        #print("Dynamixel#%d has been successfully fixed to velocity profile maximum %d" % (id,val))

def add_SyncReadIDs(groupSyncRead, DXL_IDS):
    for id in DXL_IDS:
        try:
            groupSyncRead.addParam(id)
            print("SyncRead ID %d added" % id)
        except:
            print("Motor [ID:%03d] groupSyncRead addition failed" % id)

def add_SyncWriteIDParams(groupSyncWrite, DXL_IDS, param_addr, param_len):
    for id in DXL_IDS:
        try:
            groupSyncWrite.addParam(id, param_len, param_addr)
            print("SyncWrite ID %d added" % id)
        except:
            print("Motor [ID:%03d] groupSyncWrite addition failed" % id)

def clearSR(groupSyncRead):
    groupSyncRead.clearParam()

def init_ids(dT, DXL_IDS):
    # print(DXL_IDS)
    dT.id = DXL_IDS
    

def extractSyncReadData(groupSyncRead, DXL_IDS, dT):
    for i in range(0, len(DXL_IDS)):
        try:
            dT.angle[i] = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_POS, LEN_PRESENT_POS)
            dT.velocity[i] = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_VEL, LEN_PRESENT_VEL)
            dT.current[i] = groupSyncRead.getData(DXL_IDS[i], ADDR_PRESENT_CURR, LEN_PRESENT_CURR)
        except:
            print("[ID:%03d] groupSyncRead getdata failed" % (i))


def setHomingOffset(portHandler,packetHandler, DXL_IDS):
    #read present position
    for id in DXL_IDS:
        disable_torque(portHandler, packetHandler, id)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, HOMING_OFFSET_ADDRESS, 0)
        dxl_present_pos = get_present_pos(id, portHandler, packetHandler)
        #print("present position of motor %d is %d" % (id, dxl_present_pos))
        if dxl_present_pos > 0:
            homing_offset = (-dxl_present_pos)
        elif dxl_present_pos < 0:
            homing_offset = dxl_present_pos
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, HOMING_OFFSET_ADDRESS, homing_offset)
        if dxl_comm_result != COMM_SUCCESS:
            #print failed to set homing offset
            print("failed to set homing offset with result %s" % packetHandler.getTxRxResult(dxl_comm_result))
        else:
            #print succeeded to set homing offset
            print("succeeded to set homing offset")
        enable_torque(portHandler, packetHandler, id)
        #print("present position of motor %d is %d" % (id, get_present_pos(id, portHandler, packetHandler)))
        
