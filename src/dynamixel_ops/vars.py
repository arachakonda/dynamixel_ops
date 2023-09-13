# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POS      = 116
ADDR_GOAL_CURR      = 102
ADDR_PRESENT_CURR = 126
ADDR_PRESENT_VEL = 128
ADDR_PRESENT_POS = 132
ADDR_OP_MODE = 11
ADDR_DRIVE_MODE = 10
ADDR_PROF_VEL = 112

LEN_PRESENT_POS = 4
LEN_PRESENT_VEL = 4
LEN_PRESENT_CURR = 2
LEN_GOAL_POS = 4
LEN_GOAL_CURR = 2

LEN_PRESENT_DATA = 10

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

CURRENT_LIMIT = 3000
POSITION_LIMIT = 4096

POSITION_CONTROL_MODE = 3
CURRENT_CONTROL_MODE = 0
VELOCITY_CONTROL_MODE = 1
EXTENDED_POSITION_CONTROL_MODE = 4

REV_FACTOR = 0.229/60 # losely translating to 0.229 rev/min unit of Velocity-based Profile
MAX_REV_VEL = 81/60 #losely translating to 81 rev/min limit of No Load Speed at 5.0 V
MAX_VEL_PROF_VAL = int(MAX_REV_VEL/REV_FACTOR) #maximum velocity profile value for the given no_load speed
#can the velocity profile can be dynamically adapted by taking the velocity data from the motor itself instead of no load speed?

MAX_RETRIES = 3

HOMING_OFFSET_ADDRESS = 20