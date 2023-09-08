import rospy
from dynamixel_sdk import *
from dynamixel_ops.msg import *
from dynamixel_ops.vars import *
from dynamixel_ops.utils import *
from dynamixel_ops.comms import *

DXL_IDS = [1]
def pub_dynamixel_control_current(current):
    pub = rospy.Publisher('dynamixelCmdTopic', dynamixelCmd, queue_size=10)
    rospy.init_node('dynamixelCurrentControlNode', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        init_ids(current, DXL_IDS)
        current.current[0] = 10
        print(current)
        pub.publish(current)
        rate.sleep()

def main():
    try:
        current = dynamixelCmd()
        pub_dynamixel_control_current(current)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")

if __name__ == '__main__':
    main()
