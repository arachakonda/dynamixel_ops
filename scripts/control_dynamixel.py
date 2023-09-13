import rospy
from dynamixel_sdk import *
from dynamixel_ops.msg import *
from dynamixel_ops.vars import *
from dynamixel_ops.utils import *
from dynamixel_ops.comms import *

DXL_IDS = [1]
def pub_dynamixel_control(position):
    pub = rospy.Publisher('dynamixelCmd', dynamixelCmd, queue_size=10)
    rospy.init_node('dynamixelControlNode', anonymous=True)
    rate = rospy.Rate(200) # 10hz
    while not rospy.is_shutdown():
        init_ids(position, DXL_IDS)
        if(position.pulses[0]<=4080):
            position.pulses[0]+= 10
        else:
            position.pulses[0]= 0
        
        print(position)
        pub.publish(position)
        rate.sleep()

def main():
    try:
        position = dynamixelCmd()
        pub_dynamixel_control(position)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")

if __name__ == '__main__':
    main()
