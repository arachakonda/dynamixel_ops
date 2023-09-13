import rospy
from dynamixel_sdk import *
from dynamixel_ops.msg import *
from dynamixel_ops.vars import *
from dynamixel_ops.utils import *
from dynamixel_ops.comms import *
import time

DXL_IDS = [1]
def pub_dynamixel_control(position):
    pub = rospy.Publisher('/dynamixel_ops/dynamixelCmd', dynamixelCmd, queue_size=1)
    rospy.init_node('dynamixelControlNode', anonymous=True)
    rate = rospy.Rate(100) # 20hz
    while not rospy.is_shutdown():
        if(position.angle[0] < 2*3.14):
            position.angle[0]+= 0.0025
        else:
            position.angle[0]= 0
            time.sleep(2)
        
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
