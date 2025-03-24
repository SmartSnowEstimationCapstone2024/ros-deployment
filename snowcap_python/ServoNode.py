# ros imports
import rospy
from std_msgs.msg import Int32MultiArray, Float64
from Rosmaster_Lib import Rosmaster
# python imports


global rosmaster, override_val, gate_servo, left_baffle, right_baffle

gate_servo = (5, 0, 120)
left_baffle = (3, 20, 180)
right_baffle = (4, 160, 0)
rosmsater = Rosmaster()
override_val = -1

def set_gate(level):
    print(level)

def level_callback(msg: Float64):
    global override_val
    if override_val >= 0:
        set_gate(override_val)
    else:
        set_gate(msg.data)

def coverage_callback(msg: Int32MultiArray):
    print(msg)

def override_callbak(msg: Float64):
    try:
        global override_val, rosmsater
        override_val = msg.data
        if override_val >= 0:
            set_gate(override_val)
            rosmaster.set_uart_servo_angle(left_baffle[0], left_baffle[1])
            rosmaster.set_uart_servo_angle(right_baffle[0], right_baffle[1])
    except:
        rospy.logwarn("Failed receiving the override msg.")