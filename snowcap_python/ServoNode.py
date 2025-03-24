# ros imports
import rospy
from std_msgs.msg import Int32MultiArray, Float64
from Rosmaster_Lib import Rosmaster
# python imports
import torch

global rosmaster, override_val, gate_servo, left_baffle, right_baffle, pubtopic

gate_servo = (5, 0, 120)
left_baffle = (3, 20, 180)
right_baffle = (4, 160, 0)
rosmaster = Rosmaster()
override_val = -1
pubtopic = None

def set_gate(level):
    try:
        global pubtopic, rosmaster, gate_servo
        if level < 0:
            level = 0
        pubtopic.publish(Float64(level))
        rosmaster.set_uart_servo_angle(gate_servo[0], (level * (gate_servo[2] - gate_servo[1])))
    except:
        rospy.logwarn("Failed to set dispense rate/gate.")

def level_callback(msg: Float64):
    global override_val
    if override_val >= 0:
        set_gate(override_val / 4.0)
    else:
        set_gate(msg.data / 1.78)

def coverage_callback(msg: Int32MultiArray):
    try:
        if override_val < 0:
            global rosmaster, left_baffle, right_baffle
            data = torch.tensor(msg.data, dtype=torch.int32).view(-1)
            left, right = torch.split(data, 2)
            left = left.tolist()
            right = right.tolist()
            right.reverse()
            datamax = torch.max(data).item()
            for i in range(0, len(left)):
                if left[i] > datamax * 0.25:
                    rosmaster.set_uart_servo_angle(left_baffle[0], ((i / float(len(left))) * (left_baffle[2] - left_baffle[1])))
                    break
            for i in range(0, len(right)):
                if right[i] > datamax * 0.25:
                    rosmaster.set_uart_servo_angle(right_baffle[0], ((i / float(len(right)) / (right_baffle[2] - right_baffle[1]))))
                    break
    except:
        rospy.logwarn("Failed to set baffles from coverage msg.")

def override_callback(msg: Float64):
    try:
        global override_val, rosmaster
        override_val = msg.data
        if override_val >= 0:
            set_gate(override_val)
            rosmaster.set_uart_servo_angle(left_baffle[0], left_baffle[1])
            rosmaster.set_uart_servo_angle(right_baffle[0], right_baffle[1])
    except:
        rospy.logwarn("Failed receiving the override msg.")

if __name__ == '__main__':
    rospy.init_node('ServoNode')
    pubtopic = rospy.Publisher("/snowcap/dispenserate", Float64, queue_size=1)
    rospy.Subscriber('/snowcap/snowsegments', Int32MultiArray, coverage_callback)
    rospy.Subscriber('/snowcap/snowlevel', Float64, level_callback)
    rospy.Subscriber('/snowcap/override', Float64, override_callback)
    rospy.spin()