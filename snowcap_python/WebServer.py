# ros imports
import rospy
from std_msgs.msg import Int32MultiArray, Float64
# python imports
from flask import Flask, request, jsonify
import torch

flaskServer = Flask(__name__)
global current_level, tri_coverage, override_flag, override_val, dispense_rate, pubtopic
current_level = 0
tri_coverage = [0,0,0]
override_flag = False
override_val = 0
dispense_rate = 0
pubtopic = None

def level_callback(msg: Float64):
    try:
        global current_level
        current_level = msg.data
    except:
        rospy.logwarn("Failed to receive level msg.")

def coverage_callback(msg: Int32MultiArray):
    try:
        data = torch.tensor(msg.data, dtype=torch.int32).view(-1)
        if len(data) > 1:
            global tri_coverage
            data = torch.split(data, 3)
            data = [torch.sum(data[0]).item(), torch.sum(data[1]).item(), torch.sum(data[2]).item()]
            print(data)
            tri_coverage = [data[0], data[1], data[2]]
    except:
        rospy.logwarn("Failed to receive coverage msg.")

def dispense_callback(msg: Float64):
    try:
        global dispense_rate
        dispense_rate = msg.data
    except:
        rospy.logwarn("Failed to receive dispense rate msg.")

    

@flaskServer.route('/snowlevel', methods=['GET'])
def get_snow_level():
    # Static test values
    global current_level, tri_coverage, override_flag, override_val, dispense_rate
    data = {
        "snow_level": current_level,
        "segment_coverage": tri_coverage,
        "override_flag": override_flag,
        "override_preset": override_val,
        "dispensing_rate": dispense_rate
    }
    rospy.loginfo(f"Sending Static Data: {data}")  # Debugging print
    return jsonify(data)

@flaskServer.route('/snowlevel', methods=['POST'])
def set_override():
    try:
        global override_flag, override_val, pubtopic
        data = request.get_json(force=True)
        override_flag = data["override_flag"]
        override_val = data["override_preset"]
        msg = Float64()
        if override_flag == True:
            msg.data = override_val
        else:
            msg.data = -1
        pubtopic.publish(msg)
    except:
        rospy.logwarn("Failed to parse POST msg.")

if __name__ == '__main__':
    rospy.init_node('WebServerNode')
    pubtopic = rospy.Publisher("/snowcap/override", Float64, queue_size=1)
    rospy.Subscriber('/snowcap/snowsegments', Int32MultiArray, coverage_callback)
    rospy.Subscriber('/snowcap/snowlevel', Float64, level_callback)
    rospy.Subscriber('/snowcap/dispenserate', Float64, dispense_callback)
    flaskServer.run(debug=True, host='0.0.0.0', port=5000)
    rospy.spin()
