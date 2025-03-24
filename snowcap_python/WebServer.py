# ros imports
import rospy
from std_msgs.msg import Int32MultiArray, Float64
# python imports
from flask import Flask, jsonify
import torch

flaskServer = Flask(__name__)
global current_level
global tri_coverage
current_level = 0
tri_coverage = [0,0,0]

def level_callback(msg: Float64):
    try:
        global current_level
        current_level = msg.data
    except:
        rospy.logwarn("Failed to receive level msg.")

def coverage_callback(msg: Int32MultiArray):
    #try:
        data = torch.tensor(msg.data, dtype=torch.int32).view(-1)
        if len(data) > 1:
            global tri_coverage
            data = torch.split(data, 3)
            data = [torch.sum(data[0]).item(), torch.sum(data[1]).item(), torch.sum(data[2]).item()]
            print(data)
            tri_coverage = [data[0], data[1], data[2]]
    #except:
    #    rospy.logwarn("Failed to receive coverage msg.")

    

@flaskServer.route('/snowlevel', methods=['GET'])
def get_snow_level():
    # Static test values
    global current_level, tri_coverage
    data = {
        "snow_level": current_level,
        "segment_coverage": tri_coverage
    }
    rospy.loginfo(f"Sending Static Data: {data}")  # Debugging print
    return jsonify(data)

if __name__ == '__main__':
    rospy.init_node('WebServerNode')
    rospy.Subscriber('/snowcap/snowsegments', Int32MultiArray, coverage_callback)
    rospy.Subscriber('/snowcap/snowlevel', Float64, level_callback)
    flaskServer.run(debug=True, host='0.0.0.0', port=5000)
    rospy.spin()
