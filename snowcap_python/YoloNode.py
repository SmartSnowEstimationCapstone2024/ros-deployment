# ros imports
import rospy, rospkg
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout, Int32MultiArray
# python imports
import os
import torch
from snowcap_python.RosCapture import RosVideoCapture
from ultralytics import YOLO

# initalize video capture node and object
rospy.init_node("YoloNode")
rgb_stream = RosVideoCapture(topic="/camera/color/image_raw", encoding="passthrough", as_type=None)
# initalize model
device = torch.device('cpu')
if torch.cuda.is_available():
    device = torch.device('cuda')
    rospy.loginfo("YoloNode now using device cuda.")
else:
    rospy.logwarn("ResNetNode unable to use cuda, falling back to cpu.")
yoloModel = YOLO(os.path.join(rospkg.RosPack().get_path('snowcap'), 'models/YoloModel.pt'))
yoloModel.to(device)

def coverageInterpreter():
    # deal with defining the message
    msgDim = [MultiArrayDimension()]
    msgDim[0].label = "Coverage Pixels"
    valid_cap, img = rgb_stream.read()
    while not valid_cap:
        valid_cap, img = rgb_stream.read()
    msgDim[0].size = img.shape[1]
    msgDim[0].stride = img.shape[1]
    layout = MultiArrayLayout()
    layout.dim = msgDim
    msg = Int32MultiArray()
    msg.layout = layout

    pubTopic = rospy.Publisher("/snowcap/snowsegments", Int32MultiArray, queue_size=1)
    pubRate = rospy.Rate(10)
    while not (rospy.is_shutdown() or rospy.is_shutdown_requested()):
        valid_cap, img = rgb_stream.read()
        if valid_cap:
            output = torch.full((1,),0, dtype=torch.int32)
            results = yoloModel.predict(img)
            for result in results:
                if (result.masks != None):
                    for mask in result.masks:
                        if output.size(dim=0) == 1:
                            output = torch.full((mask.data.shape[2],), 0, dtype=torch.int32).to(device)
                        output = torch.add(output, torch.sum(mask.data, 1, dtype=torch.int32).view(-1))
            if len(results) != 0:
                msg.data = output
                pubTopic.publish(msg)
        pubRate.sleep()

if __name__ == '__main__':
    try:
        coverageInterpreter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass