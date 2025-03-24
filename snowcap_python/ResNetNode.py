# ros imports
import rospy, rospkg
from std_msgs.msg import Float64
# python imports
import os
from snowcap_python.RosCapture import RosVideoCapture
import torch
from torchvision import models, transforms

# initalize video capture node and object
rospy.init_node("ResNetNode")
rospy.loginfo("starting cam node")
rgb_stream = RosVideoCapture(topic="/camera/color/image_raw", encoding="passthrough", as_type=None)
# initalize model
device = torch.device('cpu')
if torch.cuda.is_available():
    device = torch.device('cuda')
    rospy.loginfo("ResNetNode now using device cuda.")
else:
    rospy.logwarn("ResNetNode unable to use cuda, falling back to cpu.")
checkpoint = torch.load(os.path.join(rospkg.RosPack().get_path('snowcap'), 'models/ResNetModel.pth'), map_location=device)
resNetModel = models.resnet50(weights=checkpoint)
resNetModel.fc = torch.nn.Linear(in_features=2048, out_features=1)
resNetModel.eval()
transform = transforms.Compose([transforms.ToTensor()])

def levelInterpreter():
    pubTopic = rospy.Publisher("/snowcap/snowlevel", Float64, queue_size=1)    
    pubRate = rospy.Rate(10)
    while not (rospy.is_shutdown() or rospy.is_shutdown_requested()):
        valid_cap, img = rgb_stream.read()
        if valid_cap:
            with torch.no_grad():
                img = transform(img)
                output = resNetModel(img[None,:,:,:]).item() * 1.78
                pubTopic.publish(Float64(output))
        pubRate.sleep()

if __name__ == '__main__':
    try:
        levelInterpreter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
