from RosCapture import RosVideoCapture
import cv2
from ultralytics import YOLO
import torch
from torchvision import models, transforms
from torchvision.io import read_image
from Rosmaster_Lib import Rosmaster
from flask import Flask, jsonify

rosmaster = Rosmaster()
rgb_stream = RosVideoCapture(topic="/camera/rgb/image_raw", node_name="snowcap_rgb_capture", encoding="bgr8", as_type=None)

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
checkpoint = torch.load('best_model.pth', map_location=device)
resnetModel = models.resnet50(weights=checkpoint)
resnetModel.fc = torch.nn.Linear(in_features=2048, out_features=1)
resnetModel.eval()
transform = transforms.Compose([transforms.ToTensor()])

#yoloModel = YOLO('best.pt')

flaskServer = Flask(__name__)

@flaskServer.route('/snowlevel', methods=['GET'])
def get_snow_level():
    print("Recieved request!")
    valid_cap, img = rgb_stream.read()
    data = {"snow_level": 0}
    if valid_cap:
        with torch.no_grad():
            img = transform(img)
            output = resnetModel(img[None,:,:,:]).item()*1.78 + 0.1
            data['snow_level'] = output
    return jsonify(data)

if __name__ == '__main__':
    flaskServer.run(debug=True, host='0.0.0.0', port=5000)
