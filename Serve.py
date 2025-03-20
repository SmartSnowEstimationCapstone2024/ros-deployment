from RosCapture import RosVideoCapture
import cv2
from ultralytics import YOLO
import torch
from torchvision import models, transforms
from torchvision.io import read_image
from Mod_Rosmaster_Lib import Rosmaster
from flask import Flask, jsonify

rosmaster = Rosmaster()
rgb_stream = RosVideoCapture(topic="/camera/rgb/image_raw", node_name="snowcap_rgb_capture", encoding="bgr8", as_type=None)

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
checkpoint = torch.load('best_model.pth', map_location=device)
resnetModel = models.resnet50(weights=checkpoint)
resnetModel.fc = torch.nn.Linear(in_features=2048, out_features=1)
resnetModel.eval()
transform = transforms.Compose([transforms.ToTensor()])

yoloModel = YOLO('best.pt')

flaskServer = Flask(__name__)

def calculate_coverage(segments, detections, width, height):
    segment_height = height // 3
    coverage = [0, 0, 0]
    
    for detection in detections:
        x1, y1, x2, y2, conf, cls = detection.tolist()
        bbox_area = (x2 - x1) * (y2 - y1)
        
        for i in range(3):
            seg_top = i * segment_height
            seg_bottom = (i + 1) * segment_height
            
            inter_top = max(y1, seg_top)
            inter_bottom = min(y2, seg_bottom)
            
            if inter_top < inter_bottom:  # There is an intersection
                inter_area = (x2 - x1) * (inter_bottom - inter_top)
                coverage[i] += inter_area
    
    for i in range(3):
        total_area = width * segment_height
        coverage[i] = (coverage[i] / total_area) * 100  # Convert to percentage
    
    return coverage

@flaskServer.route('/snowlevel', methods=['GET'])
def get_snow_level():
    print("Received request!")

    # Static test values
    data = {
        "snow_level": 0.6,  # Static snow depth (adjust as needed)
        "segment_coverage": [20.0, 30.0, 40.0]  # Static coverage percentages
    }

    print(f"Sending Static Data: {data}")  # Debugging print
    return jsonify(data)

# @flaskServer.route('/snowlevel', methods=['GET'])
# def get_snow_level():
    # print("Received request!")
    # valid_cap, img = rgb_stream.read()
    # data = {"snow_level": 0, "segment_coverage": [0, 0, 0]}
    # 
    # if valid_cap:
        # with torch.no_grad():
            # img_tensor = transform(img)
            # output = resnetModel(img_tensor[None,:,:,:]).item() * 1.78 + 0.1
            # data['snow_level'] = output
            # 
            # Run YOLO model to get detections
            # results = yoloModel(img)
            # detections = results[0].boxes.data if len(results) > 0 else []
            # 
            # height, width, _ = img.shape
            # segment_coverage = calculate_coverage(3, detections, width, height)
            # data['segment_coverage'] = segment_coverage
    # 
    # return jsonify(data)

if __name__ == '__main__':
    flaskServer.run(debug=True, host='0.0.0.0', port=5000)