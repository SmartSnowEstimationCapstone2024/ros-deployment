import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

class RosVideoCapture:
    def __init__(self, topic="/camera/depth/image_raw", node_name="ros_img_capture", encoding="passthrough", as_type="uint8"):
        self.topic = topic
        self.bridge = CvBridge()
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.node_name = node_name
        self.encoding = encoding
        self.as_type = as_type

        # Start ROS subscriber in a separate thread
        self.thread = threading.Thread(target=self._start_ros_node, daemon=True)
        self.thread.start()

    def _start_ros_node(self):
        rospy.init_node(self.node_name, anonymous=True, disable_signals=True)
        rospy.Subscriber(self.topic, Image, self._callback)
        rospy.spin()

    def _callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
            if self.as_type is not None:
                frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX).astype(self.as_type)
            with self.lock:
                self.frame = frame
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()

    def release(self):
        self.running = False
        rospy.signal_shutdown("Releasing RosVideoCapture")