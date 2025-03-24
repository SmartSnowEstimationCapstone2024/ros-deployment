# ros imports
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# python imports
import cv2
import os
import re

path = rospkg.RosPack().get_path('snowcap')
rematcher = re.compile('^(?!\\.gitignore)')
images = list(filter(rematcher.match, os.listdir(os.path.join(path, 'test_images'))))
bridge = CvBridge()

def imagePublisher():
    pubTopic = rospy.Publisher("/camera/color/image_raw", Image)
    pubRate = rospy.Rate(10)
    while not (rospy.is_shutdown() or rospy.is_shutdown_requested()):
        for i in images:
            try:
                img = cv2.imread(os.path.join(path, 'test_images', i))
                img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
                pubTopic.publish(img)
                pubRate.sleep()
            except:
                pass


if __name__ == '__main__':
    try:
        rospy.init_node("TestCamNode")
        imagePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass