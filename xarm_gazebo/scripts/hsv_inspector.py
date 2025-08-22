#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class HSVInspector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self.image_callback)
        self.frame = None
        cv2.namedWindow("Click to Inspect HSV")
        cv2.setMouseCallback("Click to Inspect HSV", self.mouse_callback)

    def image_callback(self, msg):
        self.frame = self.bridge.compressed_imgmsg_to_cv2(msg)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.frame is not None:
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            pixel = hsv[y, x]
            print(f"Clicked at ({x},{y}) - HSV: {pixel}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.frame is not None:
                cv2.imshow("Click to Inspect HSV", self.frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('hsv_inspector')
    hsv_inspector = HSVInspector()
    hsv_inspector.run()
