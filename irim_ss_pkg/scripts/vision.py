#! /usr/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class Vision:
    
    def __init__(self):
        print("Vision node started")
        self.bridge = CvBridge()
        rospy.init_node('image_listener')
        image_topic = "/camera/image_raw"
        # self.img_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.data = rospy.wait_for_message('/camera/image_raw', Image, timeout=5)
        self.process()


    def image_callback(self,msg):
        print("img received")
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            print("OK")

    def process(self):
        bgr_img = self.bridge.imgmsg_to_cv2(self.data, "bgr8")
        b,g,r = cv2.split(bgr_img)
        cv2.imshow("b", b)
        cv2.imshow("g", g)
        cv2.imshow("r", r)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # bitwiseAnd = cv2.bitwise_and(b,g,r)
        # cv2.imshow("merge", bitwiseAnd)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

def main():
    Vision()
    rospy.spin()

if __name__ == '__main__':
    main()