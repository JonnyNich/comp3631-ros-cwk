# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class colourIdentifier(Node):
    def __init__(self, sensitivity):
        super().__init__('cI')
        self.sensitivity = sensitivity

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscrtopicibe to should you wish to receive image data
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.callback, 10)
                
    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv_red_mid1 = np.array([0, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        red_mask_upper = cv2.inRange(hsv_img, hsv_red_mid1, hsv_red_upper)
        hsv_red_mid2 = np.array([180, 255, 255])
        hsv_red_lower = np.array([180 - self.sensitivity, 100, 100])
        red_mask_lower = cv2.inRange(hsv_img, hsv_red_lower, hsv_red_mid2)
        red_mask = cv2.bitwise_or(red_mask_lower, red_mask_upper)

        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        green_mask = cv2.inRange(hsv_img, hsv_green_lower, hsv_green_upper)

        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        blue_mask = cv2.inRange(hsv_img, hsv_blue_lower, hsv_blue_upper)

        rg_mask = cv2.bitwise_or(red_mask, green_mask)
        rgb_mask = cv2.bitwise_or(rg_mask, blue_mask)

        filtered_img = cv2.bitwise_and(img, img, mask=rgb_mask)

        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL) 

        cv2.imshow('camera_Feed', filtered_img)

        cv2.resizeWindow('camera_Feed', 320, 240) 

        cv2.waitKey(3) 
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        # Show the resultant images you have created.
        

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():

    def signal_handler(sig, frame):
        rclpy.shutdown()
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    cI = colourIdentifier(20)

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    

# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
