# Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab2)

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
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import signal
from math import sin, cos

class BlueBoxWatcher(Node):
    def __init__(self, sensitivity, area):
        super().__init__('blue_box_watcher')
        
        self.goal_completed = False
        
        self.blue_found = False
        self.sensitivity = sensitivity
        self.area = area
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        
        self.box_center_x = -1
        self.box_center_y = -1
        self.img_center_x = -1
        self.img_center_y = -1
        self.contour_area = -1
        
    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s

        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        self.publisher.publish(desired_velocity)
        
    
    def image_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img_height, img_width, _ = img.shape
        self.img_center_x, self.img_center_y = img_width / 2, img_width / 2
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
        
        # Calculate contours
        contours, hierarchy = cv2.findContours(blue_mask,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        if len(contours) > 0:
            # Get largest contour
            c = max(contours, key=cv2.contourArea)

            #Moments can calculate the center of the contour
            M = cv2.moments(c)
            
            # cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            # print (cx, cy)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            self.contour_area = cv2.contourArea(c)
            # print (cv2.contourArea(c))
            if cv2.contourArea(c) > 100:

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                (x, y), radius = cv2.minEnclosingCircle(c)
                self.box_center_x, self.box_center_y = int(x),int(y) 
                # print(f"box_center: {self.box_center_x},{self.box_center_y}. img_center: {self.img_center_x}. {self.img_center_y}")
                # print (f"Diff. X: {abs(self.box_center_x - self.img_center_x)}, Y: {abs(self.box_center_y - self.img_center_y)}")
                radius = int(radius) 

                cv2.circle(img,(self.box_center_x, self.box_center_y),radius,(255,255,0) ,1)

                # Then alter the values of any flags, and stop if in motion
                if self.blue_found == False:
                    self.blue_found = True
                    self.stop()
                    print ("blue found")
            else:
                self.blue_found = False   

        # filtered_img = cv2.bitwise_and(img, img, mask=rgb_mask)

        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL) 

        cv2.imshow('camera_Feed', img)

        cv2.resizeWindow('camera_Feed', 320, 240) 

        cv2.waitKey(3) 
    
    def search_for_blue(self):
        print ("Searching for blue")
        self.blue_found = False # We might have spotted blue previously, but we want to disregard that temp[orarily]
        # returns True if blue found, False if not
        turn = Twist()
        turn.angular.z = np.pi / 2
        for i in range(40):
            if self.blue_found == False:
                self.publisher.publish(turn)
                self.rate.sleep()
            else:
                print ("Found blue")
                return True
        else:
            # This runs if a full rotation happens without a blue being found
            # Move to next goal
            print ("Couldn't find blue")
            return False
        
    def approach_blue(self):
        print ("Approaching blue")
        # Go towards blue if detected
        # Making sure there is indeed a lbue
        if self.blue_found == True:
            # At this point, there should be no more rotation or movement
            # Line up the robot so it's somewaht central to the ox
            while (abs(self.box_center_x - self.img_center_x) > 5):
                turn = Twist()
                if (self.box_center_x - self.img_center_x) > 0:      
                    print ("clockwise")
                    turn.angular.z = np.pi / -40
                else:
                    print ("counter-clockwise")
                    turn.angular.z = np.pi / 40
                for _ in range(5):
                    self.publisher.publish(turn)
                    self.rate.sleep()
            self.stop()
            # If in line
            while (self.contour_area < 250000 or self.contour_area > 400000):
                if self.contour_area > 400000:
                    # Too close to object, need to move backwards
                    print("backward")
                    self.too_close = True
                    self.walk_backward()
                elif self.contour_area <= 230000 :
                    print("forward")
                    self.too_close = False
                    # Too far away from object, need to move forwards
                    self.walk_forward()
            self.stop()

    def send_goal(self, x, y, yaw):
        self.goal_completed = False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)
        
        self.get_logger().info(f"Next goal: {goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}")

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_completed = True
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.goal_completed = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
    
    def start_search(self):
        if self.running == False:
            self.running = True
            print ("searching for blue")
            first_goal = self.goals.pop(0)
            self.send_goal(first_goal[0], first_goal[1], first_goal[2])

def main():
    def signal_handler(sig, frame):
        bbw.stop()
        rclpy.shutdown()
    
    # Instantiate colour identifier
    rclpy.init(args=None)
   
    bbw = BlueBoxWatcher(20, 10)

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(bbw,), daemon=True)
    thread.start()
    
    goals = [[8.87, -13.2, 0], [-8.82, -13.8, 0], [-11.3, 4.07, 0], [7.37, 5.42, 0]]

    try:
        while rclpy.ok():
            while len(goals) > 0:
                next_goal = goals.pop(0)
                bbw.send_goal(next_goal[0], next_goal[1], next_goal[2])
                # Wait for goal to complete
                while bbw.goal_completed == False:
                    pass
                if bbw.search_for_blue() == True:
                    bbw.approach_blue()
                    goals = []
                    break
                
            
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
