#!/usr/bin/env python3
import rospy
import cv2
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from driver import RobotDriver
from robot_utils import spawn_position
from clue_node import ClueNode

class FizzDetectiveController:
    # Class Attributes
    startcoords = [5.5, 2.498, 0.1, 0, 0, -0.707, 0.707]

    def __init__(self):
        self.bridge = CvBridge()
        self.driver = RobotDriver(kp=0.02, ki=0.00001, kd = 0.005, target_v=0.3)
        self.ready = False
        self.clue_detector = ClueNode() # Initialize the clue detector
        self.at_clue_board = False 
        
        # Subscriber (Ensure the topic matches your Gazebo robot)
        self.sub = rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.process_frame)

    def check_for_hazards(self, frame):
        # Your YOLO logic goes here
        return False

    def process_frame(self, msg):
        if not self.ready:
            rospy.loginfo("Controller is not ready. Waiting to process frames.")
            return 

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        # Hazard Detection
        hazard_detected = self.check_for_hazards(frame)
        
        # Inside process_frame
        #self.driver.update_drive(frame, thresh_low=170,thresh_high=255, hazard_detected=False)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('fizz_detective')
        controller = FizzDetectiveController()
        
        # Reset position FIRST
        spawn_position(controller.startcoords)
        
        # Indicate ready to process frames
        controller.ready = True

        rospy.loginfo("FizzDetectiveController is now running. Clue Detection running in background")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass