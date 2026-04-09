#!/usr/bin/env python3

import rospy
import cv2
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from driver import RobotDriver
from robot_utils import spawn_position
from clue_node import ClueNode

class FizzDetectiveController:
    # Class Attributes
    startcoords = [5.5, 2.498, 0.1, 0, 0, -0.707, 0.707]

    # Team_14,YURIEL,0,NA
    # Team_14,YURIEL,1,FIVE


    def __init__(self):
        self.bridge = CvBridge()
        self.driver = RobotDriver(kp=0.02, ki=0.00001, kd = 0.005, target_v=0.3)
        self.ready = False
        self.clue_detector = ClueNode() # Initialize the clue detector
        self.at_clue_board = False 
        self.start = False
        self.submission_history = {}
        
        # Subscriber (Ensure the topic matches your Gazebo robot)
        self.sub = rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.process_frame)
        self.clue_type = rospy.Subscriber('/clue_type', String, self.clueboard_type)
        self.clue_value = rospy.Subscriber('/clue_value', String, self.clueboard_value)
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.start = rospy.Subscriber('/gui_ready', String, self.set_ready) # Subscribe to GUI ready signal
        rospy.sleep(1) # ensure publishers and subscribers are set up before processing frames

        # Initializing Clue Board Type/Value
        self.latest_type = None
        self.latest_value = None
        self.current_location_id = 0

    def set_ready(self, msg):
        if msg.data == "GUI_READY":
            self.start = True
            rospy.loginfo("Received GUI_READY signal. Controller is now ready to process frames.")

    def check_for_hazards(self, frame):
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

    def clueboard_type(self, msg):
        # Location ID: (1-8) OR (0: start and -1: end)
        self.latest_type = msg.data

        # Find Location ID
        if msg.data == "SIZE":
            self.current_location_id = 1
        elif msg.data == "VICTIM":
            self.current_location_id = 2
        elif msg.data == "CRIME":
            self.current_location_id = 3
        elif msg.data == "TIME":
            self.current_location_id = 4
        elif msg.data == "PLACE":
            self.current_location_id = 5
        elif msg.data == "MOTIVE":
            self.current_location_id = 6
        elif msg.data == "WEAPON":
            self.current_location_id = 7
        elif msg.data == "BANDIT":
            self.current_location_id = 8
        else:
            self.current_location_id = 0
            return

        self.attempt_submission()

    def clueboard_value(self, msg):
        self.latest_value = msg.data
        self.attempt_submission()

    def attempt_submission(self):
        if self.start and self.latest_type and self.latest_value:
            rospy.loginfo(f"Pair complete! Submitting {self.latest_type}: {self.latest_value}")
            
            self.submit_clue(self.current_location_id, self.latest_value)
            
            # Reset for next clue
            self.latest_type = None
            self.latest_value = None
            self.current_location_id = 0

    def submit_clue(self, location_id, prediction):
        formatted_prediction = prediction.replace(" ", "").upper()

        # Do nothing if already logged this clue for this location
        if location_id in self.submission_history and self.submission_history[location_id] == formatted_prediction:
            return
        
        team_id = "Team_14"
        password = "YURIEL"
        clue_message = f"{team_id} detected clue at Location {location_id}: {formatted_prediction}"
        score_msg = f"{team_id},{password},{location_id},{formatted_prediction}"
        self.score_pub.publish(String(data=score_msg))
        rospy.loginfo(f"Submitted clue: {score_msg}")
        self.submission_history[location_id] = formatted_prediction

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