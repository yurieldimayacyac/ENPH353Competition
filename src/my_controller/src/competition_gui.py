#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tkinter as tk
from PIL import Image as PILImage, ImageTk

from robot_utils import spawn_position
from FizzDetectiveController import FizzDetectiveController

class CompetitionGui:
    def __init__(self):
        rospy.init_node('competition_gui', anonymous=True) # anonymous = True prevents repeat names
        self.bridge = CvBridge()
        self.root = tk.Tk()
        self.root.title("Fizz Detective Competition GUI")
        self.root.geometry("1400x800") # Set window size
        self.start_coords = FizzDetectiveController.startcoords # Access class attribute for start coordinates

        self.video_window = tk.Frame(self.root)
        self.video_window.pack(pady=10, fill='x')

        # Main Camera Feed
        self.label_main = tk.Label(self.video_window, text="Main Camera Feed (/B1/rrbot/camera1/image_raw)")
        self.label_main.grid(row=0, column=0, padx=5) # Label at Row 0
        self.view_main = tk.Label(self.video_window)
        self.view_main.grid(row=1, column=0, padx=5)  # Image at Row 1

        # Secondary Camera Feed
        self.label_clue = tk.Label(self.video_window, text="Debug Feed (/debug_clue)")
        self.label_clue.grid(row=0, column=1, padx=5) # Label at Row 0
        self.view_clue = tk.Label(self.video_window)
        self.view_clue.grid(row=1, column=1, padx=5)  # Image at Row 1


        # Buttons
        self.button_frame = tk.Frame(self.root)
        self.button_frame.pack(pady=20)

        tk.Button(self.button_frame, text="RESPAWN: Start", bg="green", fg="white", 
                  command=lambda: self.respawn_at(self.start_coords), width=20).grid(row=0, column=0, padx=5)

        tk.Button(self.button_frame, text="CLUE BOARD A", bg="blue", fg="white", 
                  command=lambda: self.respawn_at([5.5, 2.1, 0, 0, 0, 0, 1]), width=20).grid(row=0, column=1, padx=5)

        tk.Button(self.button_frame, text="CLUE BOARD B", bg="blue", fg="white", 
                  command=lambda: self.respawn_at([-3.2, 4.5, 0, 0, 0, 0, 1]), width=20).grid(row=1, column=1, padx=5, pady=5)

        # Text Box
        self.clue_frame = tk.LabelFrame(self.root, text="Detected Clues Log")
        self.clue_frame.pack(pady=10, fill="both", expand=True, padx=20)

        self.clue_log = tk.Text(self.clue_frame, height=5, state='disabled') # 'disabled' prevents typing
        self.clue_log.pack(side="left", fill="both", expand=True)

        self.clue_label = tk.Label(self.root, text="Waiting for Clues...", font=("Arial", 18), bg="black", fg="white")
        self.clue_label.pack(pady=10)

        # Add a scrollbar in case the list gets long
        self.scrollbar = tk.Scrollbar(self.clue_frame, command=self.clue_log.yview)
        self.scrollbar.pack(side="right", fill="y")
        self.clue_log.config(yscrollcommand=self.scrollbar.set)

        # New Subscriber for the string data
        from std_msgs.msg import String

        # Subscribers
        rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.update_main_feed, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/debug_clue', Image, self.update_clue_feed)
        rospy.Subscriber('/detected_clue', String, self.update_clue_text)


    def respawn_at(self, pose):
        rospy.loginfo(f"Respawning at: {pose}")

        try: 
            spawn_position(pose)
        except Exception as e:
            rospy.logerr(f"Error occurred while respawning: {e}")

    def update_main_feed(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (640,320))
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            img = PILImage.fromarray(rgb_image)
            imgtk = ImageTk.PhotoImage(image=img)

            self.view_main.configure(image=imgtk)
            self.view_main.imgtk = imgtk
        except Exception as e:
            pass

    def update_clue_feed(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (320, 180))
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            img = PILImage.fromarray(rgb_image)
            imgtk = ImageTk.PhotoImage(image=img)

            self.view_clue.imgtk = imgtk
            self.view_clue.configure(image=imgtk)
        except Exception as e:
            pass

    def update_clue_text(self, msg):
        detected_string = msg.data
        
        self.clue_label.config(text=f"CLUE FOUND: {detected_string}")
        
        rospy.loginfo(f"GUI updated with clue: {detected_string}")

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    try:
        gui = CompetitionGui()
        gui.run()
    except rospy.ROSInterruptException:
        pass