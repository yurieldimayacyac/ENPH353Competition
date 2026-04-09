#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
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
        self.root.configure(bg="#2c3e50")
        self.root.geometry("1400x800") # Set window size
        self.start_coords = FizzDetectiveController.startcoords # Access class attribute for start coordinates
        self.clue_history = []

        self.video_window = tk.Frame(self.root)
        self.video_window.pack(pady=10, fill='x')
        self.video_window.configure(bg="#2c3e50")

        # Main Camera Feed
        self.label_main = tk.Label(self.video_window, text="Main Camera Feed (/B1/rrbot/camera1/image_raw)")
        self.label_main.grid(row=0, column=0, padx=5) # Label at Row 0
        self.view_main = tk.Label(self.video_window)
        self.view_main.grid(row=1, column=0, padx=5)  # Image at Row 1
        self.view_main.grid_propagate(False)

        # Secondary Camera Feed
        self.label_clue = tk.Label(self.video_window, text="Debug Feed (/debug_clue)")
        self.label_clue.grid(row=0, column=1, padx=5) # Label at Row 0
        self.view_clue = tk.Label(self.video_window)
        self.view_clue.grid(row=1, column=1, padx=5)  # Image at Row 1
        self.view_clue.grid_propagate(False)

        # Master CONTROLS
        self.master_frame = tk.LabelFrame(self.root, text=" COMPETITION MASTER CONTROL ", fg="white", bg="#2c3e50", font=("Arial", 12, "bold"))
        self.master_frame.pack(pady=10, fill="x", padx=20)

        # Large START Button
        tk.Button(self.master_frame, text="▶ START SCOREBOARD", bg="#27ae60", fg="white", font=("Arial", 14, "bold"),
                  command=self.start_competition, width=25).pack(side="left", padx=20, pady=10)

        # Large STOP Button
        tk.Button(self.master_frame, text="■ STOP SCOREBOARD", bg="#c0392b", fg="white", font=("Arial", 14, "bold"),
                  command=self.stop_competition, width=25).pack(side="left", padx=20, pady=10)

        # Teleport to Start
        tk.Button(self.master_frame, text="↺ RESPAWN AT START", bg="#2980b9", fg="white", font=("Arial", 12),
                  command=lambda: spawn_position(FizzDetectiveController.startcoords), width=20).pack(side="right", padx=20)

        # Teleport to Clue Boards
        self.teleport_frame = tk.LabelFrame(self.root, text=" TELEPORT TO CLUE BOARDS ", fg="white", bg="#2c3e50")
        self.teleport_frame.pack(pady=10, fill="x", padx=20)

        # Coordinates for the 8 boards
        clue_coords = {
            "Board 1": [5.798047, 1.940394, 0.1, 0, 0, -1.589, 1.589],
            "Board 2": [5.277280, -1.069720, 0.1, 0, 0, -2.1, 2.1],
            "Board 3": [4.318, -1.55, 0.1, 0, 0, -0.97, 0.242],
            "Board 4": [0.615, -0.72, 0.1, 0, 0, -0.2973, -0.954],
            "Board 5": [0.66, 1.711, 0.1, 0, 0, 0.36, -0.93],
            "Board 6": [-3.18, 1.5, 0.1, 0, 0, 0.94, 0.34],
            "Board 7": [-4.08, -1.865, 0.1, 0, 0, -0.035, 0.999],
            "Board 8": [-1.177, -1.2, 1.85, 0, 0, 0, 1.0]
        }

        for i, (name, pose) in enumerate(clue_coords.items()):
            btn = tk.Button(self.teleport_frame, text=name, width=12, bg="#34495e", fg="white",
                            command=lambda p=pose: spawn_position(p))
            btn.grid(row=0, column=i, padx=5, pady=10)

        # Teleport to Pink Lines
        self.pink_line_frame = tk.LabelFrame(self.root, text=" TELEPORT TO PINK LINES ", fg="white", bg="#2c3e50")
        self.pink_line_frame.pack(pady=10, fill="x", padx=20)

        # Coordinates for your Pink Lines
        pink_line_coords = {
            "Pink Line 1": [0.48664, -0.159, 0.2, 0, 0, 0, 1.0],
            "Pink Line 2": [-3.82, 0.49, 0.2, 0, 0, 0, -1.02],
            "Pink Line 3": [-4.14, -2.31, 0.2, 0, 0, 0, 1.0]
        }

        for i, (name, pose) in enumerate(pink_line_coords.items()):
            btn = tk.Button(self.pink_line_frame, text=name, width=12, bg="#9b59b6", fg="white",
                            command=lambda p=pose: spawn_position(p))
            btn.grid(row=0, column=i, padx=5, pady=10)

        # Log and Status
        self.clue_label = tk.Label(self.root, text="Ready to Launch", font=("Arial", 18), bg="#2c3e50", fg="yellow")
        self.clue_label.pack(pady=5)

        self.log_frame = tk.Frame(self.root, bg="#2c3e50")
        self.log_frame.pack(pady=10, fill="both", expand=True, padx=20)
        
        self.clue_log = tk.Text(self.log_frame, height=6, state='disabled', bg="#1e1e1e", fg="#00ff00", font=("Courier", 10))
        self.clue_log.pack(side="left", fill="both", expand=True)

        # Publishers
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.ready = rospy.Publisher('/gui_ready', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.update_main_feed, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/debug_clue', Image, self.update_clue_feed)
        rospy.Subscriber('/clue_type', String, self.update_clue_type)
        rospy.Subscriber('/clue_value', String, self.update_clue_val)
    

    def start_competition(self):
        msg = "Team_14,YURIEL,0,NA"
        self.score_pub.publish(String(data=msg))
        rospy.loginfo("Published start message to scoreboard: " + msg)
        self.add_to_log(">>> SCOREBOARD STARTED (0,NA)")
        self.clue_label.config(text="TIMER RUNNING", fg="#2ecc71")
        self.ready.publish(String(data="GUI_READY")) # Notify controller that GUI is ready to receive updates
        
    def stop_competition(self):
        msg = "Team_14,YURIEL,-1,NA"
        self.score_pub.publish(String(data=msg))
        self.add_to_log("<<< SCOREBOARD STOPPED (-1,NA)")
        self.clue_label.config(text="TIMER STOPPED", fg="#e74c3c")

    def add_to_log(self, text):
        self.clue_log.config(state='normal')
        self.clue_log.insert(tk.END, f" >> {text}\n")
        self.clue_log.see(tk.END)
        self.clue_log.config(state='disabled')

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
            cv_image = cv2.resize(cv_image, (640,320))
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            img = PILImage.fromarray(rgb_image)
            imgtk = ImageTk.PhotoImage(image=img)

            self.view_clue.configure(image=imgtk)
            self.view_clue.imgtk = imgtk
        except Exception as e:
            pass

    def update_clue_type(self, msg):
        detected_string = msg.data
        
        self.clue_label.config(text=f"CLUE TYPE: {detected_string}")
        
    def update_clue_val(self, msg):
        detected_string = msg.data

        if detected_string in self.clue_history:
            return

        self.clue_label.config(text=f"CLUE VALUE: {detected_string}")
        # Add to the visual log so you can see a history of found clues
        self.add_to_log(f"Found Value: {detected_string}")

        self.clue_history.append(detected_string) # Keep track of found clues to prevent duplicates

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    try:
        gui = CompetitionGui()
        gui.run()
    except rospy.ROSInterruptException:
        pass