#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class RobotDriver:
    def __init__(self, kp=0.005, ki=0.0001, kd=0.000, target_v=0.2):
        self.pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_v = target_v
        
        # Memory
        self.integral = 0
        self.prev_error = 0 # Stores error from the previous frame
        self.last_steering = 0.0
        self.ROAD_WIDTH_PX = 400 
        self.blind_count = 0
        self.max_ghost_frames = 25 

    def update_drive(self, frame, thresh_low=200, thresh_high=255, hazard_detected=False):
        vel = Twist()
        if hazard_detected:
            vel.linear.x = 0; vel.angular.z = 0
            self.pub.publish(vel); return
        
        h, w = frame.shape[:2]
        mid = w // 2
        roi_start = int(h * 0.7)
        roi = frame[roi_start:h, :] 
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        mask = cv2.inRange(gray, thresh_low, thresh_high)
        
        left_mask = mask[:, :mid]
        right_mask = mask[:, mid:]
        
        M_L = cv2.moments(left_mask)
        M_R = cv2.moments(right_mask)
        
        l_pos = int(M_L["m10"] / M_L["m00"]) if M_L["m00"] > 0 else None
        r_pos = (int(M_R["m10"] / M_R["m00"]) + mid) if M_R["m00"] > 0 else None

        avg_intensity = np.mean(gray)
        l_val = gray[5, l_pos] if l_pos else 0
        r_val = gray[5, r_pos - mid] if r_pos else 0 

        if l_pos is None and r_pos is None:
            self.blind_count += 1
            if self.blind_count < self.max_ghost_frames:
                vel.linear.x = self.target_v * 0.7
                vel.angular.z = self.last_steering
                status = f"GHOSTING ({self.blind_count})"
            else:
                vel.linear.x = 0.0
                vel.angular.z = -0.75 if self.last_steering > 0 else 0.75
                status = "PIVOTING (LOST)"
        else:
            self.blind_count = 0
            if l_pos is not None and r_pos is not None:
                line_to_follow = (l_pos + r_pos) // 2
                self.ROAD_WIDTH_PX = r_pos - l_pos
                status = "BOTH"
            elif l_pos is not None:
                line_to_follow = l_pos + (self.ROAD_WIDTH_PX // 2)
                status = "ONLY LEFT"
            else:
                line_to_follow = r_pos - (self.ROAD_WIDTH_PX // 2)
                status = "ONLY RIGHT"

            error = line_to_follow - mid
            
            # PID
            P = self.kp * error
            
            self.integral += error
            self.integral = max(min(self.integral, 500), -500)
            I = self.ki * self.integral
            
            d_error = error - self.prev_error
            D = self.kd * d_error
            
            # Combine all terms
            steering = -(P + I + D)
            
            # Save current error for the next frame's D calculation
            self.prev_error = error
            
            self.last_steering = steering
            vel.linear.x = self.target_v
            vel.angular.z = steering

        rospy.loginfo_throttle(0.2, 
            f"[{status}] L_px:{l_pos}({l_val}) R_px:{r_pos}({r_val}) Road_Avg:{avg_intensity:.1f} Steer:{vel.angular.z:.2f}")

        self.pub.publish(vel)