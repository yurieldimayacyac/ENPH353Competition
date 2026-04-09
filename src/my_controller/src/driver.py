#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class RobotDriver:
    def __init__(self, kp=0.008, ki=0.0001, kd=0.003, target_v=0.2):
        self.pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        # Increased base kp and kd for higher "stiffness"
        self.kp = kp
        self.ki = ki
        self.kd = kd 
        self.target_v = target_v
        
        # Memory
        self.integral = 0
        self.prev_error = 0 
        self.last_steering = 0.0
        self.ROAD_WIDTH_PX = 400 
        self.blind_count = 0
        self.max_ghost_frames = 20 # Slightly shorter to trigger recovery faster

    def update_drive(self, frame, thresh_low=200, thresh_high=255, hazard_detected=False):
        vel = Twist()
        # Initialize error to avoid the logger crash if we exit early or go blind
        current_error = self.prev_error 
        status = "UNKNOWN"

        if hazard_detected:
            vel.linear.x = 0; vel.angular.z = 0
            self.pub.publish(vel); return
        
        h, w = frame.shape[:2]
        mid = w // 2
        roi_start = int(h * 0.75) # Narrowed ROI for more immediate feedback
        roi = frame[roi_start:h, :] 
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, thresh_low, thresh_high)
        
        # Crosswalk Check
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len([cnt for cnt in contours if cv2.contourArea(cnt) > 150]) > 2:
            vel.linear.x = self.target_v; vel.angular.z = 0.0
            status = "CROSSWALK"
            self.pub.publish(vel)
            return

        # Line detection
        left_mask = mask[:, :mid]
        right_mask = mask[:, mid:]
        M_L = cv2.moments(left_mask)
        M_R = cv2.moments(right_mask)
        
        l_pos = int(M_L["m10"] / M_L["m00"]) if M_L["m00"] > 100 else None
        r_pos = (int(M_R["m10"] / M_R["m00"]) + mid) if M_R["m00"] > 100 else None

        if l_pos is None and r_pos is None:
            self.blind_count += 1
            if self.blind_count < self.max_ghost_frames:
                vel.linear.x = self.target_v * 0.6
                vel.angular.z = self.last_steering
                status = "GHOSTING"
            else:
                vel.linear.x = 0.0
                vel.angular.z = 0.85 if self.last_steering > 0 else -0.85
                status = "RECOVERING"
        else:
            self.blind_count = 0
            # CENTROID PRIORITY: Force the target to be the mathematical center
            if l_pos is not None and r_pos is not None:
                line_to_follow = (l_pos + r_pos) // 2
                self.ROAD_WIDTH_PX = r_pos - l_pos
                status = "BOTH"
            elif l_pos is not None:
                line_to_follow = l_pos + (self.ROAD_WIDTH_PX // 2)
                status = "LEFT_ONLY"
            else:
                line_to_follow = r_pos - (self.ROAD_WIDTH_PX // 2)
                status = "RIGHT_ONLY"

            current_error = line_to_follow - mid
            
            # Quadratic Boost: Amplifies steering significantly as it leaves center
            norm_err = abs(current_error) / mid
            boost = 1.0 + (1.5 * (norm_err ** 2)) 
            
            P = (self.kp * boost) * current_error
            self.integral = max(min(self.integral + current_error, 300), -300)
            I = self.ki * self.integral
            D = self.kd * (current_error - self.prev_error)
            
            steering = -(P + I + D)
            
            self.prev_error = current_error
            self.last_steering = steering
            vel.linear.x = self.target_v
            vel.angular.z = steering

        # Fixed logger: uses 'current_error' which is always defined now
        rospy.loginfo_throttle(0.2, 
            f"[{status}] Err:{current_error} L:{l_pos} R:{r_pos} Steer:{vel.angular.z:.2f}")

        self.pub.publish(vel)