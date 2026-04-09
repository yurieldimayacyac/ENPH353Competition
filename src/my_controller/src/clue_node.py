#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os

from clue_model import ClueBoardDetector

class ClueNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.alphabet = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        reference_path = os.path.expanduser('~/ros_ws/src/my_controller/src/img/reference_clue.png')
        self.reference_img = cv2.imread(reference_path)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = ClueBoardDetector().to(self.device)

        weights_path = os.path.expanduser('~/ros_ws/src/my_controller/src/model.pth')
        self.model.load_state_dict(torch.load(weights_path, map_location=self.device))
        self.model.eval()

        self.sub = rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.process_frame, queue_size=1, buff_size=2**24)
        self.debug_pub = rospy.Publisher('/debug_clue', Image, queue_size=1) # publish to topic /debug_clue
        self.type_pub = rospy.Publisher('/clue_type', String, queue_size=1)
        self.val_pub = rospy.Publisher('/clue_value', String, queue_size=1)

        self.frame_counter = 0

    def process_frame(self, msg):
        self.frame_counter += 1
        if self.frame_counter % 20 != 0:  # Process every 20th frame
            return
        
        try: 
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image message: {e}")
            return

        found, img, board_cont, mask, (x1,y1), blue_pixel_count = self.hsv_filter(frame)
        hud_img = frame.copy()

        if found and blue_pixel_count > 45000:
            # Change Coordinates back to Uncropped Coordinates for HUD
            display_cnt = board_cont.copy()
            display_cnt[:, :, 0] += x1  
            display_cnt[:, :, 1] += y1

            cv2.drawContours(hud_img, [display_cnt], -1, (0,255,0), 2) # draw detected contour on HUD
            M = cv2.moments(board_cont)

            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cX_global = cX + x1

                offset_x = cX_global - (hud_img.shape[1] // 2)

                cv2.drawMarker(hud_img, (cX, cY), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(hud_img, f"Offset: {offset_x}", (cX + 10, cY - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            new_img = self.straighten_board_geom(img, board_cont)

            if new_img is not None:
                pred_type, pred_val = self.character_split(new_img)

                valid_string = False

                for pred_string in [pred_type, pred_val]:
                    if 4 <= len(pred_string) < 13:
                        valid_string = True

                if valid_string:
                    self.type_pub.publish(String(data="")) # Publish empty string to indicate invalid read
                    self.val_pub.publish(String(data="")) # Publish empty string to indicate invalid read

                    self.type_pub.publish(String(data=pred_type))
                    self.val_pub.publish(String(data=pred_val))
                    cv2.putText(hud_img, f"READ: {pred_string}", (170, 80), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(hud_img, "bgr8"))

            else: 
                rospy.logwarn("Phase 2 Failed: Could not find 4 corners")

    def character_split(self, warped_board):
        target_height, target_width = warped_board.shape[:2]
        img_gray = cv2.cvtColor(warped_board, cv2.COLOR_BGR2GRAY)

        # Split into Top and Bottom
        top_half = img_gray[0 : (target_height//2) - 10, :]
        bottom_half = img_gray[(target_height//2) + 10 : target_height, :]

        halves = [top_half, bottom_half]

        half_images = [[], []]
        half_layouts = [[], []]

        for string_ind, zone_img in enumerate(halves):
            blurred = cv2.GaussianBlur(zone_img, (3, 3), 0)
            _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Value Setup for Space Detection
            space_thresh = 20
            last_x_end = None

            # Sort characters left-to-right
            contours = sorted(contours, key=lambda ctr: cv2.boundingRect(ctr)[0])

            for ctr in contours:
                x, y, w, h = cv2.boundingRect(ctr)
                aspect_ratio = w / float(h)

                # Filter noise based on tested notebook values
                if h > 15 and 0.1 < aspect_ratio < 1.5:

                    # Space Detection
                    if last_x_end is not None:
                        gap = x - last_x_end
                        if gap > space_thresh:
                            half_layouts[string_ind].append("SPACE")

                    last_x_end = x + w

                    letter_crop = thresh[y:y+h, x:x+w]

                    # Square Padding to maintain aspect ratio
                    size = max(h, w) + 4
                    square = np.zeros((size, size), dtype=np.uint8)
                    ax, ay = (size - w) // 2, (size - h) // 2
                    square[ay:ay+h, ax:ax+w] = letter_crop

                    # Resize to 32x32 for the CNN
                    letter_resize = cv2.resize(square, (32, 32))

                    # Normalize to (-1.0, 1.0)
                    img_float = letter_resize.astype(np.float32) / 255.0
                    img_final = (img_float - 0.5) / 0.5
                    half_images[string_ind].append(img_final)

                    # Add "CHAR" to character layout
                    half_layouts[string_ind].append("CHAR")

            last_x_end = None  # reset for next half

        final_strings = ["",""]

        for half in range(2):
            if len(half_images[half]) > 0:
                # Prepare batch for Torch
                batch_tensor = torch.tensor(np.array(half_images[half])).unsqueeze(1).to(self.device)

                with torch.no_grad():
                    output = self.model(batch_tensor)
                    predictions = torch.argmax(output, dim=1)

                char_index = 0
                detected_string = ""

                for character in half_layouts[half]:
                    if character == "SPACE":
                        detected_string += " "
                    elif character == "CHAR" and char_index < len(predictions):
                        detected_string += self.alphabet[predictions[char_index].item()]
                        char_index += 1

                final_strings[half] = detected_string

        return final_strings[0], final_strings[1]
        
    def hsv_filter(self, cv2_image):
        x1, y1 = 0, 0
        blue_pixel_thresh = 10000
        blue_board_found = False
        crop_img = None
        local_cnt = None # Initialize local contour

        image_height, image_width = cv2_image.shape[:2]
        hsv_img = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

        # Blue threshold values
        lower_blue = np.array([100, 120, 95])
        upper_blue = np.array([120,255,255])

        mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
        blue_pixel_count = np.sum(mask == 255)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            roi = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(roi)
            buffer = 15 # Increased buffer to ensure corners aren't clipped

            # Define crop boundaries
            y1, y2 = max(0, y - buffer), min(image_height, y + h + buffer)
            x1, x2 = max(0, x - buffer), min(image_width, x + w + buffer)

            crop_img = cv2_image[y1:y2, x1:x2]

            # Shift Contour to Local Coordinates for Character Splitting
            local_cnt = roi.copy()
            local_cnt[:, :, 0] -= x1
            local_cnt[:, :, 1] -= y1

        if blue_pixel_count > blue_pixel_thresh:
            blue_board_found = True
        else: 
            blue_board_found = False

        return blue_board_found, crop_img, local_cnt, mask, (x1,y1), blue_pixel_count

    def straighten_board_geom(self, crop_img, roi_contour):
        # Simplify contour to find 4 corners
        # 0.02 precision factor for finding corners
        peri = cv2.arcLength(roi_contour, True)
        approx = cv2.approxPolyDP(roi_contour, 0.02 * peri, True)

        if len(approx) == 4:
            # Reshape to (4, 2) and sort the points
            pts = approx.reshape(4, 2)
            rect = np.zeros((4, 2), dtype="float32")

            # Top-left has smallest sum, Bottom-right has largest sum
            s = pts.sum(axis=1)
            rect[0] = pts[np.argmin(s)]
            rect[2] = pts[np.argmax(s)]

            # Top-right has smallest difference, Bottom-left has largest difference
            diff = np.diff(pts, axis=1)
            rect[1] = pts[np.argmin(diff)]
            rect[3] = pts[np.argmax(diff)]

            dst = np.array([
                [0, 0],
                [640, 0],
                [640, 320],
                [0, 320]], dtype="float32")

            # Calculate the transformation matrix
            M = cv2.getPerspectiveTransform(rect, dst)

            # Warp the actual crop image
            straightened = cv2.warpPerspective(crop_img, M, (640, 320))
            return straightened

        else:
            print(f"Failed to find 4 corners. Found {len(approx)} instead.")
            return None
                
def main(args=None):
        rospy.init_node('clue_node')
        clue_node = ClueNode()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

if __name__ == '__main__':        
        main()