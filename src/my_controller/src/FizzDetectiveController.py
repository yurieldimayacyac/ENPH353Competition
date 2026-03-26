#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SherROS_Holmes:
    def __init__(self):
        rospy.init_node('fizz_detective_node')
        self.cmd_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size = 1)
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size = 1)

        while self.cmd_pub.get_num_connections() < 1 or self.score_pub.get_num_connections() < 1:
            if rospy.is_shutdown(): # return immediately if ROS isn't even operating
                return
            rospy.loginfo("Waiting for simulation and tracker to connect")
            rospy.sleep(0.5)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw",Image, self.camera_callback)

        self.latest_frame = None
        
        rospy.loginfo("Waiting for camera feed")
        while self.latest_frame is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("Ready to Start Simulation")
        time.sleep(1)
        self.clue_found = False
        self.time_trials = False
        time.sleep(1)

    def move(self):

        # start timer
        sim_time = 5
        self.score_pub.publish("Team14,sherrosholmes14,0,NA")
        rospy.loginfo("Robot Started!")

        # start moving forward
        move_cmd = Twist() # create instance of Twist message
        move_cmd.linear.x = 0.5 # (0.5 m/s)

        rate = rospy.Rate(20)

        start_time = rospy.get_time()
        rospy.sleep(0.1)
        while rospy.get_time() - start_time < sim_time and not rospy.is_shutdown():
            if self.cmd_pub.get_num_connections() > 0:
                self.cmd_pub.publish(move_cmd)
            else:
                rospy.logwarn("no one listening! holding position")
            rate.sleep()

        # stop robot
        self.cmd_pub.publish(Twist()) # publish to /B1/cmd_vel (then to Gazebo)
        time.sleep(0.1) # give ROS time to send 0 velocity
        self.score_pub.publish("Team14,sherrosholmes14,-1,NA")
        rospy.loginfo("Robot Stopped!")

    def camera_callback(self,data):

        ## TODO: potentially something in here that calls stop() if 'clue' found
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.latest_frame = cv_image

            import cv2

            cv2.namedWindow("Vision", cv2.WINDOW_NORMAL)
            cv2.moveWindow("Vision", 500, 100)

            cv2.imshow("Vision", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            print(e)

if __name__ == '__main__':
    try:
        detective = SherROS_Holmes()
        detective.move()
    except rospy.ROSInterruptException:
        pass


    

        