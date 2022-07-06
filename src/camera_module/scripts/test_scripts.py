#! /usr/bin/env python3

# from imutils.object_detection import non_max_suppression
from camera_msg.msg import JointState
import numpy as np
import rospy
import copy
import cv2
from sensor_msgs.msg import CompressedImage
from numpy import asarray
import mediapipe as mp
import imutils
from imutils.object_detection import non_max_suppression

class human_detection:
    def __init__(self) -> None:
        # mediapipe init
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        # HOGdecriptor init
        self.HOGCV = cv2.HOGDescriptor()
        self.HOGCV.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # camera details
        self.camera_width = 640
        self.camera_height = 480
        self.HFOV = 53.50
        self.VFOV = 41.6
        self.angle_offset = 0
        self.angle_boundary = 0
        self.count=0
        
        #topic subcribption 
        self.image_topic = '/camera/color/image_raw/compressed'
        # self.bag_file = '/home/rover/MTRX5700-Major-Project/bags/test2_scan.bag'
        self.output_topic = "human_pose"
        
        self.is_lose = True
        
        rospy.init_node('human_detection', anonymous=True)
        rospy.Subscriber(self.image_topic, CompressedImage, self.cam_callback)
        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)
        rospy.spin()

    # camera callback function will publish a human relative angle 
    def cam_callback(self, ros_data):
        # print("enter")
        #image conversion 
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cv2.imshow('cv_img', image_np)
        k = cv2.waitKey(10)
        angle = self.detect_pose(image_np, self.pose)            
        
    def detect_pose(self, frame, pose):
        frame.flags.writeable = False
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        results = pose.process(frame)

        # Draw the pose annotation on the image.
        frame.flags.writeable = True
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.mp_drawing.draw_landmarks(
            frame,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
            # landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        )
        # Flip the image horizontally for a selfie-view display.
        cv2.imshow('MediaPipe Pose', cv2.flip(frame, 1))
        cv2.waitKey(10)
        
        # calculate angle      
        if results.pose_landmarks:
            h2 = 0
            h1 = self.camera_width
            for landmark in results.pose_landmarks.landmark:  
                if (landmark.x < h1):
                    h1 = landmark.x
                if (landmark.x > h2):
                    h2 = landmark.x
                
            [angle_h1, angle_h2] = self.pose_update(h1*self.camera_width, h2*self.camera_width)
            print("Here is data for each - h1: {}, h2: {} angle {} {}".format(h1,h2, angle_h1, angle_h2))
            
            # res = HumanAngle()
            # res.min_angle = angle_h1/180*np.pi + self.angle_offset - self.angle_boundary
            # res.max_angle = angle_h2/180*np.pi + self.angle_offset + self.angle_boundary
            # self.pub.publish(res)

    def pose_update(self, h1, h2):
        # confirm whether only v1 and v2 are needed
        list_of_angles = []
        angle_h1 = float(h1-self.camera_width/2)/(self.camera_width/2)*(self.HFOV)
        angle_h2 = float(h2-self.camera_width/2)/(self.camera_width/2)*(self.HFOV)
        list_of_angles = [angle_h1,angle_h2]
        
        print(list_of_angles)
        return list_of_angles

if __name__ == "__main__":
    human_detection()
