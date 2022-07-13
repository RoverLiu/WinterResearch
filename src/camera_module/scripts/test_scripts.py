#! /usr/bin/env python3

# from imutils.object_detection import non_max_suppression
from numpy import size
from torch import true_divide
from camera_msg.msg import JointState
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from numpy import asarray
import mediapipe as mp
class arm_details:
    def __init__(self) -> None:
        self.hip = None
        self.shoulder = None
        self.elbow = None
        self.wrist = None
        self.hand = None

class human_detection:
    def __init__(self) -> None:
        # mediapipe init
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

        # camera details
        self.camera_width = 640
        self.camera_height = 480
        
        #topic subcribption 
        self.image_topic = '/camera/color/image_raw/compressed_throttled'
        self.depth_topic = '/camera/depth/image_rect_raw/compressed_throttled'
        self.pointcloud_topic = '/camera/depth/color/points'
        self.output_topic = "human_pose"
        
        # save arm and hand details
        self.is_detected = False
        # save a list of tuples
        self.left_arm_pose = arm_details()
        self.right_arm_pose = arm_details()
        
        rospy.init_node('human_detection', anonymous=True)
        rospy.Subscriber(self.image_topic, CompressedImage, self.cam_callback)
        # rospy.Subscriber(self.depth_topic, CompressedImage, self.cam_depth_callback)
        # rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)
        rospy.spin()

    # camera callback function (process RGB data and save)
    def cam_callback(self, ros_data):
        #image conversion 
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # print("size of rgb: {}".format(size(np_arr)))
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # show the RGB image
        cv2.imshow('cv_img', image_np)
        k = cv2.waitKey(10)
        # detect and update arm poses
        angle = self.detect_pose(image_np)   
        
    # def cam_depth_callback(self, ros_data):
    #     pass
    #     #### direct conversion to CV2 ####
    #     np_arr = np.fromstring(ros_data.data, np.uint8)
    #     # print("pos middle: {}, size of data: {}".format( ros_data.data[0], size(np_arr)))
    #     image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #     cv2.imshow('cv_img', image_np)
    #     # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    #     # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
    
    
    # get the pose of human if there is
    def detect_pose(self, frame):
        
        # get pose and show
        frame.flags.writeable = False
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        results = self.pose.process(frame)

        # Draw the pose annotation on the image.
        frame.flags.writeable = True
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.mp_drawing.draw_landmarks(
            frame,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
        )
        # Flip the image horizontally for a selfie-view display.
        cv2.imshow('MediaPipe Pose', cv2.flip(frame, 1))
        cv2.waitKey(10)
        
        # calculate angle      
        if results.pose_landmarks:
            # update 2D pose
            self.pose_update(results.pose_landmarks)            
            
        else:
            self.is_detected = False

    def pose_update(self, landmarks):
        # confirm whether only v1 and v2 are needed
        
        output = JointState()
        
        output.left.shoulder = [landmarks.landmark[11].x, landmarks.landmark[11].y, landmarks.landmark[11].z, landmarks.landmark[11].visibility]
        output.left.elbow = [landmarks.landmark[13].x, landmarks.landmark[13].y, landmarks.landmark[13].z, landmarks.landmark[13].visibility]
        output.left.wrist = [landmarks.landmark[15].x, landmarks.landmark[15].y, landmarks.landmark[15].z, landmarks.landmark[15].visibility]
        output.left.hand_pinky = [landmarks.landmark[17].x, landmarks.landmark[17].y, landmarks.landmark[17].z, landmarks.landmark[17].visibility]
        output.left.hand_index = [landmarks.landmark[19].x, landmarks.landmark[19].y, landmarks.landmark[19].z, landmarks.landmark[19].visibility]
        output.left.hand_thumb = [landmarks.landmark[21].x, landmarks.landmark[21].y, landmarks.landmark[21].z, landmarks.landmark[21].visibility]
        output.left.hip = [landmarks.landmark[23].x, landmarks.landmark[23].y, landmarks.landmark[23].z, landmarks.landmark[23].visibility]
        
        output.right.shoulder = [landmarks.landmark[12].x, landmarks.landmark[12].y, landmarks.landmark[12].z, landmarks.landmark[12].visibility]
        output.right.elbow = [landmarks.landmark[14].x, landmarks.landmark[14].y, landmarks.landmark[14].z, landmarks.landmark[14].visibility]
        output.right.wrist = [landmarks.landmark[16].x, landmarks.landmark[16].y, landmarks.landmark[16].z, landmarks.landmark[16].visibility]
        output.right.hand_pinky = [landmarks.landmark[18].x, landmarks.landmark[18].y, landmarks.landmark[18].z, landmarks.landmark[18].visibility]
        output.right.hand_index = [landmarks.landmark[20].x, landmarks.landmark[20].y, landmarks.landmark[20].z, landmarks.landmark[20].visibility]
        output.right.hand_thumb = [landmarks.landmark[22].x, landmarks.landmark[22].y, landmarks.landmark[22].z, landmarks.landmark[22].visibility]
        output.right.hip = [landmarks.landmark[24].x, landmarks.landmark[24].y, landmarks.landmark[24].z, landmarks.landmark[24].visibility]
        
        self.pub.publish(output)
        
        
if __name__ == "__main__":
    human_detection()
