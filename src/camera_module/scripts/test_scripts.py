#! /usr/bin/env python3

# from imutils.object_detection import non_max_suppression
from numpy import size
from torch import true_divide
from camera_msg.msg import JointState
import numpy as np
import rospy
import copy
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
        self.HFOV = 53.50
        self.VFOV = 41.6
        self.angle_offset = 0
        self.angle_boundary = 0
        self.count=0
        
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

    # def pointcloud_callback(self, ros_data):
    #     # dat = pc2.read_points(ros_data, field_names = ("x", "y", "z", "rgb"), skip_nans=False, uvs=[(640,0)])
    #     # print("data size: {}, height: {}, width: {}, raw_step: {}".format(sum(1 for _ in dat), ros_data.height, ros_data.width, ros_data.row_step))
    #     nose = (self.read_depth(round(self.left_arm_pose.hip.x*self.camera_width), round(self.left_arm_pose.hip.y*self.camera_height), ros_data))
    #     # print("nose position: x {}, y {}, z {}".format(nose[0], nose[1], nose[2]))
    #     for p in nose:
    #         print(p)
        
    #     rospy.sleep(1.0)
    #     # self.read_depth(self.no)
    #     # for p in dat:
    #         # print(" x : {}  y: {} z: {}".format(p[0],p[1],p[2]))
    #     # print("height: {}, width: {}, size of data: {}".format( ros_data.height, ros_data.width, len(ros_data.data)))
    #     # pass
        
    
    # def read_depth(self, width, height, data) :
    #     # rospy.loginfo("Data width: {}, data height: {}".format(data.width, data.height))
    #     # read function
    #     if (self.camera_width*height+width >= data.width) :
    #         return -1
    #     int_data = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[(self.camera_width*height+width, 0)])
    #     # int_data = next(data_out)
    #     # rospy.loginfo("int_data " + str(int_data))
    #     return int_data

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
            # landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        )
        # Flip the image horizontally for a selfie-view display.
        cv2.imshow('MediaPipe Pose', cv2.flip(frame, 1))
        cv2.waitKey(10)
        
        # calculate angle      
        if results.pose_landmarks:
            # update 2D pose
            self.left_arm_pose.hip = results.pose_landmarks.landmark[0]
            
            
        else:
            self.is_detected = False

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
