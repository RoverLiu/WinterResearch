#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
# from camera_msg.msg import JointState
import numpy as np
import rospy
import mediapipe as mp
from camera_msg.msg import JointState


class human_pose_estimation:
    
    def __init__(self) -> None:
        # set up the camera
        '''  Set up  '''
        self.camera_width = 640
        self.camera_height = 480
        
        self.pipeline = rs.pipeline()    #  Define the process pipeline, Create a pipe 
        self.config = rs.config()    #  Define configuration config
        self.config.enable_stream(rs.stream.depth, self.camera_width, self.camera_height, rs.format.z16, 15)      #  To configure depth flow 
        self.config.enable_stream(rs.stream.color, self.camera_width, self.camera_height, rs.format.bgr8, 15)     #  To configure color flow 

        # config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 90)
        # config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

        # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        self.pipe_profile = self.pipeline.start(self.config)       # streaming The flow begins 

        #  Create aligned objects with color Flow alignment 
        self.align_to = rs.stream.color      # align_to  Is the stream type for which the depth frame is scheduled to be aligned 
        self.align = rs.align(self.align_to)      # rs.align  Align the depth frame with other frames 
        
        # mediapipe init
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        # ros
        rospy.init_node('human_pose', anonymous=True)
        # rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.output_topic = "human_pose"
        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)

 

    '''  Get the alignment image frame and camera parameters  '''
    def get_aligned_images(self):
        
        frames = self.pipeline.wait_for_frames()     #  Wait for image frame , Get the frameset of color and depth  
        aligned_frames = self.align.process(frames)      #  Get alignment frame , Align the depth box with the color box  

        self.aligned_depth_frame = aligned_frames.get_depth_frame()      #  Gets the in the alignment frame depth frame  
        self.aligned_color_frame = aligned_frames.get_color_frame()      #  Gets the in the alignment frame color frame 

        ####  Get camera parameters  ####
        self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics     #  Get the depth parameter （ Pixel coordinate system to camera coordinate system will use ）
        self.color_intrin = self.aligned_color_frame.profile.as_video_stream_profile().intrinsics     #  Get camera internal parameters 


        ####  take images To numpy arrays #### 
        self.img_color = np.asanyarray(self.aligned_color_frame.get_data())       # RGB chart  
        self.img_depth = np.asanyarray(self.aligned_depth_frame.get_data())       #  Depth map （ Default 16 position ）

        return self.color_intrin, self.depth_intrin, self.img_color, self.img_depth, self.aligned_depth_frame


    '''  Obtain the 3D coordinates of random points  '''
    def get_3d_camera_coordinate(self, depth_pixel):
        x = depth_pixel[0]
        y = depth_pixel[1]
        self.dis = self.aligned_depth_frame.get_distance(x, y)        #  Get the depth corresponding to the pixel 
        # print ('depth: ',dis) #  The unit of depth is m
        self.camera_coordinate = rs.rs2_deproject_pixel_to_point(self.depth_intrin, depth_pixel, self.dis)
        # print ('camera_coordinate: ',camera_coordinate)
        return self.camera_coordinate
    
    def landmark2coordinate(self, landmark):
        ''' convert landmark to 3D coordinates and show it in the image (img_color)'''
        # calculate
        depth_pixel = []
        depth_pixel.append(round(landmark.x * self.camera_width))
        depth_pixel.append(round(landmark.y * self.camera_height))
        
        if (depth_pixel[0] >= self.camera_width or depth_pixel[1] >= self.camera_height 
            or depth_pixel[0] < 0 or depth_pixel[1] < 0):
            # fail
            return [0,0,-1]
        
        self.get_3d_camera_coordinate(depth_pixel)

        # draw
        self.draw(depth_pixel, self.camera_coordinate)
        
        return self.camera_coordinate

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
        # calculate pose
        
        output = JointState()
        output.left.shoulder = self.landmark2coordinate(landmark=landmarks.landmark[11])
        # output.left.shoulder = [landmarks.landmark[11].x, landmarks.landmark[11].y, landmarks.landmark[11].z, landmarks.landmark[11].visibility]
        output.left.elbow = self.landmark2coordinate(landmark=landmarks.landmark[13])
        # output.left.elbow = [landmarks.landmark[13].x, landmarks.landmark[13].y, landmarks.landmark[13].z, landmarks.landmark[13].visibility]
        output.left.wrist = self.landmark2coordinate(landmark=landmarks.landmark[15])
        # output.left.wrist = [landmarks.landmark[15].x, landmarks.landmark[15].y, landmarks.landmark[15].z, landmarks.landmark[15].visibility]
        output.left.hand_pinky = self.landmark2coordinate(landmark=landmarks.landmark[17])
        # output.left.hand_pinky = [landmarks.landmark[17].x, landmarks.landmark[17].y, landmarks.landmark[17].z, landmarks.landmark[17].visibility]
        output.left.hand_index = self.landmark2coordinate(landmark=landmarks.landmark[19])
        # output.left.hand_index = [landmarks.landmark[19].x, landmarks.landmark[19].y, landmarks.landmark[19].z, landmarks.landmark[19].visibility]
        output.left.hand_thumb = self.landmark2coordinate(landmark=landmarks.landmark[21])
        # output.left.hand_thumb = [landmarks.landmark[21].x, landmarks.landmark[21].y, landmarks.landmark[21].z, landmarks.landmark[21].visibility]
        output.left.hip = self.landmark2coordinate(landmark=landmarks.landmark[23])
        # output.left.hip = [landmarks.landmark[23].x, landmarks.landmark[23].y, landmarks.landmark[23].z, landmarks.landmark[23].visibility]
        
        
        output.right.shoulder = self.landmark2coordinate(landmark=landmarks.landmark[12])
        # output.right.shoulder = [landmarks.landmark[12].x, landmarks.landmark[12].y, landmarks.landmark[12].z, landmarks.landmark[12].visibility]
        output.right.elbow = self.landmark2coordinate(landmark=landmarks.landmark[14])
        # output.right.elbow = [landmarks.landmark[14].x, landmarks.landmark[14].y, landmarks.landmark[14].z, landmarks.landmark[14].visibility]
        output.right.wrist = self.landmark2coordinate(landmark=landmarks.landmark[16])
        # output.right.wrist = [landmarks.landmark[16].x, landmarks.landmark[16].y, landmarks.landmark[16].z, landmarks.landmark[16].visibility]
        output.right.hand_pinky = self.landmark2coordinate(landmark=landmarks.landmark[18])
        # output.right.hand_pinky = [landmarks.landmark[18].x, landmarks.landmark[18].y, landmarks.landmark[18].z, landmarks.landmark[18].visibility]
        output.right.hand_index = self.landmark2coordinate(landmark=landmarks.landmark[20])
        # output.right.hand_index = [landmarks.landmark[20].x, landmarks.landmark[20].y, landmarks.landmark[20].z, landmarks.landmark[20].visibility]
        output.right.hand_thumb = self.landmark2coordinate(landmark=landmarks.landmark[22])
        # output.right.hand_thumb = [landmarks.landmark[22].x, landmarks.landmark[22].y, landmarks.landmark[22].z, landmarks.landmark[22].visibility]
        output.right.hip = self.landmark2coordinate(landmark=landmarks.landmark[24])
        # output.right.hip = [landmarks.landmark[24].x, landmarks.landmark[24].y, landmarks.landmark[24].z, landmarks.landmark[24].visibility]
        
        # update for ros
        self.pub.publish(output)
        
    

    def draw(self, pixel_coor, coor_3d):
        '''  Display images and annotations  '''
        ####  Mark points and their coordinates in the diagram  ####
        cv2.circle(self.img_color, pixel_coor, 8, [255,0,255], thickness=-1)
        cv2.putText(self.img_color,"x: {:.1}, y: {:.1}, z: {:.1}".format(coor_3d[0], coor_3d[1], coor_3d[2]), (pixel_coor[0], pixel_coor[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,[0,0,255])

    def run(self):
        ''' run the whole detection'''
        while True:
            # update image
            self.get_aligned_images() 

            # detect the poses
            self.detect_pose(self.img_color)
            
            # show the status        
            cv2.imshow('RealSence',self.img_color)
            key = cv2.waitKey(10)
            
            # stop
            if rospy.is_shutdown():
                print('shutdown')
                break
        
        
if __name__=="__main__":
    my_estimator = human_pose_estimation()
        
    my_estimator.run()
