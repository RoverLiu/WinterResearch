;; Auto-generated. Do not edit!


(when (boundp 'camera_msg::JointState)
  (if (not (find-package "CAMERA_MSG"))
    (make-package "CAMERA_MSG"))
  (shadow 'JointState (find-package "CAMERA_MSG")))
(unless (find-package "CAMERA_MSG::JOINTSTATE")
  (make-package "CAMERA_MSG::JOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude JointState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass camera_msg::JointState
  :super ros::object
  :slots (_header _left _right ))

(defmethod camera_msg::JointState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left __left) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:right __right) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _left __left)
   (setq _right __right)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _left
    (* 4    (length _left)) 4
    ;; float32[] _right
    (* 4    (length _right)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _left
     (write-long (length _left) s)
     (dotimes (i (length _left))
       (sys::poke (elt _left i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _right
     (write-long (length _right) s)
     (dotimes (i (length _right))
       (sys::poke (elt _right i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _left
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _left i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _right
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _right i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get camera_msg::JointState :md5sum-) "73808776ee8064d08af80cf01bcc1b74")
(setf (get camera_msg::JointState :datatype-) "camera_msg/JointState")
(setf (get camera_msg::JointState :definition-)
      "
########################################
# Messages
########################################
Header header
float32[] left
float32[] right
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :camera_msg/JointState "73808776ee8064d08af80cf01bcc1b74")


