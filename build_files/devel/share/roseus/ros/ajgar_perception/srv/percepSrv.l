;; Auto-generated. Do not edit!


(when (boundp 'ajgar_perception::percepSrv)
  (if (not (find-package "AJGAR_PERCEPTION"))
    (make-package "AJGAR_PERCEPTION"))
  (shadow 'percepSrv (find-package "AJGAR_PERCEPTION")))
(unless (find-package "AJGAR_PERCEPTION::PERCEPSRV")
  (make-package "AJGAR_PERCEPTION::PERCEPSRV"))
(unless (find-package "AJGAR_PERCEPTION::PERCEPSRVREQUEST")
  (make-package "AJGAR_PERCEPTION::PERCEPSRVREQUEST"))
(unless (find-package "AJGAR_PERCEPTION::PERCEPSRVRESPONSE")
  (make-package "AJGAR_PERCEPTION::PERCEPSRVRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass ajgar_perception::percepSrvRequest
  :super ros::object
  :slots (_flag ))

(defmethod ajgar_perception::percepSrvRequest
  (:init
   (&key
    ((:flag __flag) 0)
    )
   (send-super :init)
   (setq _flag (round __flag))
   self)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; int32 _flag
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _flag
       (write-long _flag s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _flag
     (setq _flag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ajgar_perception::percepSrvResponse
  :super ros::object
  :slots (_maskValue _tfArray ))

(defmethod ajgar_perception::percepSrvResponse
  (:init
   (&key
    ((:maskValue __maskValue) (instance sensor_msgs::PointCloud2 :init))
    ((:tfArray __tfArray) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _maskValue __maskValue)
   (setq _tfArray __tfArray)
   self)
  (:maskValue
   (&rest __maskValue)
   (if (keywordp (car __maskValue))
       (send* _maskValue __maskValue)
     (progn
       (if __maskValue (setq _maskValue (car __maskValue)))
       _maskValue)))
  (:tfArray
   (&optional __tfArray)
   (if __tfArray (setq _tfArray __tfArray)) _tfArray)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _maskValue
    (send _maskValue :serialization-length)
    ;; float32[] _tfArray
    (* 4    (length _tfArray)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _maskValue
       (send _maskValue :serialize s)
     ;; float32[] _tfArray
     (write-long (length _tfArray) s)
     (dotimes (i (length _tfArray))
       (sys::poke (elt _tfArray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _maskValue
     (send _maskValue :deserialize buf ptr-) (incf ptr- (send _maskValue :serialization-length))
   ;; float32[] _tfArray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tfArray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tfArray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass ajgar_perception::percepSrv
  :super ros::object
  :slots ())

(setf (get ajgar_perception::percepSrv :md5sum-) "a5c0d5df4352df7c34178b1c281fe9a4")
(setf (get ajgar_perception::percepSrv :datatype-) "ajgar_perception/percepSrv")
(setf (get ajgar_perception::percepSrv :request) ajgar_perception::percepSrvRequest)
(setf (get ajgar_perception::percepSrv :response) ajgar_perception::percepSrvResponse)

(defmethod ajgar_perception::percepSrvRequest
  (:response () (instance ajgar_perception::percepSrvResponse :init)))

(setf (get ajgar_perception::percepSrvRequest :md5sum-) "a5c0d5df4352df7c34178b1c281fe9a4")
(setf (get ajgar_perception::percepSrvRequest :datatype-) "ajgar_perception/percepSrvRequest")
(setf (get ajgar_perception::percepSrvRequest :definition-)
      "int32 flag
---
sensor_msgs/PointCloud2 maskValue 
float32[] tfArray

 



================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
")

(setf (get ajgar_perception::percepSrvResponse :md5sum-) "a5c0d5df4352df7c34178b1c281fe9a4")
(setf (get ajgar_perception::percepSrvResponse :datatype-) "ajgar_perception/percepSrvResponse")
(setf (get ajgar_perception::percepSrvResponse :definition-)
      "int32 flag
---
sensor_msgs/PointCloud2 maskValue 
float32[] tfArray

 



================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
")



(provide :ajgar_perception/percepSrv "a5c0d5df4352df7c34178b1c281fe9a4")


