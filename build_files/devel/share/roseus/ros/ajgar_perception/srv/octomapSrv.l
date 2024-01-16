;; Auto-generated. Do not edit!


(when (boundp 'ajgar_perception::octomapSrv)
  (if (not (find-package "AJGAR_PERCEPTION"))
    (make-package "AJGAR_PERCEPTION"))
  (shadow 'octomapSrv (find-package "AJGAR_PERCEPTION")))
(unless (find-package "AJGAR_PERCEPTION::OCTOMAPSRV")
  (make-package "AJGAR_PERCEPTION::OCTOMAPSRV"))
(unless (find-package "AJGAR_PERCEPTION::OCTOMAPSRVREQUEST")
  (make-package "AJGAR_PERCEPTION::OCTOMAPSRVREQUEST"))
(unless (find-package "AJGAR_PERCEPTION::OCTOMAPSRVRESPONSE")
  (make-package "AJGAR_PERCEPTION::OCTOMAPSRVRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass ajgar_perception::octomapSrvRequest
  :super ros::object
  :slots (_kinectInputPt _maskInputPt ))

(defmethod ajgar_perception::octomapSrvRequest
  (:init
   (&key
    ((:kinectInputPt __kinectInputPt) (instance sensor_msgs::PointCloud2 :init))
    ((:maskInputPt __maskInputPt) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _kinectInputPt __kinectInputPt)
   (setq _maskInputPt __maskInputPt)
   self)
  (:kinectInputPt
   (&rest __kinectInputPt)
   (if (keywordp (car __kinectInputPt))
       (send* _kinectInputPt __kinectInputPt)
     (progn
       (if __kinectInputPt (setq _kinectInputPt (car __kinectInputPt)))
       _kinectInputPt)))
  (:maskInputPt
   (&rest __maskInputPt)
   (if (keywordp (car __maskInputPt))
       (send* _maskInputPt __maskInputPt)
     (progn
       (if __maskInputPt (setq _maskInputPt (car __maskInputPt)))
       _maskInputPt)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _kinectInputPt
    (send _kinectInputPt :serialization-length)
    ;; sensor_msgs/PointCloud2 _maskInputPt
    (send _maskInputPt :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _kinectInputPt
       (send _kinectInputPt :serialize s)
     ;; sensor_msgs/PointCloud2 _maskInputPt
       (send _maskInputPt :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _kinectInputPt
     (send _kinectInputPt :deserialize buf ptr-) (incf ptr- (send _kinectInputPt :serialization-length))
   ;; sensor_msgs/PointCloud2 _maskInputPt
     (send _maskInputPt :deserialize buf ptr-) (incf ptr- (send _maskInputPt :serialization-length))
   ;;
   self)
  )

(defclass ajgar_perception::octomapSrvResponse
  :super ros::object
  :slots (_outputPt ))

(defmethod ajgar_perception::octomapSrvResponse
  (:init
   (&key
    ((:outputPt __outputPt) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _outputPt __outputPt)
   self)
  (:outputPt
   (&rest __outputPt)
   (if (keywordp (car __outputPt))
       (send* _outputPt __outputPt)
     (progn
       (if __outputPt (setq _outputPt (car __outputPt)))
       _outputPt)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _outputPt
    (send _outputPt :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _outputPt
       (send _outputPt :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _outputPt
     (send _outputPt :deserialize buf ptr-) (incf ptr- (send _outputPt :serialization-length))
   ;;
   self)
  )

(defclass ajgar_perception::octomapSrv
  :super ros::object
  :slots ())

(setf (get ajgar_perception::octomapSrv :md5sum-) "616b90dff5e1a5812063533505c3044e")
(setf (get ajgar_perception::octomapSrv :datatype-) "ajgar_perception/octomapSrv")
(setf (get ajgar_perception::octomapSrv :request) ajgar_perception::octomapSrvRequest)
(setf (get ajgar_perception::octomapSrv :response) ajgar_perception::octomapSrvResponse)

(defmethod ajgar_perception::octomapSrvRequest
  (:response () (instance ajgar_perception::octomapSrvResponse :init)))

(setf (get ajgar_perception::octomapSrvRequest :md5sum-) "616b90dff5e1a5812063533505c3044e")
(setf (get ajgar_perception::octomapSrvRequest :datatype-) "ajgar_perception/octomapSrvRequest")
(setf (get ajgar_perception::octomapSrvRequest :definition-)
      "sensor_msgs/PointCloud2 kinectInputPt
sensor_msgs/PointCloud2 maskInputPt 

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
---
sensor_msgs/PointCloud2 outputPt 
 



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

(setf (get ajgar_perception::octomapSrvResponse :md5sum-) "616b90dff5e1a5812063533505c3044e")
(setf (get ajgar_perception::octomapSrvResponse :datatype-) "ajgar_perception/octomapSrvResponse")
(setf (get ajgar_perception::octomapSrvResponse :definition-)
      "sensor_msgs/PointCloud2 kinectInputPt
sensor_msgs/PointCloud2 maskInputPt 

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
---
sensor_msgs/PointCloud2 outputPt 
 



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



(provide :ajgar_perception/octomapSrv "616b90dff5e1a5812063533505c3044e")


