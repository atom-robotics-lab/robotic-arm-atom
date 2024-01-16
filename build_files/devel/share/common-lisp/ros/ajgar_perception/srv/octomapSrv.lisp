; Auto-generated. Do not edit!


(cl:in-package ajgar_perception-srv)


;//! \htmlinclude octomapSrv-request.msg.html

(cl:defclass <octomapSrv-request> (roslisp-msg-protocol:ros-message)
  ((kinectInputPt
    :reader kinectInputPt
    :initarg :kinectInputPt
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (maskInputPt
    :reader maskInputPt
    :initarg :maskInputPt
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass octomapSrv-request (<octomapSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <octomapSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'octomapSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ajgar_perception-srv:<octomapSrv-request> is deprecated: use ajgar_perception-srv:octomapSrv-request instead.")))

(cl:ensure-generic-function 'kinectInputPt-val :lambda-list '(m))
(cl:defmethod kinectInputPt-val ((m <octomapSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ajgar_perception-srv:kinectInputPt-val is deprecated.  Use ajgar_perception-srv:kinectInputPt instead.")
  (kinectInputPt m))

(cl:ensure-generic-function 'maskInputPt-val :lambda-list '(m))
(cl:defmethod maskInputPt-val ((m <octomapSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ajgar_perception-srv:maskInputPt-val is deprecated.  Use ajgar_perception-srv:maskInputPt instead.")
  (maskInputPt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <octomapSrv-request>) ostream)
  "Serializes a message object of type '<octomapSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kinectInputPt) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'maskInputPt) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <octomapSrv-request>) istream)
  "Deserializes a message object of type '<octomapSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kinectInputPt) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'maskInputPt) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<octomapSrv-request>)))
  "Returns string type for a service object of type '<octomapSrv-request>"
  "ajgar_perception/octomapSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomapSrv-request)))
  "Returns string type for a service object of type 'octomapSrv-request"
  "ajgar_perception/octomapSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<octomapSrv-request>)))
  "Returns md5sum for a message object of type '<octomapSrv-request>"
  "616b90dff5e1a5812063533505c3044e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'octomapSrv-request)))
  "Returns md5sum for a message object of type 'octomapSrv-request"
  "616b90dff5e1a5812063533505c3044e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<octomapSrv-request>)))
  "Returns full string definition for message of type '<octomapSrv-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 kinectInputPt~%sensor_msgs/PointCloud2 maskInputPt ~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'octomapSrv-request)))
  "Returns full string definition for message of type 'octomapSrv-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 kinectInputPt~%sensor_msgs/PointCloud2 maskInputPt ~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <octomapSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kinectInputPt))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'maskInputPt))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <octomapSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'octomapSrv-request
    (cl:cons ':kinectInputPt (kinectInputPt msg))
    (cl:cons ':maskInputPt (maskInputPt msg))
))
;//! \htmlinclude octomapSrv-response.msg.html

(cl:defclass <octomapSrv-response> (roslisp-msg-protocol:ros-message)
  ((outputPt
    :reader outputPt
    :initarg :outputPt
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass octomapSrv-response (<octomapSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <octomapSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'octomapSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ajgar_perception-srv:<octomapSrv-response> is deprecated: use ajgar_perception-srv:octomapSrv-response instead.")))

(cl:ensure-generic-function 'outputPt-val :lambda-list '(m))
(cl:defmethod outputPt-val ((m <octomapSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ajgar_perception-srv:outputPt-val is deprecated.  Use ajgar_perception-srv:outputPt instead.")
  (outputPt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <octomapSrv-response>) ostream)
  "Serializes a message object of type '<octomapSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'outputPt) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <octomapSrv-response>) istream)
  "Deserializes a message object of type '<octomapSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'outputPt) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<octomapSrv-response>)))
  "Returns string type for a service object of type '<octomapSrv-response>"
  "ajgar_perception/octomapSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomapSrv-response)))
  "Returns string type for a service object of type 'octomapSrv-response"
  "ajgar_perception/octomapSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<octomapSrv-response>)))
  "Returns md5sum for a message object of type '<octomapSrv-response>"
  "616b90dff5e1a5812063533505c3044e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'octomapSrv-response)))
  "Returns md5sum for a message object of type 'octomapSrv-response"
  "616b90dff5e1a5812063533505c3044e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<octomapSrv-response>)))
  "Returns full string definition for message of type '<octomapSrv-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 outputPt ~% ~%~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'octomapSrv-response)))
  "Returns full string definition for message of type 'octomapSrv-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 outputPt ~% ~%~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <octomapSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'outputPt))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <octomapSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'octomapSrv-response
    (cl:cons ':outputPt (outputPt msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'octomapSrv)))
  'octomapSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'octomapSrv)))
  'octomapSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomapSrv)))
  "Returns string type for a service object of type '<octomapSrv>"
  "ajgar_perception/octomapSrv")