; Auto-generated. Do not edit!


(cl:in-package ajgar_perception-srv)


;//! \htmlinclude percepSrv-request.msg.html

(cl:defclass <percepSrv-request> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:integer
    :initform 0))
)

(cl:defclass percepSrv-request (<percepSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <percepSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'percepSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ajgar_perception-srv:<percepSrv-request> is deprecated: use ajgar_perception-srv:percepSrv-request instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <percepSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ajgar_perception-srv:flag-val is deprecated.  Use ajgar_perception-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <percepSrv-request>) ostream)
  "Serializes a message object of type '<percepSrv-request>"
  (cl:let* ((signed (cl:slot-value msg 'flag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <percepSrv-request>) istream)
  "Deserializes a message object of type '<percepSrv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flag) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<percepSrv-request>)))
  "Returns string type for a service object of type '<percepSrv-request>"
  "ajgar_perception/percepSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'percepSrv-request)))
  "Returns string type for a service object of type 'percepSrv-request"
  "ajgar_perception/percepSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<percepSrv-request>)))
  "Returns md5sum for a message object of type '<percepSrv-request>"
  "a5c0d5df4352df7c34178b1c281fe9a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'percepSrv-request)))
  "Returns md5sum for a message object of type 'percepSrv-request"
  "a5c0d5df4352df7c34178b1c281fe9a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<percepSrv-request>)))
  "Returns full string definition for message of type '<percepSrv-request>"
  (cl:format cl:nil "int32 flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'percepSrv-request)))
  "Returns full string definition for message of type 'percepSrv-request"
  (cl:format cl:nil "int32 flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <percepSrv-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <percepSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'percepSrv-request
    (cl:cons ':flag (flag msg))
))
;//! \htmlinclude percepSrv-response.msg.html

(cl:defclass <percepSrv-response> (roslisp-msg-protocol:ros-message)
  ((maskValue
    :reader maskValue
    :initarg :maskValue
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (tfArray
    :reader tfArray
    :initarg :tfArray
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass percepSrv-response (<percepSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <percepSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'percepSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ajgar_perception-srv:<percepSrv-response> is deprecated: use ajgar_perception-srv:percepSrv-response instead.")))

(cl:ensure-generic-function 'maskValue-val :lambda-list '(m))
(cl:defmethod maskValue-val ((m <percepSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ajgar_perception-srv:maskValue-val is deprecated.  Use ajgar_perception-srv:maskValue instead.")
  (maskValue m))

(cl:ensure-generic-function 'tfArray-val :lambda-list '(m))
(cl:defmethod tfArray-val ((m <percepSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ajgar_perception-srv:tfArray-val is deprecated.  Use ajgar_perception-srv:tfArray instead.")
  (tfArray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <percepSrv-response>) ostream)
  "Serializes a message object of type '<percepSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'maskValue) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tfArray))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'tfArray))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <percepSrv-response>) istream)
  "Deserializes a message object of type '<percepSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'maskValue) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tfArray) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tfArray)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<percepSrv-response>)))
  "Returns string type for a service object of type '<percepSrv-response>"
  "ajgar_perception/percepSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'percepSrv-response)))
  "Returns string type for a service object of type 'percepSrv-response"
  "ajgar_perception/percepSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<percepSrv-response>)))
  "Returns md5sum for a message object of type '<percepSrv-response>"
  "a5c0d5df4352df7c34178b1c281fe9a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'percepSrv-response)))
  "Returns md5sum for a message object of type 'percepSrv-response"
  "a5c0d5df4352df7c34178b1c281fe9a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<percepSrv-response>)))
  "Returns full string definition for message of type '<percepSrv-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 maskValue ~%float32[] tfArray~%~% ~%~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'percepSrv-response)))
  "Returns full string definition for message of type 'percepSrv-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 maskValue ~%float32[] tfArray~%~% ~%~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <percepSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'maskValue))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tfArray) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <percepSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'percepSrv-response
    (cl:cons ':maskValue (maskValue msg))
    (cl:cons ':tfArray (tfArray msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'percepSrv)))
  'percepSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'percepSrv)))
  'percepSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'percepSrv)))
  "Returns string type for a service object of type '<percepSrv>"
  "ajgar_perception/percepSrv")