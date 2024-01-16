; Auto-generated. Do not edit!


(cl:in-package ajgar_core-srv)


;//! \htmlinclude tfValueSrv-request.msg.html

(cl:defclass <tfValueSrv-request> (roslisp-msg-protocol:ros-message)
  ((tfArray
    :reader tfArray
    :initarg :tfArray
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass tfValueSrv-request (<tfValueSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tfValueSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tfValueSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ajgar_core-srv:<tfValueSrv-request> is deprecated: use ajgar_core-srv:tfValueSrv-request instead.")))

(cl:ensure-generic-function 'tfArray-val :lambda-list '(m))
(cl:defmethod tfArray-val ((m <tfValueSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ajgar_core-srv:tfArray-val is deprecated.  Use ajgar_core-srv:tfArray instead.")
  (tfArray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tfValueSrv-request>) ostream)
  "Serializes a message object of type '<tfValueSrv-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tfValueSrv-request>) istream)
  "Deserializes a message object of type '<tfValueSrv-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tfValueSrv-request>)))
  "Returns string type for a service object of type '<tfValueSrv-request>"
  "ajgar_core/tfValueSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tfValueSrv-request)))
  "Returns string type for a service object of type 'tfValueSrv-request"
  "ajgar_core/tfValueSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tfValueSrv-request>)))
  "Returns md5sum for a message object of type '<tfValueSrv-request>"
  "e2d3842affcc45c41842273c7f827da7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tfValueSrv-request)))
  "Returns md5sum for a message object of type 'tfValueSrv-request"
  "e2d3842affcc45c41842273c7f827da7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tfValueSrv-request>)))
  "Returns full string definition for message of type '<tfValueSrv-request>"
  (cl:format cl:nil "float32[] tfArray ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tfValueSrv-request)))
  "Returns full string definition for message of type 'tfValueSrv-request"
  (cl:format cl:nil "float32[] tfArray ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tfValueSrv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tfArray) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tfValueSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'tfValueSrv-request
    (cl:cons ':tfArray (tfArray msg))
))
;//! \htmlinclude tfValueSrv-response.msg.html

(cl:defclass <tfValueSrv-response> (roslisp-msg-protocol:ros-message)
  ((reachedGoal
    :reader reachedGoal
    :initarg :reachedGoal
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass tfValueSrv-response (<tfValueSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tfValueSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tfValueSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ajgar_core-srv:<tfValueSrv-response> is deprecated: use ajgar_core-srv:tfValueSrv-response instead.")))

(cl:ensure-generic-function 'reachedGoal-val :lambda-list '(m))
(cl:defmethod reachedGoal-val ((m <tfValueSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ajgar_core-srv:reachedGoal-val is deprecated.  Use ajgar_core-srv:reachedGoal instead.")
  (reachedGoal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tfValueSrv-response>) ostream)
  "Serializes a message object of type '<tfValueSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reachedGoal) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tfValueSrv-response>) istream)
  "Deserializes a message object of type '<tfValueSrv-response>"
    (cl:setf (cl:slot-value msg 'reachedGoal) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tfValueSrv-response>)))
  "Returns string type for a service object of type '<tfValueSrv-response>"
  "ajgar_core/tfValueSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tfValueSrv-response)))
  "Returns string type for a service object of type 'tfValueSrv-response"
  "ajgar_core/tfValueSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tfValueSrv-response>)))
  "Returns md5sum for a message object of type '<tfValueSrv-response>"
  "e2d3842affcc45c41842273c7f827da7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tfValueSrv-response)))
  "Returns md5sum for a message object of type 'tfValueSrv-response"
  "e2d3842affcc45c41842273c7f827da7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tfValueSrv-response>)))
  "Returns full string definition for message of type '<tfValueSrv-response>"
  (cl:format cl:nil "bool reachedGoal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tfValueSrv-response)))
  "Returns full string definition for message of type 'tfValueSrv-response"
  (cl:format cl:nil "bool reachedGoal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tfValueSrv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tfValueSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'tfValueSrv-response
    (cl:cons ':reachedGoal (reachedGoal msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'tfValueSrv)))
  'tfValueSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'tfValueSrv)))
  'tfValueSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tfValueSrv)))
  "Returns string type for a service object of type '<tfValueSrv>"
  "ajgar_core/tfValueSrv")