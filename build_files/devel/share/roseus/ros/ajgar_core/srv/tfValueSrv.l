;; Auto-generated. Do not edit!


(when (boundp 'ajgar_core::tfValueSrv)
  (if (not (find-package "AJGAR_CORE"))
    (make-package "AJGAR_CORE"))
  (shadow 'tfValueSrv (find-package "AJGAR_CORE")))
(unless (find-package "AJGAR_CORE::TFVALUESRV")
  (make-package "AJGAR_CORE::TFVALUESRV"))
(unless (find-package "AJGAR_CORE::TFVALUESRVREQUEST")
  (make-package "AJGAR_CORE::TFVALUESRVREQUEST"))
(unless (find-package "AJGAR_CORE::TFVALUESRVRESPONSE")
  (make-package "AJGAR_CORE::TFVALUESRVRESPONSE"))

(in-package "ROS")





(defclass ajgar_core::tfValueSrvRequest
  :super ros::object
  :slots (_tfArray ))

(defmethod ajgar_core::tfValueSrvRequest
  (:init
   (&key
    ((:tfArray __tfArray) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _tfArray __tfArray)
   self)
  (:tfArray
   (&optional __tfArray)
   (if __tfArray (setq _tfArray __tfArray)) _tfArray)
  (:serialization-length
   ()
   (+
    ;; float32[] _tfArray
    (* 4    (length _tfArray)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _tfArray
     (write-long (length _tfArray) s)
     (dotimes (i (length _tfArray))
       (sys::poke (elt _tfArray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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

(defclass ajgar_core::tfValueSrvResponse
  :super ros::object
  :slots (_reachedGoal ))

(defmethod ajgar_core::tfValueSrvResponse
  (:init
   (&key
    ((:reachedGoal __reachedGoal) nil)
    )
   (send-super :init)
   (setq _reachedGoal __reachedGoal)
   self)
  (:reachedGoal
   (&optional (__reachedGoal :null))
   (if (not (eq __reachedGoal :null)) (setq _reachedGoal __reachedGoal)) _reachedGoal)
  (:serialization-length
   ()
   (+
    ;; bool _reachedGoal
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _reachedGoal
       (if _reachedGoal (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _reachedGoal
     (setq _reachedGoal (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ajgar_core::tfValueSrv
  :super ros::object
  :slots ())

(setf (get ajgar_core::tfValueSrv :md5sum-) "e2d3842affcc45c41842273c7f827da7")
(setf (get ajgar_core::tfValueSrv :datatype-) "ajgar_core/tfValueSrv")
(setf (get ajgar_core::tfValueSrv :request) ajgar_core::tfValueSrvRequest)
(setf (get ajgar_core::tfValueSrv :response) ajgar_core::tfValueSrvResponse)

(defmethod ajgar_core::tfValueSrvRequest
  (:response () (instance ajgar_core::tfValueSrvResponse :init)))

(setf (get ajgar_core::tfValueSrvRequest :md5sum-) "e2d3842affcc45c41842273c7f827da7")
(setf (get ajgar_core::tfValueSrvRequest :datatype-) "ajgar_core/tfValueSrvRequest")
(setf (get ajgar_core::tfValueSrvRequest :definition-)
      "float32[] tfArray 
---
bool reachedGoal
")

(setf (get ajgar_core::tfValueSrvResponse :md5sum-) "e2d3842affcc45c41842273c7f827da7")
(setf (get ajgar_core::tfValueSrvResponse :datatype-) "ajgar_core/tfValueSrvResponse")
(setf (get ajgar_core::tfValueSrvResponse :definition-)
      "float32[] tfArray 
---
bool reachedGoal
")



(provide :ajgar_core/tfValueSrv "e2d3842affcc45c41842273c7f827da7")


