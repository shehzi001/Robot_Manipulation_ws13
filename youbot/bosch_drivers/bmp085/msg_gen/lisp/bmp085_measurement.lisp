; Auto-generated. Do not edit!


(cl:in-package bmp085-msg)


;//! \htmlinclude bmp085_measurement.msg.html

(cl:defclass <bmp085_measurement> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (is_connected
    :reader is_connected
    :initarg :is_connected
    :type cl:boolean
    :initform cl:nil)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (pressure
    :reader pressure
    :initarg :pressure
    :type cl:float
    :initform 0.0))
)

(cl:defclass bmp085_measurement (<bmp085_measurement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bmp085_measurement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bmp085_measurement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmp085-msg:<bmp085_measurement> is deprecated: use bmp085-msg:bmp085_measurement instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <bmp085_measurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmp085-msg:header-val is deprecated.  Use bmp085-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'is_connected-val :lambda-list '(m))
(cl:defmethod is_connected-val ((m <bmp085_measurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmp085-msg:is_connected-val is deprecated.  Use bmp085-msg:is_connected instead.")
  (is_connected m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <bmp085_measurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmp085-msg:temperature-val is deprecated.  Use bmp085-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'pressure-val :lambda-list '(m))
(cl:defmethod pressure-val ((m <bmp085_measurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmp085-msg:pressure-val is deprecated.  Use bmp085-msg:pressure instead.")
  (pressure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bmp085_measurement>) ostream)
  "Serializes a message object of type '<bmp085_measurement>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_connected) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pressure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bmp085_measurement>) istream)
  "Deserializes a message object of type '<bmp085_measurement>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'is_connected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pressure) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bmp085_measurement>)))
  "Returns string type for a message object of type '<bmp085_measurement>"
  "bmp085/bmp085_measurement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bmp085_measurement)))
  "Returns string type for a message object of type 'bmp085_measurement"
  "bmp085/bmp085_measurement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bmp085_measurement>)))
  "Returns md5sum for a message object of type '<bmp085_measurement>"
  "e46f06afb39aba894bcfac63fab0dcc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bmp085_measurement)))
  "Returns md5sum for a message object of type 'bmp085_measurement"
  "e46f06afb39aba894bcfac63fab0dcc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bmp085_measurement>)))
  "Returns full string definition for message of type '<bmp085_measurement>"
  (cl:format cl:nil "Header header~%bool is_connected # is the sensor connected?~%float32 temperature #[C]~%float32 pressure #[kPa]~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bmp085_measurement)))
  "Returns full string definition for message of type 'bmp085_measurement"
  (cl:format cl:nil "Header header~%bool is_connected # is the sensor connected?~%float32 temperature #[C]~%float32 pressure #[kPa]~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bmp085_measurement>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bmp085_measurement>))
  "Converts a ROS message object to a list"
  (cl:list 'bmp085_measurement
    (cl:cons ':header (header msg))
    (cl:cons ':is_connected (is_connected msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':pressure (pressure msg))
))
