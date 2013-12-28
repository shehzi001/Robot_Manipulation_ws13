; Auto-generated. Do not edit!


(cl:in-package bmp085-srv)


;//! \htmlinclude weather_forcast-request.msg.html

(cl:defclass <weather_forcast-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass weather_forcast-request (<weather_forcast-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <weather_forcast-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'weather_forcast-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmp085-srv:<weather_forcast-request> is deprecated: use bmp085-srv:weather_forcast-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <weather_forcast-request>) ostream)
  "Serializes a message object of type '<weather_forcast-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <weather_forcast-request>) istream)
  "Deserializes a message object of type '<weather_forcast-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<weather_forcast-request>)))
  "Returns string type for a service object of type '<weather_forcast-request>"
  "bmp085/weather_forcastRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'weather_forcast-request)))
  "Returns string type for a service object of type 'weather_forcast-request"
  "bmp085/weather_forcastRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<weather_forcast-request>)))
  "Returns md5sum for a message object of type '<weather_forcast-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'weather_forcast-request)))
  "Returns md5sum for a message object of type 'weather_forcast-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<weather_forcast-request>)))
  "Returns full string definition for message of type '<weather_forcast-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'weather_forcast-request)))
  "Returns full string definition for message of type 'weather_forcast-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <weather_forcast-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <weather_forcast-request>))
  "Converts a ROS message object to a list"
  (cl:list 'weather_forcast-request
))
;//! \htmlinclude weather_forcast-response.msg.html

(cl:defclass <weather_forcast-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass weather_forcast-response (<weather_forcast-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <weather_forcast-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'weather_forcast-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmp085-srv:<weather_forcast-response> is deprecated: use bmp085-srv:weather_forcast-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <weather_forcast-response>) ostream)
  "Serializes a message object of type '<weather_forcast-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <weather_forcast-response>) istream)
  "Deserializes a message object of type '<weather_forcast-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<weather_forcast-response>)))
  "Returns string type for a service object of type '<weather_forcast-response>"
  "bmp085/weather_forcastResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'weather_forcast-response)))
  "Returns string type for a service object of type 'weather_forcast-response"
  "bmp085/weather_forcastResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<weather_forcast-response>)))
  "Returns md5sum for a message object of type '<weather_forcast-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'weather_forcast-response)))
  "Returns md5sum for a message object of type 'weather_forcast-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<weather_forcast-response>)))
  "Returns full string definition for message of type '<weather_forcast-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'weather_forcast-response)))
  "Returns full string definition for message of type 'weather_forcast-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <weather_forcast-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <weather_forcast-response>))
  "Converts a ROS message object to a list"
  (cl:list 'weather_forcast-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'weather_forcast)))
  'weather_forcast-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'weather_forcast)))
  'weather_forcast-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'weather_forcast)))
  "Returns string type for a service object of type '<weather_forcast>"
  "bmp085/weather_forcast")