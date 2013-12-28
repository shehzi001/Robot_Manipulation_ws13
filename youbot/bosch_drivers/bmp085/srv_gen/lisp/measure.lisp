; Auto-generated. Do not edit!


(cl:in-package bmp085-srv)


;//! \htmlinclude measure-request.msg.html

(cl:defclass <measure-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass measure-request (<measure-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <measure-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'measure-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmp085-srv:<measure-request> is deprecated: use bmp085-srv:measure-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <measure-request>) ostream)
  "Serializes a message object of type '<measure-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <measure-request>) istream)
  "Deserializes a message object of type '<measure-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<measure-request>)))
  "Returns string type for a service object of type '<measure-request>"
  "bmp085/measureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'measure-request)))
  "Returns string type for a service object of type 'measure-request"
  "bmp085/measureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<measure-request>)))
  "Returns md5sum for a message object of type '<measure-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'measure-request)))
  "Returns md5sum for a message object of type 'measure-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<measure-request>)))
  "Returns full string definition for message of type '<measure-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'measure-request)))
  "Returns full string definition for message of type 'measure-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <measure-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <measure-request>))
  "Converts a ROS message object to a list"
  (cl:list 'measure-request
))
;//! \htmlinclude measure-response.msg.html

(cl:defclass <measure-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass measure-response (<measure-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <measure-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'measure-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmp085-srv:<measure-response> is deprecated: use bmp085-srv:measure-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <measure-response>) ostream)
  "Serializes a message object of type '<measure-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <measure-response>) istream)
  "Deserializes a message object of type '<measure-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<measure-response>)))
  "Returns string type for a service object of type '<measure-response>"
  "bmp085/measureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'measure-response)))
  "Returns string type for a service object of type 'measure-response"
  "bmp085/measureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<measure-response>)))
  "Returns md5sum for a message object of type '<measure-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'measure-response)))
  "Returns md5sum for a message object of type 'measure-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<measure-response>)))
  "Returns full string definition for message of type '<measure-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'measure-response)))
  "Returns full string definition for message of type 'measure-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <measure-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <measure-response>))
  "Converts a ROS message object to a list"
  (cl:list 'measure-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'measure)))
  'measure-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'measure)))
  'measure-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'measure)))
  "Returns string type for a service object of type '<measure>"
  "bmp085/measure")