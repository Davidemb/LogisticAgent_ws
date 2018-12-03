; Auto-generated. Do not edit!


(cl:in-package patrolling_sim-srv)


;//! \htmlinclude GoToStartPosSrv-request.msg.html

(cl:defclass <GoToStartPosSrv-request> (roslisp-msg-protocol:ros-message)
  ((teamsize
    :reader teamsize
    :initarg :teamsize
    :type std_msgs-msg:UInt8
    :initform (cl:make-instance 'std_msgs-msg:UInt8))
   (sleep_between_goals
    :reader sleep_between_goals
    :initarg :sleep_between_goals
    :type std_msgs-msg:UInt8
    :initform (cl:make-instance 'std_msgs-msg:UInt8)))
)

(cl:defclass GoToStartPosSrv-request (<GoToStartPosSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToStartPosSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToStartPosSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name patrolling_sim-srv:<GoToStartPosSrv-request> is deprecated: use patrolling_sim-srv:GoToStartPosSrv-request instead.")))

(cl:ensure-generic-function 'teamsize-val :lambda-list '(m))
(cl:defmethod teamsize-val ((m <GoToStartPosSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader patrolling_sim-srv:teamsize-val is deprecated.  Use patrolling_sim-srv:teamsize instead.")
  (teamsize m))

(cl:ensure-generic-function 'sleep_between_goals-val :lambda-list '(m))
(cl:defmethod sleep_between_goals-val ((m <GoToStartPosSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader patrolling_sim-srv:sleep_between_goals-val is deprecated.  Use patrolling_sim-srv:sleep_between_goals instead.")
  (sleep_between_goals m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToStartPosSrv-request>) ostream)
  "Serializes a message object of type '<GoToStartPosSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'teamsize) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sleep_between_goals) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToStartPosSrv-request>) istream)
  "Deserializes a message object of type '<GoToStartPosSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'teamsize) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sleep_between_goals) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToStartPosSrv-request>)))
  "Returns string type for a service object of type '<GoToStartPosSrv-request>"
  "patrolling_sim/GoToStartPosSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToStartPosSrv-request)))
  "Returns string type for a service object of type 'GoToStartPosSrv-request"
  "patrolling_sim/GoToStartPosSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToStartPosSrv-request>)))
  "Returns md5sum for a message object of type '<GoToStartPosSrv-request>"
  "46245284711564dca6f0ea10c19a6573")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToStartPosSrv-request)))
  "Returns md5sum for a message object of type 'GoToStartPosSrv-request"
  "46245284711564dca6f0ea10c19a6573")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToStartPosSrv-request>)))
  "Returns full string definition for message of type '<GoToStartPosSrv-request>"
  (cl:format cl:nil "std_msgs/UInt8 teamsize~%std_msgs/UInt8 sleep_between_goals~%~%================================================================================~%MSG: std_msgs/UInt8~%uint8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToStartPosSrv-request)))
  "Returns full string definition for message of type 'GoToStartPosSrv-request"
  (cl:format cl:nil "std_msgs/UInt8 teamsize~%std_msgs/UInt8 sleep_between_goals~%~%================================================================================~%MSG: std_msgs/UInt8~%uint8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToStartPosSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'teamsize))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sleep_between_goals))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToStartPosSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToStartPosSrv-request
    (cl:cons ':teamsize (teamsize msg))
    (cl:cons ':sleep_between_goals (sleep_between_goals msg))
))
;//! \htmlinclude GoToStartPosSrv-response.msg.html

(cl:defclass <GoToStartPosSrv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GoToStartPosSrv-response (<GoToStartPosSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToStartPosSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToStartPosSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name patrolling_sim-srv:<GoToStartPosSrv-response> is deprecated: use patrolling_sim-srv:GoToStartPosSrv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToStartPosSrv-response>) ostream)
  "Serializes a message object of type '<GoToStartPosSrv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToStartPosSrv-response>) istream)
  "Deserializes a message object of type '<GoToStartPosSrv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToStartPosSrv-response>)))
  "Returns string type for a service object of type '<GoToStartPosSrv-response>"
  "patrolling_sim/GoToStartPosSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToStartPosSrv-response)))
  "Returns string type for a service object of type 'GoToStartPosSrv-response"
  "patrolling_sim/GoToStartPosSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToStartPosSrv-response>)))
  "Returns md5sum for a message object of type '<GoToStartPosSrv-response>"
  "46245284711564dca6f0ea10c19a6573")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToStartPosSrv-response)))
  "Returns md5sum for a message object of type 'GoToStartPosSrv-response"
  "46245284711564dca6f0ea10c19a6573")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToStartPosSrv-response>)))
  "Returns full string definition for message of type '<GoToStartPosSrv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToStartPosSrv-response)))
  "Returns full string definition for message of type 'GoToStartPosSrv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToStartPosSrv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToStartPosSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToStartPosSrv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoToStartPosSrv)))
  'GoToStartPosSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoToStartPosSrv)))
  'GoToStartPosSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToStartPosSrv)))
  "Returns string type for a service object of type '<GoToStartPosSrv>"
  "patrolling_sim/GoToStartPosSrv")