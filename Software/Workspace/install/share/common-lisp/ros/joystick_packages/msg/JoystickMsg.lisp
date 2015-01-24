; Auto-generated. Do not edit!


(cl:in-package joystick_packages-msg)


;//! \htmlinclude JoystickMsg.msg.html

(cl:defclass <JoystickMsg> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (number
    :reader number
    :initarg :number
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass JoystickMsg (<JoystickMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoystickMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoystickMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name joystick_packages-msg:<JoystickMsg> is deprecated: use joystick_packages-msg:JoystickMsg instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <JoystickMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_packages-msg:type-val is deprecated.  Use joystick_packages-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <JoystickMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_packages-msg:number-val is deprecated.  Use joystick_packages-msg:number instead.")
  (number m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <JoystickMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_packages-msg:value-val is deprecated.  Use joystick_packages-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoystickMsg>) ostream)
  "Serializes a message object of type '<JoystickMsg>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoystickMsg>) istream)
  "Deserializes a message object of type '<JoystickMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoystickMsg>)))
  "Returns string type for a message object of type '<JoystickMsg>"
  "joystick_packages/JoystickMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoystickMsg)))
  "Returns string type for a message object of type 'JoystickMsg"
  "joystick_packages/JoystickMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoystickMsg>)))
  "Returns md5sum for a message object of type '<JoystickMsg>"
  "d852a84dd4742fd04b95b69c878d3172")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoystickMsg)))
  "Returns md5sum for a message object of type 'JoystickMsg"
  "d852a84dd4742fd04b95b69c878d3172")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoystickMsg>)))
  "Returns full string definition for message of type '<JoystickMsg>"
  (cl:format cl:nil "int8 type~%int8 number~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoystickMsg)))
  "Returns full string definition for message of type 'JoystickMsg"
  (cl:format cl:nil "int8 type~%int8 number~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoystickMsg>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoystickMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'JoystickMsg
    (cl:cons ':type (type msg))
    (cl:cons ':number (number msg))
    (cl:cons ':value (value msg))
))
