; Auto-generated. Do not edit!


(cl:in-package cameras-msg)


;//! \htmlinclude GrayImage.msg.html

(cl:defclass <GrayImage> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GrayImage (<GrayImage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GrayImage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GrayImage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cameras-msg:<GrayImage> is deprecated: use cameras-msg:GrayImage instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <GrayImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cameras-msg:image-val is deprecated.  Use cameras-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <GrayImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cameras-msg:width-val is deprecated.  Use cameras-msg:width instead.")
  (width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GrayImage>) ostream)
  "Serializes a message object of type '<GrayImage>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'image))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'image))
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GrayImage>) istream)
  "Deserializes a message object of type '<GrayImage>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'image) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'image)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GrayImage>)))
  "Returns string type for a message object of type '<GrayImage>"
  "cameras/GrayImage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GrayImage)))
  "Returns string type for a message object of type 'GrayImage"
  "cameras/GrayImage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GrayImage>)))
  "Returns md5sum for a message object of type '<GrayImage>"
  "df5494704111269f7ec4fa328ea1a39e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GrayImage)))
  "Returns md5sum for a message object of type 'GrayImage"
  "df5494704111269f7ec4fa328ea1a39e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GrayImage>)))
  "Returns full string definition for message of type '<GrayImage>"
  (cl:format cl:nil "int8[] image~%int8 width~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GrayImage)))
  "Returns full string definition for message of type 'GrayImage"
  (cl:format cl:nil "int8[] image~%int8 width~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GrayImage>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'image) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GrayImage>))
  "Converts a ROS message object to a list"
  (cl:list 'GrayImage
    (cl:cons ':image (image msg))
    (cl:cons ':width (width msg))
))
