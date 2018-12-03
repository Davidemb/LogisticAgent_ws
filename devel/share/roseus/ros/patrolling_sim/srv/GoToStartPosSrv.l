;; Auto-generated. Do not edit!


(when (boundp 'patrolling_sim::GoToStartPosSrv)
  (if (not (find-package "PATROLLING_SIM"))
    (make-package "PATROLLING_SIM"))
  (shadow 'GoToStartPosSrv (find-package "PATROLLING_SIM")))
(unless (find-package "PATROLLING_SIM::GOTOSTARTPOSSRV")
  (make-package "PATROLLING_SIM::GOTOSTARTPOSSRV"))
(unless (find-package "PATROLLING_SIM::GOTOSTARTPOSSRVREQUEST")
  (make-package "PATROLLING_SIM::GOTOSTARTPOSSRVREQUEST"))
(unless (find-package "PATROLLING_SIM::GOTOSTARTPOSSRVRESPONSE")
  (make-package "PATROLLING_SIM::GOTOSTARTPOSSRVRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass patrolling_sim::GoToStartPosSrvRequest
  :super ros::object
  :slots (_teamsize _sleep_between_goals ))

(defmethod patrolling_sim::GoToStartPosSrvRequest
  (:init
   (&key
    ((:teamsize __teamsize) (instance std_msgs::UInt8 :init))
    ((:sleep_between_goals __sleep_between_goals) (instance std_msgs::UInt8 :init))
    )
   (send-super :init)
   (setq _teamsize __teamsize)
   (setq _sleep_between_goals __sleep_between_goals)
   self)
  (:teamsize
   (&rest __teamsize)
   (if (keywordp (car __teamsize))
       (send* _teamsize __teamsize)
     (progn
       (if __teamsize (setq _teamsize (car __teamsize)))
       _teamsize)))
  (:sleep_between_goals
   (&rest __sleep_between_goals)
   (if (keywordp (car __sleep_between_goals))
       (send* _sleep_between_goals __sleep_between_goals)
     (progn
       (if __sleep_between_goals (setq _sleep_between_goals (car __sleep_between_goals)))
       _sleep_between_goals)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/UInt8 _teamsize
    (send _teamsize :serialization-length)
    ;; std_msgs/UInt8 _sleep_between_goals
    (send _sleep_between_goals :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/UInt8 _teamsize
       (send _teamsize :serialize s)
     ;; std_msgs/UInt8 _sleep_between_goals
       (send _sleep_between_goals :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/UInt8 _teamsize
     (send _teamsize :deserialize buf ptr-) (incf ptr- (send _teamsize :serialization-length))
   ;; std_msgs/UInt8 _sleep_between_goals
     (send _sleep_between_goals :deserialize buf ptr-) (incf ptr- (send _sleep_between_goals :serialization-length))
   ;;
   self)
  )

(defclass patrolling_sim::GoToStartPosSrvResponse
  :super ros::object
  :slots ())

(defmethod patrolling_sim::GoToStartPosSrvResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass patrolling_sim::GoToStartPosSrv
  :super ros::object
  :slots ())

(setf (get patrolling_sim::GoToStartPosSrv :md5sum-) "46245284711564dca6f0ea10c19a6573")
(setf (get patrolling_sim::GoToStartPosSrv :datatype-) "patrolling_sim/GoToStartPosSrv")
(setf (get patrolling_sim::GoToStartPosSrv :request) patrolling_sim::GoToStartPosSrvRequest)
(setf (get patrolling_sim::GoToStartPosSrv :response) patrolling_sim::GoToStartPosSrvResponse)

(defmethod patrolling_sim::GoToStartPosSrvRequest
  (:response () (instance patrolling_sim::GoToStartPosSrvResponse :init)))

(setf (get patrolling_sim::GoToStartPosSrvRequest :md5sum-) "46245284711564dca6f0ea10c19a6573")
(setf (get patrolling_sim::GoToStartPosSrvRequest :datatype-) "patrolling_sim/GoToStartPosSrvRequest")
(setf (get patrolling_sim::GoToStartPosSrvRequest :definition-)
      "std_msgs/UInt8 teamsize
std_msgs/UInt8 sleep_between_goals

================================================================================
MSG: std_msgs/UInt8
uint8 data
---
")

(setf (get patrolling_sim::GoToStartPosSrvResponse :md5sum-) "46245284711564dca6f0ea10c19a6573")
(setf (get patrolling_sim::GoToStartPosSrvResponse :datatype-) "patrolling_sim/GoToStartPosSrvResponse")
(setf (get patrolling_sim::GoToStartPosSrvResponse :definition-)
      "std_msgs/UInt8 teamsize
std_msgs/UInt8 sleep_between_goals

================================================================================
MSG: std_msgs/UInt8
uint8 data
---
")



(provide :patrolling_sim/GoToStartPosSrv "46245284711564dca6f0ea10c19a6573")


