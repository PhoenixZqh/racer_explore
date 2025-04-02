;; Auto-generated. Do not edit!


(when (boundp 'plan_env::ChunkStamps)
  (if (not (find-package "PLAN_ENV"))
    (make-package "PLAN_ENV"))
  (shadow 'ChunkStamps (find-package "PLAN_ENV")))
(unless (find-package "PLAN_ENV::CHUNKSTAMPS")
  (make-package "PLAN_ENV::CHUNKSTAMPS"))

(in-package "ROS")
;;//! \htmlinclude ChunkStamps.msg.html


(defclass plan_env::ChunkStamps
  :super ros::object
  :slots (_from_drone_id _idx_lists _time ))

(defmethod plan_env::ChunkStamps
  (:init
   (&key
    ((:from_drone_id __from_drone_id) 0)
    ((:idx_lists __idx_lists) ())
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _from_drone_id (round __from_drone_id))
   (setq _idx_lists __idx_lists)
   (setq _time (float __time))
   self)
  (:from_drone_id
   (&optional __from_drone_id)
   (if __from_drone_id (setq _from_drone_id __from_drone_id)) _from_drone_id)
  (:idx_lists
   (&rest __idx_lists)
   (if (keywordp (car __idx_lists))
       (send* _idx_lists __idx_lists)
     (progn
       (if __idx_lists (setq _idx_lists (car __idx_lists)))
       _idx_lists)))
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; int32 _from_drone_id
    4
    ;; plan_env/IdxList[] _idx_lists
    (apply #'+ (send-all _idx_lists :serialization-length)) 4
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _from_drone_id
       (write-long _from_drone_id s)
     ;; plan_env/IdxList[] _idx_lists
     (write-long (length _idx_lists) s)
     (dolist (elem _idx_lists)
       (send elem :serialize s)
       )
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _from_drone_id
     (setq _from_drone_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; plan_env/IdxList[] _idx_lists
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _idx_lists (let (r) (dotimes (i n) (push (instance plan_env::IdxList :init) r)) r))
     (dolist (elem- _idx_lists)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get plan_env::ChunkStamps :md5sum-) "4649dd6de5a06d0faa920a6802067e3f")
(setf (get plan_env::ChunkStamps :datatype-) "plan_env/ChunkStamps")
(setf (get plan_env::ChunkStamps :definition-)
      "
int32 from_drone_id
IdxList[] idx_lists
float64 time
================================================================================
MSG: plan_env/IdxList
int32[] ids
")



(provide :plan_env/ChunkStamps "4649dd6de5a06d0faa920a6802067e3f")


