;; Auto-generated. Do not edit!


(when (boundp 'msg_set::DroneState)
  (if (not (find-package "MSG_SET"))
    (make-package "MSG_SET"))
  (shadow 'DroneState (find-package "MSG_SET")))
(unless (find-package "MSG_SET::DRONESTATE")
  (make-package "MSG_SET::DRONESTATE"))

(in-package "ROS")
;;//! \htmlinclude DroneState.msg.html


(defclass msg_set::DroneState
  :super ros::object
  :slots (_drone_id _grid_ids _recent_attempt_time _stamp _pos _vel _yaw _cur_state ))

(defmethod msg_set::DroneState
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:grid_ids __grid_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:recent_attempt_time __recent_attempt_time) 0.0)
    ((:stamp __stamp) 0.0)
    ((:pos __pos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:yaw __yaw) 0.0)
    ((:cur_state __cur_state) 0)
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _grid_ids __grid_ids)
   (setq _recent_attempt_time (float __recent_attempt_time))
   (setq _stamp (float __stamp))
   (setq _pos __pos)
   (setq _vel __vel)
   (setq _yaw (float __yaw))
   (setq _cur_state (round __cur_state))
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:grid_ids
   (&optional __grid_ids)
   (if __grid_ids (setq _grid_ids __grid_ids)) _grid_ids)
  (:recent_attempt_time
   (&optional __recent_attempt_time)
   (if __recent_attempt_time (setq _recent_attempt_time __recent_attempt_time)) _recent_attempt_time)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:cur_state
   (&optional __cur_state)
   (if __cur_state (setq _cur_state __cur_state)) _cur_state)
  (:serialization-length
   ()
   (+
    ;; int32 _drone_id
    4
    ;; int8[] _grid_ids
    (* 1    (length _grid_ids)) 4
    ;; float64 _recent_attempt_time
    8
    ;; float64 _stamp
    8
    ;; float32[] _pos
    (* 4    (length _pos)) 4
    ;; float32[] _vel
    (* 4    (length _vel)) 4
    ;; float32 _yaw
    4
    ;; int32 _cur_state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _drone_id
       (write-long _drone_id s)
     ;; int8[] _grid_ids
     (write-long (length _grid_ids) s)
     (dotimes (i (length _grid_ids))
       (write-byte (elt _grid_ids i) s)
       )
     ;; float64 _recent_attempt_time
       (sys::poke _recent_attempt_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _stamp
       (sys::poke _stamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32[] _pos
     (write-long (length _pos) s)
     (dotimes (i (length _pos))
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _vel
     (write-long (length _vel) s)
     (dotimes (i (length _vel))
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _cur_state
       (write-long _cur_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _drone_id
     (setq _drone_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8[] _grid_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _grid_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _grid_ids i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _grid_ids i) 127) (setf (elt _grid_ids i) (- (elt _grid_ids i) 256)))
     ))
   ;; float64 _recent_attempt_time
     (setq _recent_attempt_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _stamp
     (setq _stamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vel i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _cur_state
     (setq _cur_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get msg_set::DroneState :md5sum-) "d5585437c78c5e2a88978fc6a150fd46")
(setf (get msg_set::DroneState :datatype-) "msg_set/DroneState")
(setf (get msg_set::DroneState :definition-)
      "int32 drone_id

int8[] grid_ids
float64 recent_attempt_time
float64 stamp

# only used for simulation
float32[] pos
float32[] vel
float32 yaw
int32 cur_state
")



(provide :msg_set/DroneState "d5585437c78c5e2a88978fc6a150fd46")


