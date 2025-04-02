;; Auto-generated. Do not edit!


(when (boundp 'exploration_manager::PairOpt)
  (if (not (find-package "EXPLORATION_MANAGER"))
    (make-package "EXPLORATION_MANAGER"))
  (shadow 'PairOpt (find-package "EXPLORATION_MANAGER")))
(unless (find-package "EXPLORATION_MANAGER::PAIROPT")
  (make-package "EXPLORATION_MANAGER::PAIROPT"))

(in-package "ROS")
;;//! \htmlinclude PairOpt.msg.html


(defclass exploration_manager::PairOpt
  :super ros::object
  :slots (_from_drone_id _to_drone_id _stamp _ego_ids _other_ids ))

(defmethod exploration_manager::PairOpt
  (:init
   (&key
    ((:from_drone_id __from_drone_id) 0)
    ((:to_drone_id __to_drone_id) 0)
    ((:stamp __stamp) 0.0)
    ((:ego_ids __ego_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:other_ids __other_ids) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _from_drone_id (round __from_drone_id))
   (setq _to_drone_id (round __to_drone_id))
   (setq _stamp (float __stamp))
   (setq _ego_ids __ego_ids)
   (setq _other_ids __other_ids)
   self)
  (:from_drone_id
   (&optional __from_drone_id)
   (if __from_drone_id (setq _from_drone_id __from_drone_id)) _from_drone_id)
  (:to_drone_id
   (&optional __to_drone_id)
   (if __to_drone_id (setq _to_drone_id __to_drone_id)) _to_drone_id)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:ego_ids
   (&optional __ego_ids)
   (if __ego_ids (setq _ego_ids __ego_ids)) _ego_ids)
  (:other_ids
   (&optional __other_ids)
   (if __other_ids (setq _other_ids __other_ids)) _other_ids)
  (:serialization-length
   ()
   (+
    ;; int32 _from_drone_id
    4
    ;; int32 _to_drone_id
    4
    ;; float64 _stamp
    8
    ;; int8[] _ego_ids
    (* 1    (length _ego_ids)) 4
    ;; int8[] _other_ids
    (* 1    (length _other_ids)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _from_drone_id
       (write-long _from_drone_id s)
     ;; int32 _to_drone_id
       (write-long _to_drone_id s)
     ;; float64 _stamp
       (sys::poke _stamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8[] _ego_ids
     (write-long (length _ego_ids) s)
     (dotimes (i (length _ego_ids))
       (write-byte (elt _ego_ids i) s)
       )
     ;; int8[] _other_ids
     (write-long (length _other_ids) s)
     (dotimes (i (length _other_ids))
       (write-byte (elt _other_ids i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _from_drone_id
     (setq _from_drone_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _to_drone_id
     (setq _to_drone_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _stamp
     (setq _stamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8[] _ego_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ego_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _ego_ids i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _ego_ids i) 127) (setf (elt _ego_ids i) (- (elt _ego_ids i) 256)))
     ))
   ;; int8[] _other_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _other_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _other_ids i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _other_ids i) 127) (setf (elt _other_ids i) (- (elt _other_ids i) 256)))
     ))
   ;;
   self)
  )

(setf (get exploration_manager::PairOpt :md5sum-) "f1b382d48f9c952cdea39f0f21949ad6")
(setf (get exploration_manager::PairOpt :datatype-) "exploration_manager/PairOpt")
(setf (get exploration_manager::PairOpt :definition-)
      "
int32 from_drone_id
int32 to_drone_id

float64 stamp
int8[] ego_ids
int8[] other_ids

")



(provide :exploration_manager/PairOpt "f1b382d48f9c952cdea39f0f21949ad6")


