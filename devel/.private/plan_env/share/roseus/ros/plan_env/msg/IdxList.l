;; Auto-generated. Do not edit!


(when (boundp 'plan_env::IdxList)
  (if (not (find-package "PLAN_ENV"))
    (make-package "PLAN_ENV"))
  (shadow 'IdxList (find-package "PLAN_ENV")))
(unless (find-package "PLAN_ENV::IDXLIST")
  (make-package "PLAN_ENV::IDXLIST"))

(in-package "ROS")
;;//! \htmlinclude IdxList.msg.html


(defclass plan_env::IdxList
  :super ros::object
  :slots (_ids ))

(defmethod plan_env::IdxList
  (:init
   (&key
    ((:ids __ids) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _ids __ids)
   self)
  (:ids
   (&optional __ids)
   (if __ids (setq _ids __ids)) _ids)
  (:serialization-length
   ()
   (+
    ;; int32[] _ids
    (* 4    (length _ids)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _ids
     (write-long (length _ids) s)
     (dotimes (i (length _ids))
       (write-long (elt _ids i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _ids i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get plan_env::IdxList :md5sum-) "4f22efebf407aadba2ecc69df353d113")
(setf (get plan_env::IdxList :datatype-) "plan_env/IdxList")
(setf (get plan_env::IdxList :definition-)
      "int32[] ids
")



(provide :plan_env/IdxList "4f22efebf407aadba2ecc69df353d113")


