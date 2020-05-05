(ns darwin-moves-controller.core
  (:require [asimov.api :as ros]
            [clojure.core.async :as a]))

(def msgs (ros/msgs ["resources"]))

(def node (ros/init-node! "/darwin_move_controller" :client-host "localhost" :master-host "localhost" :master-port 11311))

(def joints ["neck", "l_shoulder", "r_shoulder", "l_biceps", "r_biceps", "l_elbow", "r_elbow", "l_hip", "r_hip", "l_thigh", "r_thigh", "l_knee", "r_knee", "l_ankle", "r_ankle", "l_foot", "r_foot"])

(def joint-channels
  (zipmap joints
          (map #(ros/pub! node
                          (msgs {:package "std_msgs" :name "Float64"})
                          (str "/darwin/" % "_joint_position_controller/command"))
               joints)))

(defn take-pose [q]
  (doall (map #(a/put! (joint-channels (first %1)) {:data (second %1)}) q)))

;; (take-pose {"r_shoulder" 3.2 "l_shoulder" 2.0})

