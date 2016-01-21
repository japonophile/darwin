(ns darwin-webviewer.core
  (:require [ring.middleware.resource :as resource]
            [ring.middleware.file-info :as file-info]))

(defn default-handler [request]
  {:body "Hello world!"})

(def main-handler
  (-> default-handler
   (resource/wrap-resource "public")
   file-info/wrap-file-info))

(defn on-init []
  )

(defn on-destroy []
  )

