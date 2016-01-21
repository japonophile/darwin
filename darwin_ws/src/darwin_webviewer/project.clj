(defproject darwin-repo "0.1.0-SNAPSHOT"
  :description "Web UI for Darwin Mini robot model"
  :url "http://chopp.in/clj"
  :license {:name "Eclipse Public License"
            :url "http://www.eclipse.org/legal/epl-v10.html"}
  :dependencies [[org.clojure/clojure "1.7.0"]
                 [ring "1.2.0"]]
  :plugins [[lein-ring "0.8.7"]]
  :ring {:handler darwin-webviewer.core/main-handler
         :init    darwin-webviewer.core/on-init
         :destroy darwin-webviewer.core/on-destroy})
