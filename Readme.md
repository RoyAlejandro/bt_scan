

Package name: bt_scan

Node name: beacon_scanner

ROS wrapper file: bt_ros_wrapper.py

ROS wrapper class: BtScannerROSWrapper

bluetooth driver file: beacon_scanner.py

bluetooth driver class: BtscannerDriver

### INSTALLATION

1) git clone or copy bt_scan folder into your workspace ie: catkin_ws/bt_scan
2) catkin_make


### USAGE

1) roscore
2) sudo bash
3) rosrun bt_scan bt_ros_wrapper.py


### -------------------------------------------------------

Inspired on:
https://github.com/bowdentheo/BLE-Beacon-Scanner

Following instructions from:
https://roboticsbackend.com/create-a-ros-driver-package-introduction-what-is-a-ros-wrapper-1-4/
