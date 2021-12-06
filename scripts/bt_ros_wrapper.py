#!/usr/bin/env python

import rospy
from bt_scan.beacon_scanner import BtscannerDriver
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Trigger
from bt_scan.msg import beacon_info

class BtScannerROSWrapper:

	def __init__(self):
		self.max_attemps = rospy.get_param("~max_attemps", 10)
		self.max_scan_freq = rospy.get_param("~max_scan_freq", 5)
		self.scanner = BtscannerDriver(self.max_attemps)

		# Subscriber <- scan frequency
		rospy.Subscriber("scan_freq_command", Int32, self.callback_set_max_freq)
		# Subscriber <- number of attemps to scan
		rospy.Subscriber("detect_attemps_command", Int32, self.callback_attemps_command)
		# Subscriber <- detected beacon callback
		rospy.Subscriber("detected_beacon", beacon_info, self.callback_detected_beacon)

		# Service <> stop scanner
		rospy.Service("stop_scanner", Trigger, self.callback_stop)

		# Publisher -> max scan frequency
		self.max_freq_pub = rospy.Publisher("max_scan_freq", Int32, queue_size = 1)
		# Publisher -> beacon info publisher
		self.max_attemps_pub = rospy.Publisher("detect_attemps_command", Int32, queue_size = 1)
		# Publisher -> beacon info publisher, beacon_info is custom message
		self.beacon_info_pub = rospy.Publisher("detected_beacon", beacon_info, queue_size = 1)

	# ---------- Callbacks ---------------	
	def callback_set_max_freq(self, msg):
		self.max_scan_freq = msg.data

	def callback_attemps_command(self, msg):
		max_attemps = msg.data
		self.scanner.set_max_attemps(max_attemps)

	def callback_detected_beacon(self, msg):
		print("Beacon detected")

	# ----------- Publishing -------------
		# publish max scan freq
	def publish_max_freq(self):
		self.max_freq_pub.publish(self.max_scan_freq)
		# publish attemps command
	def publish_max_attemps(self):
		self.max_attemps_pub.publish(self.scanner.max_attemps)
		# publish beacon info
	def publish_beacon_info(self):
		actualMsg = beacon_info()
		actualMsg.header.stamp = rospy.Time.now()
		actualMsg.header.frame_id = "Beacon"
		beaconInfo = self.scanner.get_beacon_info()
		#print(actualMsg.header,"\n >>> ")
		actualMsg.type = beaconInfo[0]["type"]
		#print(actualMsg.type," / ")
		actualMsg.uuid = beaconInfo[0]["uuid"]
		#print(actualMsg.uuid," / ")
		actualMsg.majorVal = beaconInfo[0]["major"]
		#print(actualMsg.majorVal," / ")
		actualMsg.minorVal = beaconInfo[0]["minor"]
		#print(actualMsg.minorVal," / ")
		actualMsg.rssi = beaconInfo[0]["rssi"]
		#print(actualMsg.rssi," / ")
		actualMsg.macAddress = beaconInfo[0]["macAddress"]
		#print(actualMsg.macAddress," / ")
		actualMsg.url = beaconInfo[0]["url"]
		#print(actualMsg.url," / ")
		actualMsg.namespace = beaconInfo[0]["namespace"]
		#print(actualMsg.namespace," / ")
		actualMsg.instance = beaconInfo[0]["instance"]
		#print(actualMsg.instance," <<< ")
		self.beacon_info_pub.publish(actualMsg)

	# -------------- Stop ----------------
	def stop(self):
		self.scanner.stop()

	def callback_stop(self, req):
		self.stop()
		print("success  : Scanner has been stopped")


if __name__ == "__main__":
	rospy.init_node("beacon_scanner")

	bcnscanner = BtScannerROSWrapper()
	rospy.on_shutdown(bcnscanner.stop)
	rospy.loginfo("BT scanner driver started")
	rate = rospy.Rate(bcnscanner.max_scan_freq)
	while not rospy.is_shutdown():
		bcnscanner.publish_max_freq()
		bcnscanner.publish_max_attemps()
		bcnscanner.publish_beacon_info()
		rate.sleep()

	#rospy.spin()

