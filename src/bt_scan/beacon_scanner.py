#Beacon Scanner for PaLOMA

import ScanUtility
import os
import bluetooth._bluetooth as bluez

#Set bluetooth device. Default 0.
dev_id = 0

class BtscannerDriver:

	def __init__(self, max_attemps = 10):
		self.max_attemps = max_attemps
		self.beaconInfo = []
		self.beacons_detected = 0
		try:
			self.sock = bluez.hci_open_dev(dev_id)
			print ("\n *** Socket Active ***")
			print ("\n *** CTRL-C to Cancel ***")
		except:
			print ("Error accessing bluetooth")
		ScanUtility.hci_enable_le_scan(self.sock)
		print ("\n *** SCAN ENABLED ***\n")

	def set_max_attemps(self,attemps_param):
		self.max_attemps = attemps_param

	def get_beacon_info(self):
		self.beaconInfo = []
		while self.beaconInfo == [] :
			self.beaconInfo = ScanUtility.parse_events(self.sock, self.max_attemps)
		return (self.beaconInfo)

	def stop(self):
		#os.popen('sudo hciconfig hci0 reset')
		print("BT STOPPED")

