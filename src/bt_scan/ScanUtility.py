#This is a working prototype. DO NOT USE IT IN LIVE PROJECTS


import sys
import struct
import bluetooth._bluetooth as bluez

OGF_LE_CTL=0x08
OCF_LE_SET_SCAN_ENABLE=0x000C

def hci_enable_le_scan(sock):
    hci_toggle_le_scan(sock, 0x01)

def hci_disable_le_scan(sock):
    hci_toggle_le_scan(sock, 0x00)

def hci_toggle_le_scan(sock, enable):
    cmd_pkt = struct.pack("<BB", enable, 0x00)
    bluez.hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, cmd_pkt)

def packetToString(packet):
    """
    Returns the string representation of a raw HCI packet.
    """
    if sys.version_info > (3, 0):
        return ''.join('%02x' % struct.unpack("B", bytes([x]))[0] for x in packet)
    else:
        return ''.join('%02x' % struct.unpack("B", x)[0] for x in packet)

def parse_events(sock, loop_count=100):
    old_filter = sock.getsockopt( bluez.SOL_HCI, bluez.HCI_FILTER, 14)
    flt = bluez.hci_filter_new()
    bluez.hci_filter_all_events(flt)
    bluez.hci_filter_set_ptype(flt, bluez.HCI_EVENT_PKT)
    sock.setsockopt( bluez.SOL_HCI, bluez.HCI_FILTER, flt )
    # define resultsArray (RATM)
    resultsArray = [{"type": "", "uuid": "", "major": 0, "minor": 0,
                     "rssi": 0, "macAddress": "", "url": "", 
                     "namespace": "", "instance": ""}]
    for i in range(0, loop_count):
        packet = sock.recv(255)
        ptype, event, plen = struct.unpack("BBB", packet[:3])
        packetOffset = 0
        dataString = packetToString(packet)
        """
        If the bluetooth device is an beacon then show the beacon.
        """
        if (dataString[34:42] == '0303aafe') and (dataString[44:50] == '16aafe'):
            """
            Selects parts of the bluetooth packets.
            """
            broadcastType = dataString[50:52]
            if broadcastType == '00' :
                resultsArray[0]["type"] = "Eddystone UID"
                resultsArray[0]["namespace"] = str(dataString[54:74].upper())
                resultsArray[0]["instance"] = str(dataString[74:86].upper())
                #resultsArray = [{"type": type, "namespace": namespace, "instance": instance}]
                return resultsArray

            elif broadcastType == '10':
                resultsArray[0]["type"] = "Eddystone URL"
                urlprefix = dataString[54:56]
                if urlprefix == '00':
                    prefix = 'http://www.'
                elif urlprefix == '01':
                    prefix = 'https://www.'
                elif urlprefix == '02':
                    prefix = 'http://'
                elif urlprefix == '03':
                    prefix = 'https://'
                hexUrl = dataString[56:][:-2]
                if sys.version_info[0] == 3:
                    resultsArray[0]["url"] = prefix + bytes.fromhex(hexUrl).decode('utf-8')
                    rssiVal = struct.unpack("b", bytes([packet[packetOffset-1]]))
                    resultsArray[0]["rssi"] = int(rssiVal[0])
                else:
                    resultsArray[0]["url"] = prefix + hexUrl.decode("hex")
                    rssiVal = struct.unpack("b", packet[packetOffset-1])
                    resultsArray[0]["rssi"] = int(rssiVal[0])
                #resultsArray = [{"type": type, "url": url}]
                return resultsArray

            elif broadcastType == '20':
                resultsArray[0]["type"] = "Eddystone TLM"
                #resultsArray = [{"type": type}]
                return resultsArray

            elif broadcastType == '30':
                resultsArray[0]["type"] = "Eddystone EID"
                #resultsArray = [{"type": type}]
                return resultsArray

            elif broadcastType == '40':
                resultsArray[0]["type"] = "Eddystone RESERVED"
                #resultsArray = [{"type": type}]
                return resultsArray
            else: # ------RATM------
                resultsArray[0]["type"] = "Eddystone UNKNOWN"
                #resultsArray = [{"type": type}]
                return resultsArray # ------RATM------
        if dataString[38:46] == '4c000215':
            """
            Selects parts of the bluetooth packets.
            """
            resultsArray[0]["type"] = "iBeacon"
            resultsArray[0]["uuid"] = dataString[46:54] + "-" + dataString[54:58] + "-" + dataString[58:62] + "-" + dataString[62:66] + "-" + dataString[66:78]
            major = dataString[78:82]
            minor = dataString[82:86]
            resultsArray[0]["major"] = int("".join(major.split()[::-1]), 16)
            resultsArray[0]["minor"] = int("".join(minor.split()[::-1]), 16)
            """
            Organises Mac Address to display properly
            """
            scrambledAddress = dataString[14:26]
            fixStructure = iter("".join(reversed([scrambledAddress[i:i+2] for i in range(0, len(scrambledAddress), 2)])))
            resultsArray[0]["macAddress"] = ':'.join(a+b for a,b in zip(fixStructure, fixStructure))
            if sys.version_info[0] == 3:
               rssiVal = struct.unpack("b", bytes([packet[packetOffset-1]]))
               resultsArray[0]["rssi"] = int(rssiVal[0])
            else:
               rssiVal = struct.unpack("b", packet[packetOffset-1])
               resultsArray[0]["rssi"] = int(rssiVal[0])
            #resultsArray = [{"type": type, "uuid": uuid, "major": majorVal, "minor": minorVal, "rssi": rssi, "macAddress": macAddress}]
            return resultsArray
    return resultsArray
