from time import sleep
import argparse
import threading
import ctypes
import os
import struct
import datetime
import time
import math
import traceback
from importlib import util

class RNS():
	@staticmethod
	def log(msg):
		logtimefmt   = "%Y-%m-%d %H:%M:%S"
		timestamp = time.time()
		logstring = "["+time.strftime(logtimefmt)+"] "+msg
		print(logstring)

	@staticmethod
	def hexrep(data, delimit=True):
		delimiter = ":"
		if not delimit:
			delimiter = ""
		hexrep = delimiter.join("{:02x}".format(c) for c in data)
		return hexrep

	@staticmethod
	def prettyhexrep(data):
		delimiter = ""
		hexrep = "<"+delimiter.join("{:02x}".format(c) for c in data)+">"
		return hexrep

class KISS():
	FEND			= 0xC0
	FESC			= 0xDB
	TFEND			= 0xDC
	TFESC			= 0xDD

	CMD_UNKNOWN		= 0xFE
	CMD_DATA		= 0x00
	CMD_FREQUENCY	= 0x01
	CMD_BANDWIDTH	= 0x02
	CMD_TXPOWER		= 0x03
	CMD_SF			= 0x04
	CMD_CR          = 0x05
	CMD_RADIO_STATE = 0x06
	CMD_RADIO_LOCK	= 0x07
	CMD_DETECT		= 0x08
	CMD_PROMISC     = 0x0E
	CMD_READY       = 0x0F
	CMD_STAT_RX		= 0x21
	CMD_STAT_TX		= 0x22
	CMD_STAT_RSSI	= 0x23
	CMD_STAT_SNR	= 0x24
	CMD_BLINK		= 0x30
	CMD_RANDOM		= 0x40
	CMD_FW_VERSION  = 0x50
	CMD_ROM_READ    = 0x51
	CMD_ROM_WRITE   = 0x52
	CMD_CONF_SAVE   = 0x53
	CMD_CONF_DELETE = 0x54
	DETECT_REQ      = 0x73
	DETECT_RESP     = 0x46
	RADIO_STATE_OFF = 0x00
	RADIO_STATE_ON	= 0x01
	RADIO_STATE_ASK = 0xFF

	CMD_ERROR		    = 0x90
	ERROR_INITRADIO     = 0x01
	ERROR_TXFAILED	    = 0x02
	ERROR_EEPROM_LOCKED	= 0x03

	@staticmethod
	def escape(data):
		data = data.replace(bytes([0xdb]), bytes([0xdb, 0xdd]))
		data = data.replace(bytes([0xc0]), bytes([0xdb, 0xdc]))
		return data

class ROM():
	PRODUCT_RNODE  = chr(0x03)
	MODEL_A4       = chr(0xA4)
	MODEL_A9       = chr(0xA9)

	ADDR_PRODUCT   = chr(0x00)
	ADDR_MODEL     = chr(0x01)
	ADDR_HW_REV    = chr(0x02)
	ADDR_SERIAL    = chr(0x03)
	ADDR_MADE	   = chr(0x07)
	ADDR_CHKSUM	   = chr(0x0B)
	ADDR_SIGNATURE = chr(0x1B)
	ADDR_INFO_LOCK = chr(0x9B)
	ADDR_CONF_SF   = chr(0x9C)
	ADDR_CONF_CR   = chr(0x9D)
	ADDR_CONF_TXP  = chr(0x9E)
	ADDR_CONF_BW   = chr(0x9F)
	ADDR_CONF_FREQ = chr(0xA3)
	ADDR_CONF_OK   = chr(0xA7)

	INFO_LOCK_BYTE = chr(0x73)
	CONF_OK_BYTE   = chr(0x73)

class RNode():
	def __init__(self, serial_instance):
		self.serial = serial_instance
		self.timeout     = 100

		self.r_frequency = None
		self.r_bandwidth = None
		self.r_txpower   = None
		self.r_sf        = None
		self.r_state     = None
		self.r_lock      = None
		self.r_stat_rssi = 0
		self.r_stat_snr  = 0
		self.rssi_offset = 157

		self.sf = None
		self.cr = None
		self.txpower = None
		self.frequency = None
		self.bandwidth = None

		self.detected = None

		self.eeprom = None
		self.major_version = None
		self.minor_version = None
		self.version = None

		self.provisioned = None
		self.product = None
		self.model = None
		self.hw_rev = None
		self.made = None
		self.serialno = None
		self.checksum = None
		self.signature = None
		self.signature_valid = False
		self.vendor = None

		self.min_freq = None
		self.max_freq = None
		self.max_output = None

		self.configured = None
		self.conf_sf = None
		self.conf_cr = None
		self.conf_txpower = None
		self.conf_frequency = None
		self.conf_bandwidth = None

	def readLoop(self):
		try:
			in_frame = False
			escape = False
			command = KISS.CMD_UNKNOWN
			data_buffer = b""
			command_buffer = b""
			last_read_ms = int(time.time()*1000)

			while self.serial.is_open:
				if self.serial.in_waiting:
					byte = ord(self.serial.read(1))
					last_read_ms = int(time.time()*1000)

					if (in_frame and byte == KISS.FEND and command == KISS.CMD_DATA):
						in_frame = False
						self.processIncoming(data_buffer)
						data_buffer = b""
						command_buffer = b""
					elif (in_frame and byte == KISS.FEND and command == KISS.CMD_ROM_READ):
						self.eeprom = data_buffer
						in_frame = False
						data_buffer = b""
						command_buffer = b""
					elif (byte == KISS.FEND):
						in_frame = True
						command = KISS.CMD_UNKNOWN
						data_buffer = b""
						command_buffer = b""
					elif (in_frame and len(data_buffer) < 512):
						if (len(data_buffer) == 0 and command == KISS.CMD_UNKNOWN):
							command = byte
						elif (command == KISS.CMD_DATA):
							if (byte == KISS.FESC):
								escape = True
							else:
								if (escape):
									if (byte == KISS.TFEND):
										byte = KISS.FEND
									if (byte == KISS.TFESC):
										byte = KISS.FESC
									escape = False
								data_buffer = data_buffer+bytes([byte])
						elif (command == KISS.CMD_FREQUENCY):
							if (byte == KISS.FESC):
								escape = True
							else:
								if (escape):
									if (byte == KISS.TFEND):
										byte = KISS.FEND
									if (byte == KISS.TFESC):
										byte = KISS.FESC
									escape = False
								command_buffer = command_buffer+bytes([byte])
								if (len(command_buffer) == 4):
									self.r_frequency = command_buffer[0] << 24 | command_buffer[1] << 16 | command_buffer[2] << 8 | command_buffer[3]
									RNS.log("Radio reporting frequency is "+str(self.r_frequency/1000000.0)+" MHz")
									self.updateBitrate()

						elif (command == KISS.CMD_BANDWIDTH):
							if (byte == KISS.FESC):
								escape = True
							else:
								if (escape):
									if (byte == KISS.TFEND):
										byte = KISS.FEND
									if (byte == KISS.TFESC):
										byte = KISS.FESC
									escape = False
								command_buffer = command_buffer+bytes([byte])
								if (len(command_buffer) == 4):
									self.r_bandwidth = command_buffer[0] << 24 | command_buffer[1] << 16 | command_buffer[2] << 8 | command_buffer[3]
									RNS.log("Radio reporting bandwidth is "+str(self.r_bandwidth/1000.0)+" KHz")
									self.updateBitrate()

						elif (command == KISS.CMD_FW_VERSION):
							if (byte == KISS.FESC):
								escape = True
							else:
								if (escape):
									if (byte == KISS.TFEND):
										byte = KISS.FEND
									if (byte == KISS.TFESC):
										byte = KISS.FESC
									escape = False
								command_buffer = command_buffer+bytes([byte])
								if (len(command_buffer) == 2):
									self.major_version = command_buffer[0]
									self.minor_version = command_buffer[1]
									self.updateVersion()

						elif (command == KISS.CMD_TXPOWER):
							self.r_txpower = byte
							RNS.log("Radio reporting TX power is "+str(self.r_txpower)+" dBm")
						elif (command == KISS.CMD_SF):
							self.r_sf = byte
							RNS.log("Radio reporting spreading factor is "+str(self.r_sf))
							self.updateBitrate()
						elif (command == KISS.CMD_CR):
							self.r_cr = byte
							RNS.log("Radio reporting coding rate is "+str(self.r_cr))
							self.updateBitrate()
						elif (command == KISS.CMD_RADIO_STATE):
							self.r_state = byte
						elif (command == KISS.CMD_RADIO_LOCK):
							self.r_lock = byte
						elif (command == KISS.CMD_ERROR):
							if (byte == KISS.ERROR_INITRADIO):
								RNS.log(str(self)+" hardware initialisation error (code "+RNS.hexrep(byte)+")")
							elif (byte == KISS.ERROR_INITRADIO):
								RNS.log(str(self)+" hardware TX error (code "+RNS.hexrep(byte)+")")
							else:
								RNS.log(str(self)+" hardware error (code "+RNS.hexrep(byte)+")")
						elif (command == KISS.CMD_DETECT):
							if byte == KISS.DETECT_RESP:
								self.detected = True
							else:
								self.detected = False
						elif (command == KISS.CMD_STAT_RSSI):
							self.r_stat_rssi = byte - self.rssi_offset
						elif (command == KISS.CMD_STAT_SNR):
							self.r_stat_snr = int.from_bytes(bytes([byte]), byteorder="big", signed=True) * 0.25
						
				else:
					time_since_last = int(time.time()*1000) - last_read_ms
					if len(data_buffer) > 0 and time_since_last > self.timeout:
						RNS.log(str(self)+" serial read timeout")
						data_buffer = b""
						in_frame = False
						command = KISS.CMD_UNKNOWN
						escape = False
					sleep(0.08)

		except Exception as e:
			RNS.log("Error while reading from serial port")
			traceback.print_exc()
			exit()

	def processIncoming(self, data):
		self.callback(data, self)

	def updateBitrate(self):
		try:
			self.bitrate = self.sf * ( (4.0/self.cr) / (math.pow(2,self.sf)/(self.bandwidth/1000)) ) * 1000
			self.bitrate_kbps = round(self.bitrate/1000.0, 2)
		except:
			self.bitrate = 0

	def updateVersion(self):
		minstr = str(self.minor_version)
		if len(minstr) == 1:
			minstr = "0"+minstr
		self.version = str(self.major_version)+"."+minstr

	def detect(self):
		kiss_command = bytes([KISS.FEND, KISS.CMD_DETECT, KISS.DETECT_REQ, KISS.FEND, KISS.CMD_FW_VERSION, 0x00, KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring spreading factor for "+self(str))

	def initRadio(self):
		self.setFrequency()
		self.setBandwidth()
		self.setTXPower()
		self.setSpreadingFactor()
		self.setCodingRate()
		self.setRadioState(KISS.RADIO_STATE_ON)

	def setFrequency(self):
		c1 = self.frequency >> 24
		c2 = self.frequency >> 16 & 0xFF
		c3 = self.frequency >> 8 & 0xFF
		c4 = self.frequency & 0xFF
		data = KISS.escape(bytes([c1])+bytes([c2])+bytes([c3])+bytes([c4]))

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_FREQUENCY])+data+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring frequency for "+self(str))

	def setBandwidth(self):
		c1 = self.bandwidth >> 24
		c2 = self.bandwidth >> 16 & 0xFF
		c3 = self.bandwidth >> 8 & 0xFF
		c4 = self.bandwidth & 0xFF
		data = KISS.escape(bytes([c1])+bytes([c2])+bytes([c3])+bytes([c4]))

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_BANDWIDTH])+data+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring bandwidth for "+self(str))

	def setTXPower(self):
		txp = bytes([self.txpower])

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_TXPOWER])+txp+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring TX power for "+self(str))

	def setSpreadingFactor(self):
		sf = bytes([self.sf])

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_SF])+sf+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring spreading factor for "+self(str))

	def setCodingRate(self):
		cr = bytes([self.cr])

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_CR])+cr+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring coding rate for "+self(str))

	def setRadioState(self, state):
		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_RADIO_STATE])+bytes([state])+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring radio state for "+self(str))

	def setPromiscuousMode(self, state):
		if state == True:
			kiss_command = bytes([KISS.FEND, KISS.CMD_PROMISC, 0x01, KISS.FEND])
		else:
			kiss_command = bytes([KISS.FEND, KISS.CMD_PROMISC, 0x00, KISS.FEND])

		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring promiscuous mode for "+self(str))

def device_probe(rnode):
	sleep(2.5)
	rnode.detect()
	sleep(0.1)
	if rnode.detected == True:
		RNS.log("RNode connected")
		RNS.log("Firmware version: "+rnode.version)
		return True
	else:
		raise IOError("Got invalid response while detecting device")

def packet_captured(data, rnode_instance):
	if rnode_instance.console_output:
		RNS.log("["+str(rnode_instance.r_stat_rssi)+" dBm] [SNR "+str(rnode_instance.r_stat_snr)+" dB] ["+str(len(data))+" bytes]\t"+str(data));
	if rnode_instance.write_to_disk:
		try:
			filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S.%f")+".pkt"
			file = open(rnode_instance.write_dir+"/"+filename, "w")
			file.write(data)
			file.close()
		except Exception as e:
			RNS.log("Error while writing packet to disk")
			os._exit(255)


def main():
	try:
		if not util.find_spec("serial"):
			raise ImportError("Serial module could not be found")
	except ImportError:
		print("")
		print("RNode Config Utility needs pyserial to work.")
		print("You can install it with: pip3 install pyserial")
		print("")
		exit()

	import serial

	try:
		parser = argparse.ArgumentParser(description="LoRa packet sniffer for RNode hardware.")
		parser.add_argument("-C", "--console", action="store_true", help="Print captured packets to the console")
		parser.add_argument("-W", action="store", metavar="directory", type=str, default=None, help="Write captured packets to a directory")
		parser.add_argument("--freq", action="store", metavar="Hz", type=int, default=None, help="Frequency in Hz")
		parser.add_argument("--bw", action="store", metavar="Hz", type=int, default=None, help="Bandwidth in Hz")
		parser.add_argument("--txp", action="store", metavar="dBm", type=int, default=None, help="TX power in dBm")
		parser.add_argument("--sf", action="store", metavar="factor", type=int, default=None, help="Spreading factor")
		parser.add_argument("--cr", action="store", metavar="rate", type=int, default=None, help="Coding rate")

		parser.add_argument("port", nargs="?", default=None, help="Serial port where RNode is attached", type=str)
		args = parser.parse_args()

		console_output = False
		write_to_disk = False
		write_dir = None

		if args.console:
			console_output = True

		if args.W:
			if not os.path.isdir(args.W):
				try:
					os.mkdir(args.W)
					write_to_disk = True
					write_dir = args.W
				except Exception as e:
					RNS.log("Could not open or create specified directory")
			else:
				write_to_disk = True
				write_dir = args.W

		if args.port:
			RNS.log("Opening serial port "+args.port+"...")
			rnode = None
			rnode_serial = None
			rnode_baudrate = 115200

			try:
				rnode_serial = serial.Serial(
					port = args.port,
					baudrate = rnode_baudrate,
					bytesize = 8,
					parity = serial.PARITY_NONE,
					stopbits = 1,
					xonxoff = False,
					rtscts = False,
					timeout = 0,
					inter_byte_timeout = None,
					write_timeout = None,
					dsrdtr = False
				)
			except Exception as e:
				RNS.log("Could not open the specified serial port. The contained exception was:")
				RNS.log(str(e))
				exit()

			rnode = RNode(rnode_serial)
			rnode.callback = packet_captured
			rnode.console_output = console_output
			rnode.write_to_disk = write_to_disk
			rnode.write_dir = write_dir

			thread = threading.Thread(target=rnode.readLoop)
			thread.setDaemon(True)
			thread.start()

			try:
				device_probe(rnode)
			except Exception as e:
				RNS.log("Serial port opened, but RNode did not respond.")
				print(e)
				exit()

			if not (args.freq and args.bw and args.sf and args.cr):
				RNS.log("Please input startup configuration:")
				print("")

			if args.freq:
				rnode.frequency = args.freq
			else:
				print("Frequency in Hz:\t", end=' ')
				rnode.frequency = int(input())


			if args.bw:
				rnode.bandwidth = args.bw
			else:
				print("Bandwidth in Hz:\t", end=' ')
				rnode.bandwidth = int(input())

			if args.txp:
				rnode.txpower = args.txp
			else:
				rnode.txpower = 2

			if args.sf:
				rnode.sf = args.sf
			else:
				print("Spreading factor:\t", end=' ')
				rnode.sf = int(input())

			if args.cr:
				rnode.cr = args.cr
			else:
				print("Coding rate:\t\t", end=' ')
				rnode.cr = int(input())

			rnode.initRadio()
			rnode.setPromiscuousMode(True)
			sleep(0.5)
			RNS.log("RNode in LoRa promiscuous mode and listening")

			if not args.W and not args.console:
				RNS.log("Warning! No output destination specified! You won't see any captured packets.")

			while True:
				input()
		else:
			print("")
			parser.print_help()
			print("")
			exit()

	except KeyboardInterrupt:
		print("")
		exit()

if __name__ == "__main__":
	main()
