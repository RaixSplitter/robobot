#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */


from datetime import *
import time as t
import cv2

class SEdge:
	# raw AD values
	edge = [0, 0, 0 , 0, 0, 0, 0, 0]
	edgeUpdCnt = 0
	edgeTime = datetime.now()
	edgeInterval = 0
	# normalizing white values
	edge_n_w = [0, 0, 0 , 0, 0, 0, 0, 0]
	edge_n_wUpdCnt = 0
	edge_n_wTime = datetime.now()
	# normalized after white calibration
	edge_n = [0, 0, 0 , 0, 0, 0, 0, 0]
	edge_nUpdCnt = 0
	edge_nTime = datetime.now()
	edge_update_interval = 0.0 # how many seconds between the edge sensor messages

	# line detection levels
	line_valid_threshold = 550 # 1000 is calibrated white
	line_crossroad_threshold = 700 # average above this is assumed to be crossing line
	low_threshold = line_valid_threshold - 100
	
	# line detection values
	target_position = 0.0
	position_on_line = 0.0
	on_line = False
	on_crossroad = False

	sendCalibRequest = False
	do_line_control = False

    # PID loop
	Kp = 0.5
	Ki = 0.0
	Kd = 0.4
	total_error = 0.0
	previous_error = 0.0
	cumulative_error = 0.0

	def setup(self):
		from uservice import service

		sendBlack = False
		loops = 0
		# turn line sensor on (command 'lip 1')
		print("% Edge (sedge.py):: turns on line sensor")
		self.topicLip = service.topicCmd + "T0/lip"
		service.send(self.topicLip, "1")
		# topic for (remote) control
		self.topicRc = service.topicCmd + "ti/rc"
		# request data
		while not service.stop:
			t.sleep(0.02)
			# white calibrate requested
			if service.args.white:
				if not sendBlack:
					# make sure black level is black
					topic = service.topicCmd + "T0/litb"
					param = "0 0 0 0 0 0 0 0"
					sendBlack = service.send(topic, param)

				elif self.edgeUpdCnt < 3:
					# request raw AD reflectivity
					service.send(service.topicCmd + "T0/livi"," ")

				elif not self.sendCalibRequest:
					# send calibration request, averaged over 30 samples
					service.send(service.topicCmd + "T0/liwi","")
					t.sleep(0.02)
					service.send(service.topicCmd + "T0/licw","100")
					# allow communication to settle
					print("# Edge (sedge.py):: sending calibration request")
					# wait for calibration to finish (each sample takes 1-2 ms)
					t.sleep(0.25)
					# save the calibration as new default
					service.send(service.topicCmd + "T0/eew","")
					self.sendCalibRequest = True
					# ask for new white values
					service.send(service.topicCmd + "T0/liwi","")
					t.sleep(0.02)

				else:
					t.sleep(0.25)
					service.args.white = False
					print(f"% Edge (sedge.py):: calibration should be fine, got {self.edge_n_wUpdCnt} updates - terminates")
					# terminate mission
					service.terminate()

			elif self.edge_n_wUpdCnt == 0:
				# get calibrated white value
				service.send(service.topicCmd + "T0/liwi"," ")

			elif self.edge_nUpdCnt == 0:
				# wait for line sensor data
				pass

			else:
				print(f"% Edge (sedge.py):: got data stream; after {loops}")
				break

			loops += 1
			if loops > 30:
				print(f"% Edge (sedge.py):: got no data after {loops} (continues edge_n_wUpdCnt={self.edge_n_wUpdCnt}, edgeUpdCnt={self.edgeUpdCnt}, edge_nUpdCnt={self.edge_nUpdCnt})")
				break
		pass

	##########################################################

	def print(self):
		from uservice import service
		print("% Edge (sedge.py):: " + str(self.edgeTime - service.startTime) +
					f" ({self.edge[0]}, " +
					f"{self.edge[1]}, " +
					f"{self.edge[2]}, " +
					f"{self.edge[3]}, " +
					f"{self.edge[4]}, " +
					f"{self.edge[5]}, " +
					f"{self.edge[6]}, " +
					f"{self.edge[7]})" +
					f" {self.edgeInterval:.2f} ms " +
					str(self.edgeUpdCnt))
	def printn(self):
		from uservice import service
		print("% Edge (sedge.py):: normalized " + str(self.edge_nTime - service.startTime) +
					f" ({self.edge_n[0]}, " +
					f"{self.edge_n[1]}, " +
					f"{self.edge_n[2]}, " +
					f"{self.edge_n[3]}, " +
					f"{self.edge_n[4]}, " +
					f"{self.edge_n[5]}, " +
					f"{self.edge_n[6]}, " +
					f"{self.edge_n[7]})" +
					f" {self.edge_update_interval:.2f} ms " +
					f" {self.position_on_line:.2f} " +
					str(self.edge_nUpdCnt))
	def printnw(self):
		from uservice import service
		print("% Edge (sedge.py):: white level " + str(self.edge_n_wTime) +
					f" ({self.edge_n_w[0]}, " +
					f"{self.edge_n_w[1]}, " +
					f"{self.edge_n_w[2]}, " +
					f"{self.edge_n_w[3]}, " +
					f"{self.edge_n_w[4]}, " +
					f"{self.edge_n_w[5]}, " +
					f"{self.edge_n_w[6]}, " +
					f"{self.edge_n_w[7]}) " +
					str(self.edge_n_wUpdCnt))

	##########################################################

	def decode(self, topic, msg):
		# decode MQTT message
		used = True
		if topic == "T0/liv": # raw AD value
			from uservice import service
			gg = msg.split(" ")
			if (len(gg) >= 4):
				t0 = self.edgeTime
				self.edgeTime = datetime.fromtimestamp(float(gg[0]))
				self.edge[0] = int(gg[1])
				self.edge[1] = int(gg[2])
				self.edge[2] = int(gg[3])
				self.edge[3] = int(gg[4])
				self.edge[4] = int(gg[5])
				self.edge[5] = int(gg[6])
				self.edge[6] = int(gg[7])
				self.edge[7] = int(gg[8])
				t1 = self.edgeTime
				if self.edgeUpdCnt == 2:
					self.edgeInterval = (t1 -t0).total_seconds()*1000
				elif self.edgeUpdCnt > 2:
					self.edgeInterval = (self.edgeInterval * 99 + (t1 -t0).total_seconds()*1000) / 100
				self.edgeUpdCnt += 1

		elif topic == "T0/livn": # normalized after calibration range (0..1000)
			from uservice import service
			gg = msg.split(" ")
			if (len(gg) >= 4):
				t0 = self.edge_nTime
				self.edge_nTime = datetime.fromtimestamp(float(gg[0]))
				self.edge_n[0] = int(gg[1])
				self.edge_n[1] = int(gg[2])
				self.edge_n[2] = int(gg[3])
				self.edge_n[3] = int(gg[4])
				self.edge_n[4] = int(gg[5])
				self.edge_n[5] = int(gg[6])
				self.edge_n[6] = int(gg[7])
				self.edge_n[7] = int(gg[8])
				t1 = self.edge_nTime
				if self.edge_nUpdCnt == 2:
					self.edge_update_interval = (t1-t0).total_seconds()*1000
				elif self.edge_nUpdCnt > 2:
					self.edge_update_interval = (self.edge_update_interval * 99 + (t1-t0).total_seconds()*1000) / 100
				self.edge_nUpdCnt += 1
				# calculate line position - actually center of gravity of white value
				# - missing edge detection
				# got new normalized values
				# debug save as a remark with timestamp
				# flog.writeDataString(f" {msg}");

				self.detect_line()
				# use to control, if active
				if self.do_line_control:
					self.follow_line()

		elif topic == "T0/liw": # get white level
			from uservice import service
			gg = msg.split(" ")
			if (len(gg) >= 4):
				self.edge_n_wTime = datetime.fromtimestamp(float(gg[0]))
				self.edge_n_w[0] = int(gg[1])
				self.edge_n_w[1] = int(gg[2])
				self.edge_n_w[2] = int(gg[3])
				self.edge_n_w[3] = int(gg[4])
				self.edge_n_w[4] = int(gg[5])
				self.edge_n_w[5] = int(gg[6])
				self.edge_n_w[6] = int(gg[7])
				self.edge_n_w[7] = int(gg[8])
				self.edge_n_wUpdCnt += 1
				# self.printnw()
		else:
			used = False
		return used

	##########################################################

	def detect_line(self):
		""" """
		posSum = 0
		
		high = max(self.edge_n)
		average = sum(self.edge_n) / len(self.edge_n)

		# detect if we are at a crossroad
		self.on_crossroad = self.line_crossroad_threshold <= average
		# is line valid (high above threshold)

		self.on_line = self.line_valid_threshold <= high
		# print(f"high: {self.high}, threshold: {self.lineValidThreshold}, over threshold: {self.lineValid}")
		# find line position
		# using COG method for values above a threshold
		summ = 0
		for i in range(8):
			# everything more black than 'low' is ignored
			v = self.edge_n[i] - self.low_threshold
			if v > 0:
				summ += v
				posSum += (i+1) * v
		if summ > 0 and self.on_line:
			self.position_on_line = posSum/summ - 4.5
		else:
			self.position_on_line = 0
		
		# print(self.position_on_line)

		# print(f"low: {self.low}, Crossing threshold: {self.crossingThreshold}, Crossing: {self.crossingLine}, Line valid threshold: {self.lineValidThreshold}, Edges: {self.edge_n}, high: {high}, validline: {self.lineValid}, avg: {average:.2f}")


	def set_line_control_targets(self, target_velocity: float, target_position: float) -> None:
		"""
		Function called in the loop of mqtt-client to determine if we should follow line
		Target position is a float determining the position on the line to keep in interval -2..2
		""" 
		self.velocity = target_velocity
		self.target_position = target_position		
		self.do_line_control = 0.0 != target_velocity


	def pid_loop(self):
		pass


	def follow_line(self):
		""" Function called in loop to give follow line commands """
		from uservice import service

		dt = self.edge_update_interval / 1000.0 # seconds

		error = self.target_position - self.position_on_line
		self.total_error += error

		self.cumulative_error += error * dt

		diff_error = (error - self.previous_error) / dt

		u = self.Kp * error + self.Ki * self.cumulative_error + self.Kd * diff_error
		u = max(min(u, 1), -1) # bound

		self.previous_error = error

		command = f"{self.velocity:.3f} {u:.3f} {t.time()}"
		service.send(self.topicRc, command) # send new turn command, maintaining velocity

		# debug print
		# if self.edge_nUpdCnt % 20 == 0:
		# 	print(f"u: {u:.3f}, p:{self.Kp * error:.3f}, i:{self.Ki * self.pid_i:.3f}, d:{self.Kd * diff_error:.3f}")


	def terminate(self):
		from uservice import service
		self.need_data = False
		print("% Edge (sedge.py):: turn off line sensor")
		service.send(self.topicLip, "0")
		# try:
		#   self.th.join()
		#   # stop subscription service from Teensy
		#   service.send(service.topicCmd + "T0/sub","livn 0")
		# except:
		#   print("% Edge thread not running")
		print("% Edge (sedge.py):: terminated")
		pass

	##########################################################

	def paint(self, img):
		h, w, ch = img.shape
		pl = int(h - h/4) # base position bottom (most positive y)
		st = int(w/10) # distance between sensors
		gh = int(h/2) # graph height
		x = st # base position left
		y = pl
		dtuGreen = (0x35, 0x88, 0) # BGR
		dtuBlue = (0xea, 0x3e, 0x2f)
		dtuRed = (0x00, 0x00, 0x99)
		dtuPurple = (0x8e, 0x23, 0x77)
		# paint baseline
		cv2.line(img, (x,y), (int(x + 7*st), int(y)), dtuGreen, thickness=1, lineType=8)
		# paint calibrated white line (top)
		cv2.line(img, (x,int(y-gh)), (int(x + 7*st), int(y-gh)), dtuGreen, thickness=1, lineType=8)
		# paint threshold line for line valid
		cv2.line(img, (x,int(y-gh*self.line_valid_threshold/1000.0)), (int(x + 7*st), int(y-gh*self.line_valid_threshold/1000.0)), dtuBlue, thickness=1, lineType=4)
		# draw current sensor readings
		for i in range(8):
			y = int(pl - self.edge_n[i]/1000 * gh)
			cv2.drawMarker(img, (x,y), dtuRed, markerType=cv2.MARKER_STAR, thickness=2, line_type=8, markerSize = 10)
			x += st
		# paint line position
		pixP = int((self.position_on_line + 4)*st)
		cv2.line(img, (pixP, int(pl)), (pixP, int(pl-gh)), dtuRed, thickness=3, lineType=4)
		# paint low line position
		pixL = pl - int(gh * self.low_threshold/1000)
		cv2.line(img, (st, pixL), (st*8, pixL), dtuRed, thickness=1, lineType=4)
		# some axis marking
		cv2.putText(img, "Left", (st,pl - 2), cv2.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
		cv2.putText(img, "Right", (int(st+6*st),pl - 2), cv2.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
		cv2.putText(img, "White (1000)", (int(st),pl - gh - 2), cv2.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
		if self.on_crossroad:
			cv2.putText(img, "Crossing", (int(st),int(pl - 20)), cv2.FONT_HERSHEY_PLAIN, 1, dtuRed, thickness=2)


# create the data object
edge = SEdge()
