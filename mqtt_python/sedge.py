from datetime import *
import numpy as np
import time as t
import cv2

np.set_printoptions(precision=2)

class SEdge:
	# raw AD values
	edge = np.zeros(8, dtype=np.int32)
	edgeUpdCnt = 0
	edgeTime = datetime.now()
	edgeInterval = 0
	# normalizing white values
	edge_n_w = np.zeros(8, dtype=np.int32)
	edge_n_wUpdCnt = 0
	edge_n_wTime = datetime.now()
	# normalized after white calibration
	edge_n = np.zeros(8, dtype=np.int32)
	edge_nUpdCnt = 0
	edge_nTime = datetime.now()
	edge_update_interval = 0.0 # how many seconds between the edge sensor messages

	# line detection levels
	line_valid_threshold = 550 # 1000 is calibrated white
	line_crossroad_valid_distance = 4 # How many sensors need to see line for it to be a crossroad
	low_threshold = line_valid_threshold - 100

	# line detection values
	target_position = 0.0
	position_on_line = 0.0
	last_valid_position = 0.0
	on_line = False
	on_crossroad = False
	do_line_control = False

	sendCalibRequest = False

	# PID loop, perfected values for speed = 0.2
	Kp = 0.0
	Ki = 0.0
	Kd = 0.0
	max_windup = 1
	total_error = 0.0
	previous_error = 0.0

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

		# Subscribe to raw line sensor messages: https://rsewiki.electro.dtu.dk/index.php?title=Help_page_Teensy_8#Line_sensor
		# service.send(service.topicCmd + "T0/sub","liv 1")
		# service.send(service.topicCmd + "T0/sub","liw 1")

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



	##########################################################

	def decode(self, topic, msg):
		# decode MQTT message
		used = True

		# Only used for calibration purposes, never needed really - and not subscribed to 
		if topic == "T0/liv": # raw AD value
			from uservice import service
			gg = msg.split(" ")
			if (len(gg) >= 4):
				# NOTE: Ocasionally the raw sensor values are all 0, probably because the sampling freq is lower than the publishing freq
				if all([int(v) == 0 for v in gg[1:9]]):
					return True

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

		# Only used for calibration purposes, never needed really - and not subscribed to 
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
		else:
			used = False
		return used

	##########################################################

	def plot_line(self):
		"""
		Stream video to X11. Remember to run ssh with x-port forwarding
			ssh -X local@10.197.218.176
		"""
		# self.paint()
		sensor_data = np.array(self.edge_n)

		if np.all(sensor_data == 0):
			return
		n = len(self.edge_n)

		# Normalize the data to 8-bit range (0-255)
		sensor_data_norm = ((sensor_data - sensor_data.min()) / (sensor_data.max() - sensor_data.min()) * 255).astype(np.uint8)

		# If we arent on line just make it all zero
		if np.all(sensor_data < self.line_valid_threshold):
			sensor_data_norm = np.zeros_like(sensor_data, dtype=np.uint8)

		# Resize for better visualization
		scale_factor = 20  # Scale up for visibility
		sensor_image = cv2.resize(sensor_data_norm, (scale_factor, n*scale_factor), interpolation=cv2.INTER_NEAREST)

		cv2.imshow("Edge sensor", sensor_image.T)
		cv2.waitKey(1)


	def detect_line(self):
		""" Calculate self.position_on_line, which will be between -3.5 .. 3.5 """		
		high = self.edge_n.max()
		average = self.edge_n.mean()

		# detect if we are at a crossroad
		# new cross detector: if the width is larger than self.line_crossroad_valid_distance
		high_list = [i for i,v in enumerate(self.edge_n) if v >= 300]
		if len(high_list) > 0:
			first_detector = high_list[0]
			last_detector  = high_list[-1]
			self.on_crossroad = (last_detector - first_detector) >= self.line_crossroad_valid_distance
		else:
			self.on_crossroad = False
		# is line valid (high above threshold)

		self.on_line = self.line_valid_threshold <= high
		# print(f"high: {self.high}, threshold: {self.lineValidThreshold}, over threshold: {self.lineValid}")

		# find line position
		# using COG method for values above a threshold
		posSum = 0
		summ = 0
		for i in range(8):
			# everything more black than 'low' is ignored
			v = self.edge_n[i] - self.low_threshold
			if v > 0:
				summ += v
				posSum += (i+1) * v

		if summ > 0 and self.on_line:
			self.position_on_line = posSum/summ - 4.5
			self.last_valid_position = self.position_on_line  # Store last valid position
		else:
			# Use last valid position instead of defaulting to 0
			# This will make the robot continue turning in the direction it was heading
			self.position_on_line = self.last_valid_position

		# print(f"low: {self.low}, Crossing threshold: {self.crossingThreshold}, Crossing: {self.crossingLine}, Line valid threshold: {self.lineValidThreshold}, Edges: {self.edge_n}, high: {high}, validline: {self.lineValid}, avg: {average:.2f}")


	def set_line_control_targets(self, target_velocity: float, target_position: float) -> None:
		"""
		Function called in the loop of mqtt-client to determine if we should follow line
		Target position is a float determining the position on the line to keep in interval -2..2
		""" 
		self.velocity = target_velocity
		self.target_position = target_position		
		self.do_line_control = 0.0 != target_velocity


	def pid_loop(self, dt: float, target: float, current:float) -> float:
		""" Do pid control for a float, that is 0 for straight on line -1 for full left and 1 for full right """
		error =  target - current # error is between -3.5 and 3.5
		self.total_error += error
		differential_error = error - self.previous_error
		self.previous_error = error

		# constrain the integral windup because there might be some undesirable
		# behavior otherwise
		self.total_error = min([self.total_error, self.max_windup])
		self.total_error = max([self.total_error, -self.max_windup])

		p_term = self.Kp * error
		i_term = self.Ki * self.total_error * dt
		d_term = self.Kd * differential_error / dt

		out = p_term + i_term + d_term
		# print(f"pos on line: {current:.2f}, u: {out:.2f} | p: {p_term:.2f}, i: {i_term:.2f}, d: {d_term:.2f}")
		return out


	def follow_line(self):
		""" Function called in loop to give follow line commands """
		from uservice import service

		dt = self.edge_update_interval / 1000.0 # seconds

		normalized_position = self.position_on_line / 3.5  # normalizefrom -1 to 1
		u = self.pid_loop(dt, self.target_position, normalized_position)

		command = f"{self.velocity:.3f} {u:.3f} {t.time()}"
		service.send(self.topicRc, command) # send new turn command, maintaining velocity


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
