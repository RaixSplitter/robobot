import time
from datetime import datetime
from threading import Thread

import cv2
import numpy as np


class SCam:
	cap = None
	th = None  # Thread instance
	savedFrame = None  # Stores the last valid frame
	frameTime = datetime.now()
	getFrame = True
	cnt = 0
	useCam = True
	imageFailCnt = 0

	def setup(self):
		"""Initialize the camera and start the thread."""
		if self.useCam:
			from uservice import service
			self.cap = cv2.VideoCapture(f'http://{service.host}:7123/stream.mjpg')

			if self.cap.isOpened():
				self.th = Thread(target=self.run, daemon=True)  # Corrected thread reference
				self.th.start()
				print("% SCam:: Camera started")
			else:
				print("% SCam:: Camera failed to open")
		else:
			print("% SCam:: Camera disabled (in scam.py)")

	def getImage(self):
		"""Retrieve the last captured frame."""
		fail = False

		if not self.useCam:
			if self.imageFailCnt == 0:
				print("% SCam:: not using cam")
			fail = True
		elif not self.cap or not self.cap.isOpened():
			if self.imageFailCnt == 0:
				print("% SCam:: could not open")
			fail = True
		else:
			from uservice import service
			self.getFrame = True
			cnt = 0  # timeout counter
			while self.getFrame and cnt < 100 and not service.stop:
				time.sleep(0.01)
				cnt += 1
			fail = self.getFrame  # True if it timed out

		if fail:
			self.imageFailCnt += 1
			return False, self.savedFrame, self.frameTime
		else:
			self.imageFailCnt = 0
			return True, self.savedFrame, self.frameTime

	def run(self):
		"""Capture frames continuously in a separate thread."""
		from uservice import service
		first = True

		while self.cap and self.cap.isOpened() and not service.stop:
			if self.getFrame or first:
				try:
					ret, frame = self.cap.read()
					if ret:
						self.savedFrame = frame
						self.frameTime = datetime.now()
						self.getFrame = False
						self.cnt += 1

						if first:
							first = False
							h, w, ch = self.savedFrame.shape
							print(f"% Camera available: size ({h}x{w}, {ch} channels)")
					else:
						print("% Failed to receive frame (stream end?). Exiting ...")
						break  # Exit loop if the stream fails
				except Exception as e:
					print(f"% Error while reading frame: {e}")
					break  # Exit loop on error
			else:
				self.cap.read()  # Discard unused frames

		print("% Camera thread stopped")

	def terminate(self):
		"""Gracefully stop the camera and release resources."""
		from uservice import service
		service.stop = True  # Ensure the thread exits

		if self.th and self.th.is_alive():
			self.th.join(timeout=1)  # Ensure the thread properly stops

		if self.cap and isinstance(self.cap, cv2.VideoCapture):
			self.cap.release()
			print("% Camera released successfully")
		else:
			print("% Camera stream was not open")

		cv2.destroyAllWindows()
		print("% Camera terminated")

cam = SCam()
