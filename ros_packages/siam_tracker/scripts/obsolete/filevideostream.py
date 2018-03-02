# import the necessary packages
from threading import Thread
import sys
import cv2
import time

# import the Queue class from Python 3
if sys.version_info >= (3, 0):
	from queue import Queue

# otherwise, import the Queue class for Python 2.7
else:
	from Queue import Queue

class FileVideoStream:
	def __init__(self, path, queueSize=128):
		# initialize the file video stream along with the boolean
		# used to indicate if the thread should be stopped or not
		self.stream = cv2.VideoCapture(path)
		#self.count = self.stream.get(cv2.CAP_PROP_FRAME_COUNT)
		#self.stream.set(cv2.CAP_PROP_POS_FRAMES,self.count-10)
		self.stopped = False

		# initialize the queue used to store frames read from
		# the video file
		self.Q = Queue(maxsize=queueSize)

	def start(self):
		# start a thread to read frames from the file video stream
		t = Thread(target=self.update, args=())
		t.daemon = True
		t.start()
		return self

	def update(self):
		ret, frame = self.stream.read()
		if(ret):
			pos_frame = self.stream.get(cv2.CAP_PROP_POS_FRAMES)
			self.Q.put(frame)

		else:
			# Reached end of stream, wait for new images 
			count = self.stream.get(cv2.CAP_PROP_FRAME_COUNT)
			self.stream.set(cv2.CAP_PROP_POS_FRAMES,count-10)
			time.sleep(0.1)

	def read(self):
		# return next frame in the queue
		return self.Q.get()

	def more(self):
		# return True if there are still frames in the queue
		return self.Q.qsize() > 0
