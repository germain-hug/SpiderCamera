# import the necessary packages
#from imutils.video import FileVideoStream
from filevideostream import FileVideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import time
import cv2

path = "/home/hugo/stream.flv"
fvs = FileVideoStream(path).start()
time.sleep(1.0)

# start the FPS timer
fps = FPS().start()
reboot = False

# loop over frames from the video file stream
while True:

	if(fvs.more() or reboot):
		frame = fvs.read()
		reboot = False
		# display the size of the queue on the frame
		cv2.putText(frame, "Queue Size: {}".format(fvs.Q.qsize()),
			(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)	
 
		# show the frame and update the FPS counter
		cv2.imshow("Frame", frame)
		cv2.waitKey(1)
		fps.update()
	else:
		cv2.waitKey(20)
		reboot = True



# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
cv2.destroyAllWindows()
fvs.stop()