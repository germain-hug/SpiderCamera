import numpy as np
import cv2
import time

# Garmin VIRB 360 URL
url = "rtsp://192.168.0.1/livePreviewStream"

cap = cv2.VideoCapture(url)
print("Opened Stream")
#time.sleep(5)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    cv2.waitKey(100)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

