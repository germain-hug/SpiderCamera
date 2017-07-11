import numpy as np
import cv2
import time

# Garmin VIRB 360 URL
#url = "rtsp://192.168.0.1/livePreviewStream"

cap = cv2.VideoCapture("../../../../stream.flv")
print("Opened Stream, waiting for frames...")

# Start at last frame
count = cap.get(cv2.CAP_PROP_FRAME_COUNT);
cap.set(cv2.CAP_PROP_POS_FRAMES,count-10);

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    cv2.waitKey(25)
    # Display the resulting frame
    if(ret):
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


print("End")
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

