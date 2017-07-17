import numpy as np
import cv2
import time

# Garmin VIRB 360 URL
# rtsp://192.168.0.1/livePreviewStream

path = "/home/hugo/stream.flv"
cap = cv2.VideoCapture(path)

while not cap.isOpened():
    cap = cv2.VideoCapture(path)
    cv2.waitKey(1000)

print("Opened Stream")
# Start at last frame
count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
cap.set(cv2.CAP_PROP_POS_FRAMES,count-10)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    cv2.waitKey(1)
    #count = count+1
    #cap.set(cv2.CAP_PROP_POS_FRAMES,count-10)

    # Display the resulting frame
    #if(ret):
    #    cv2.imshow('frame', frame)
    #   if cv2.waitKey(1) & 0xFF == ord('q'):
    #        break

    if(ret):
        cv2.imshow('frame', frame) 
        cv2.waitKey(1)
        pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
        count = count + 1

    else:
        # Reached end of stream, wait for new images 
        #print(pos_frame)
        #ret, frame = cap.read()
        #count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        cap.release()
        cap = cv2.VideoCapture(path)
        cap.set(cv2.CAP_PROP_POS_FRAMES,count-10)
        print(count-10)
        print(cap.get(cv2.CAP_PROP_POS_FRAMES))
        cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()

