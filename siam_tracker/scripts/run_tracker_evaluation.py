#!/usr/bin/python
from __future__ import division
import sys
import os
import cv2
import numpy as np
from PIL import Image
import src.siamese as siam
from src.tracker import tracker
from src.parse_arguments import parse_arguments
from src.region_to_bbox import region_to_bbox
from src.click_and_crop import click_and_crop  # Bounding Box Selection
import rospy

def main():
    # Avoid printing TF debugging information
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

    # --- Parse arguments from JSON file ---
    hp, evaluation, run, env, design = parse_arguments()
    final_score_sz = hp.response_up * (design.score_sz - 1) + 1
    image, templates_z, scores = siam.build_tracking_graph(final_score_sz, design, env)

    # --- Start Streaming from Video ---
    cap = cv2.VideoCapture(env.root_sequences + '/' + sys.argv[1] + '.mp4')
    ret, frame = cap.read()
    if(not ret):
        print "Error opening video sequence"

    # --- Save Video (Optional) ---
    if run.save_video:
        vid_write = cv2.VideoWriter(env.root_sequences + '/' + sys.argv[1] + '_out.avi',
                                    cv2.VideoWriter_fourcc(*'MJPG'), 25, (frame.shape[1], frame.shape[0]), True)

    # --- Define Initial Bounding Box ---
    BB = click_and_crop(frame, design.window_name)

    cv2.namedWindow(design.window_name)
    cv2.startWindowThread()
    cv2.setMouseCallback(design.window_name, BB.callback)

    cv2.imshow(design.window_name, frame)
    cv2.waitKey(0)

    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # ----- Define Initial Bounding Box Params & Template -----
    pos_x = int((BB.refPt[0][0] + BB.refPt[1][0]) / 2)  # Template Center
    pos_y = int((BB.refPt[0][1] + BB.refPt[1][1]) / 2)  # Template Center
    target_w = int(abs(BB.refPt[1][0] - BB.refPt[0][0]))  # Template Width / 2
    target_h = int(abs(BB.refPt[1][1] - BB.refPt[0][1]))  # Template Height / 2

    # ----- Beging Tracking -----
    tracker(hp, run, design, pos_x, pos_y, target_w, target_h, final_score_sz, templates_z, scores, cap, vid_write, frame)

    cap.release()
    cv2.destroyAllWindows()

    if run.save_video:
        vid_write.release()


if __name__ == '__main__':
    sys.exit(main())
