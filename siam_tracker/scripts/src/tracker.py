import tensorflow as tf

print('Using Tensorflow ' + tf.__version__)
import matplotlib.pyplot as plt
import sys
# sys.path.append('../')
import os
import csv
import cv2

import numpy as np
from PIL import Image
import time

import src.siamese as siam
from src.visualization import show_frame, show_crops, show_scores

# ROS Libraries
import rospy
from geometry_msgs.msg import Point


# gpu_device = 2
# os.environ['CUDA_VISIBLE_DEVICES'] = '{}'.format(gpu_device)

# read default parameters and override with custom ones
def tracker(hp, run, design,
            pos_x, pos_y, target_w, target_h,
            final_score_sz, templates_z, scores,
            cap, vid_write, start_frame, stream_path, e2s):

    scale_factors = hp.scale_step ** np.linspace(-np.ceil(hp.scale_num / 2), np.ceil(hp.scale_num / 2), hp.scale_num)
    # cosine window to penalize large displacements
    hann_1d = np.expand_dims(np.hanning(final_score_sz), axis=0)
    penalty = np.transpose(hann_1d) * hann_1d
    penalty = penalty / np.sum(penalty)

    context = design.context * (target_w + target_h)
    z_sz = np.sqrt(np.prod((target_w + context) * (target_h + context)))
    x_sz = float(design.search_sz) / design.exemplar_sz * z_sz

    # thresholds to saturate patches shrinking/growing
    min_z = hp.scale_min * z_sz
    max_z = hp.scale_max * z_sz
    min_x = hp.scale_min * x_sz
    max_x = hp.scale_max * x_sz

    run_opts = {}

    # with tf.Session(config=tf.ConfigProto(log_device_placement=True)) as sess:
    with tf.Session() as sess:
        tf.global_variables_initializer().run()

        # Initial Bounding Box from User Input
        bbox = pos_x - target_w / 2, pos_y - target_h / 2, target_w, target_h
        templates_z_ = sess.run([templates_z], feed_dict={
            siam.pos_x_ph: pos_x,
            siam.pos_y_ph: pos_y,
            siam.z_sz_ph: z_sz,
            siam.image: start_frame})

        new_templates_z_ = templates_z_  # Exemplars x3 (same size)
        num_frame = 0

        # Initialize ROS Publisher
        pub = rospy.Publisher('bbox', Point, queue_size=10)
        rospy.init_node('tracker', anonymous=True)

        # Restart streaming for online tracking
        cap.release()
        cap = cv2.VideoCapture(stream_path)
        start_frame = cap.get(cv2.CAP_PROP_FRAME_COUNT)  # Start at last frame
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame - 10)

        # ================================================
        while (cap.isOpened()):

            t_start = time.time()

            ret, frame = cap.read()
            if ret:
                frame = e2s.project(frame)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Native format is BGR
                num_frame += 1
                start_frame += 1
            else:
                cap.release()
                cap = cv2.VideoCapture(stream_path)
                cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame - 10)
                cv2.waitKey(1)

            # --- Rescale Exemplar and Search Window ---
            scaled_exemplar = z_sz * scale_factors  # Exemplars x3 (scaled)
            scaled_search_area = x_sz * scale_factors  # Search Areas x3 (scaled)
            scaled_target_w = target_w * scale_factors
            scaled_target_h = target_h * scale_factors

            # --- Feed into Network ---
            scores_ = sess.run([scores], feed_dict={
                siam.pos_x_ph: pos_x,
                siam.pos_y_ph: pos_y,
                siam.x_sz0_ph: scaled_search_area[0],
                siam.x_sz1_ph: scaled_search_area[1],
                siam.x_sz2_ph: scaled_search_area[2],
                templates_z: np.squeeze(templates_z_),
                siam.image: frame,
            }, **run_opts)
            scores_ = np.squeeze(scores_)

            # Penalize change of scale (hyperparams)
            scores_[0, :, :] = hp.scale_penalty * scores_[0, :, :]
            scores_[2, :, :] = hp.scale_penalty * scores_[2, :, :]

            # Find scale with highest peak (after penalty)
            new_scale_id = np.argmax(np.amax(scores_, axis=(1, 2)))

            # Update scaled sizes
            x_sz = (1 - hp.scale_lr) * x_sz + hp.scale_lr * scaled_search_area[new_scale_id]
            target_w = (1 - hp.scale_lr) * target_w + hp.scale_lr * scaled_target_w[new_scale_id]
            target_h = (1 - hp.scale_lr) * target_h + hp.scale_lr * scaled_target_h[new_scale_id]

            # Select response with new_scale_id
            score_ = scores_[new_scale_id, :, :]
            score_ = score_ - np.min(score_)
            score_ = score_ / np.sum(score_)

            # Apply displacement penalty (cosine window)
            score_ = (1 - hp.window_influence) * score_ + hp.window_influence * penalty
            pos_x, pos_y = _update_target_position(pos_x, pos_y, score_, final_score_sz, design.tot_stride,
                                                   design.search_sz, hp.response_up, x_sz)
            # convert <cx,cy,w,h> to <x,y,w,h> and save output
            bbox = pos_x - target_w / 2, pos_y - target_h / 2, target_w, target_h

            # Update the target representation with a rolling average --> MAKE OPTIONAL (?)
            if hp.z_lr > 0:
                new_templates_z_ = sess.run([templates_z], feed_dict={
                    siam.pos_x_ph: pos_x,
                    siam.pos_y_ph: pos_y,
                    siam.z_sz_ph: z_sz,
                    siam.image: frame
                })

                templates_z_ = (1 - hp.z_lr) * np.asarray(templates_z_) + hp.z_lr * np.asarray(new_templates_z_)

            # Update template patch size
            z_sz = (1 - hp.scale_lr) * z_sz + hp.scale_lr * scaled_exemplar[new_scale_id]

            if run.visualization:
            #    show_frame(frame, bbox, 1)

                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.rectangle(frame, (int(bbox[0]),
                                       int(bbox[1])),
                                      (int(bbox[0] + bbox[2]),
                                       int(bbox[1] + bbox[3])),
                              (0, 255, 0), 2)
                cv2.imshow(design.window_name, frame)
                cv2.waitKey(1)

                if run.save_video:
                    vid_write.write(frame)

            t_elapsed = time.time() - t_start
            print("Time Elapsed: ", t_elapsed)


            if(not rospy.is_shutdown()):
                pub.publish(Point(bbox[0],bbox[1],bbox[2])) # (x, y, w)

            # print("Bounding Box: ", bbox)

    plt.close('all')


def _update_target_position(pos_x, pos_y, score, final_score_sz, tot_stride, search_sz, response_up, x_sz):
    # find location of score maximizer
    p = np.asarray(np.unravel_index(np.argmax(score), np.shape(score)))
    # displacement from the center in search area final representation ...
    center = float(final_score_sz - 1) / 2
    disp_in_area = p - center
    # displacement from the center in instance crop
    disp_in_xcrop = disp_in_area * float(tot_stride) / response_up
    # displacement from the center in instance crop (in frame coordinates)
    disp_in_frame = disp_in_xcrop * x_sz / search_sz
    # *position* within frame in frame coordinates
    pos_y, pos_x = pos_y + disp_in_frame[0], pos_x + disp_in_frame[1]
    return pos_x, pos_y
