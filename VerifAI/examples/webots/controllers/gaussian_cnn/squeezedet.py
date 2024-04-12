from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import cv2
import time
import sys
import os
import glob

import numpy as np
import tensorflow as tf


# TOM: hack to import from submodule without __init.py__ (can we fix it?)
import sys
#ROOT_PATH = '/home/tommaso/interact/squeezeDet/'
ROOT_PATH = './'
sys.path.insert(0, ROOT_PATH + 'src')

PROB_THRESH = 0.5

from config import *
from train import _draw_box
from nets import *

FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string('mode', 'image', """'image' or 'video'.""")
#tf.app.flags.DEFINE_string('checkpoint', ROOT_PATH + 'data/model_checkpoints/squeezeDet/model.ckpt-87000',"""Path to the model parameter file.""")
tf.app.flags.DEFINE_string('checkpoint', str('/home/tommaso/Desktop/logs/SqueezeDet/train/model.ckpt-2000'),"""Path to the model parameter file.""")

def init(checkpoint_path):

  with tf.Graph().as_default():

    # Load model
    mc = kitti_squeezeDet_config()
    mc.BATCH_SIZE = 1
    # model parameters will be restored from checkpoint
    mc.LOAD_PRETRAINED_MODEL = False
    model = SqueezeDet(mc, FLAGS.gpu)

    FLAGS.checkpoint = str(checkpoint_path)

    # Start tensorflow session
    saver = tf.train.Saver(model.model_params)
    sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True))
    saver.restore(sess, FLAGS.checkpoint)

    return (sess,mc,model)


def classify(im, conf, prob_thresh):
    '''Classify image'''

    (sess,mc,model) = conf

    im = im.astype(np.float32, copy=False)
    im = cv2.resize(im, (mc.IMAGE_WIDTH, mc.IMAGE_HEIGHT))
    input_image = im - mc.BGR_MEANS

    # Detect
    det_boxes, det_probs, det_class = sess.run(
        [model.det_boxes, model.det_probs, model.det_class],
        feed_dict={model.image_input:[input_image]})

    # Filter
    final_boxes, final_probs, final_class = model.filter_prediction(
     det_boxes[0], det_probs[0], det_class[0])

    #keep_idx    = [idx for idx in range(len(final_probs)) \
    #                   if final_probs[idx] > mc.PLOT_PROB_THRESH]

    keep_idx = [idx for idx in range(len(final_probs)) \
                if final_probs[idx] > prob_thresh]

    final_boxes = [final_boxes[idx] for idx in keep_idx]
    final_probs = [final_probs[idx] for idx in keep_idx]
    final_class = [final_class[idx] for idx in keep_idx]

    # Extract labels + confidence values
    res = []
    for label, confidence, box in zip(final_class, final_probs, final_boxes):
        res.append((label,confidence,box))

    # # Save detections in file
    # cls2clr = {
    #     'car': (255, 191, 0),
    #     'cyclist': (0, 191, 255),
    #     'pedestrian':(255, 0, 191)
    # }
    # # Draw boxes
    # _draw_box(
    #     im, final_boxes,
    #     [mc.CLASS_NAMES[idx]+': (%.2f)'% prob \
    #         for idx, prob in zip(final_class, final_probs)],
    #     cdict=cls2clr,
    # )
    #
    # out_file_name = os.path.join('./', 'out.png')
    # cv2.imwrite(out_file_name, im)
    # print ('Image detection output saved to {}'.format(out_file_name))

    return res




def classify_from_file(im_path, conf, prob_thresh):
    '''Classify image from file'''

    im = cv2.imread(im_path)
    print(type(im))
    print(im.shape)
    return classify(im,conf,prob_thresh)
