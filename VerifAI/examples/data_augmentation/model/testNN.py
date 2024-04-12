import sys, getopt
import cv2
import tensorflow as tf

from model.model import Model
import matplotlib.image as mpimg

GRAPH_PATH = './data/checkpoint/car-detector-model.meta'
CHECKPOINT_PATH = './data/checkpoint/'
IMAGE_PATH = './data/test/test.jpg'


with tf.Session() as sess:
    nn = Model()
    nn.init(GRAPH_PATH, CHECKPOINT_PATH, sess)
    image = cv2.imread(IMAGE_PATH)
    print(nn.predict(image)[0])
