'''
This file contains helper tools
'''

from collections import namedtuple
import numpy as np
import cv2

bb = namedtuple('bb', ['lr', 'tr', 'tl', 'll'])
scale = namedtuple('scale', ['front', 'back'])
fgObj = namedtuple('fgObj', 'x y fgId')

unitBox = bb([1, 0], [1, 1], [0, 1], [0, 0])
bb2array = lambda bbox: np.array([r for r in bbox])

def unit2bbH(boundingBox, ldBox=unitBox):
    '''Tranform 2d box-sample into trapezoid (car displacement area)
    '''

    bbox = bb2array(boundingBox)
    ubox = bb2array(ldBox)
    h, _ = cv2.findHomography(np.float_(ubox), np.float_(np.array(bbox)))

    return h


def ld2bbSample(sample, h):
    sample = np.float32([sample]).reshape(-1, 1, 2)
    con = cv2.perspectiveTransform(sample, h)
    return np.array(list(con[0][0]))
