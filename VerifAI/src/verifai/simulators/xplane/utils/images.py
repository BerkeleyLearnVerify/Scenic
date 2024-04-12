import os

import numpy as np

try:
    import mss
    import cv2
except ImportError as e:
    raise RuntimeError('recording images requires the mss and cv2 packages') from e

sct = mss.mss()

def grab_image(monitor, resizeTo=None):
    img = cv2.cvtColor(np.array(sct.grab(monitor)), cv2.COLOR_BGRA2BGR)
    if resizeTo is not None:
        return cv2.resize(img, resizeTo)
    return img

def write_video(images, filename='out.avi', fps=10.0):
    height, width, _ = images[0].shape
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    writer = cv2.VideoWriter(filename, fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError('failed to initialize VideoWriter')
    try:
        for image in images:
            writer.write(image)
    finally:
        writer.release()
