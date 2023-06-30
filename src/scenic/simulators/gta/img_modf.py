"""
This file has basic image modification functions
"""

from itertools import product

from PIL import Image
import cv2
import numpy as np


def convert_black_white(img_data=None, img_file=None, threshold=100):
    assert img_data is not None or img_file is not None
    if img_data is None:
        img_data = Image.open(img_file)

    img_copy = img_data.copy()
    pixels = img_copy.load()

    for j, k in product(range(img_copy.size[0]), range(img_copy.size[1])):
        if (np.array(pixels[j, k][0:3]) > threshold).any():
            pixels[j, k] = (255, 255, 255, 255)
        else:
            pixels[j, k] = (0, 0, 0, 255)

    return img_copy


def get_edges(img_data=None, img_file=None, threshold=100, kernelsize=1):
    assert img_data is not None or img_file is not None
    if img_data is None:
        img_data = Image.open(img_file)

    img_copy = img_data.copy()

    # Get the black and white image
    img_bw = convert_black_white(
        img_data=img_copy, img_file=img_file, threshold=threshold
    )
    cv_bw = cv2.cvtColor(np.array(img_bw), cv2.COLOR_RGB2BGR)
    # Detect edges using Laplacian
    laplacian = cv2.Laplacian(cv_bw, cv2.CV_8U, ksize=kernelsize)

    # Convert back to Pillow image
    pil_lap = Image.fromarray(laplacian)

    # For computing Voronoi images, we need to squeeze the RGB data to 0s and 1s
    pil_squeezed = pil_lap.convert("L")
    pil_squeezed_01 = pil_squeezed.point(lambda x: 0 if x < 128 else 255, "1")
    return pil_squeezed_01
