'''
This file has basic image modification functions
'''

from PIL import Image
import cv2
from scipy.spatial import Voronoi
from itertools import product
import numpy as np


def convert_black_white(img_data=None, img_file=None, threshold=100):
    assert img_data is not None or img_file is not None
    if img_data is None:
        img_data = Image.open(img_file)

    img_copy = img_data.copy()
    pixels = img_copy.load()

    for j,k in product(range(img_copy.size[0]), range(img_copy.size[1])):
        if (np.array(pixels[j, k][0:3]) > threshold).any():
            pixels[j, k] = (255, 255, 255, 255)
        else:
            pixels[j,k] = (0, 0, 0, 255)

    return img_copy

def get_edges(img_data=None, img_file=None, threshold=100, kernelsize=1):
    assert img_data is not None or img_file is not None
    if img_data is None:
        img_data = Image.open(img_file)

    img_copy = img_data.copy()

    # Get the black and white image
    img_bw = convert_black_white(img_data=img_copy, img_file=img_file,
                                 threshold=threshold)
    cv_bw = cv2.cvtColor(np.array(img_bw), cv2.COLOR_RGB2BGR)
    # Detect edges using Laplacian
    laplacian = cv2.Laplacian(cv_bw, cv2.CV_8U, ksize=kernelsize)

    # Convert back to Pillow image
    pil_lap = Image.fromarray(laplacian)


    # For computing Voronoi images, we need to squeeze the RGB data to 0s and 1s
    pil_squeezed = pil_lap.convert('L')
    pil_squeezed_01 = pil_squeezed.point(lambda x: 0 if x < 128 else 255, '1')
    return pil_squeezed_01

def voronoi_edge(img_data=None, img_file=None, threshold=100, kernelsize=1):
    assert img_data is not None or img_file is not None
    if img_data is None:
        img_data = Image.open(img_file)

    img_copy = img_data.copy()

    # Get 0s and 1s of the edges
    pil_squeezed_01 = get_edges(img_data=img_copy, img_file=img_file,
                                threshold=threshold, kernelsize=kernelsize)

    # Collecting point for Voronoi edge computation
    nz_elements = np.nonzero(np.asarray(pil_squeezed_01))
    points = np.fliplr(np.array(nz_elements).T)
    vor = Voronoi(points)
    vor_x = vor.vertices.T[0]
    vor_y = -vor.vertices.T[1] + img_data.size[1]

    # Convert the black and white image to 0s and 1s
    img_bw = convert_black_white(img_data=img_copy,
                                 img_file=img_file, threshold=threshold)
    img_bw_squeezed = img_bw.convert('L')
    img_bw_01 = img_bw_squeezed.point(lambda x:0 if x< 128 else 255, '1')
    pixels = img_bw_01.load()

    center_x = []
    center_y = []
    for x, y in zip(vor_x, vor_y):
        if 0 < x and x < img_data.size[0] and 0 < y and y < img_data.size[1] \
                and pixels[int(x), img_data.size[1]-1 -int(y)] == 0:
            center_x.append(int(x))
            center_y.append(int(y))

    return {'edge_image':pil_squeezed_01, 'vor_center_x': center_x,
            'vor_center_y': center_y}


def plot_voronoi_plot(img_data=None, img_file=None, threshold=100, kernelsize=3,
    plot_name=None):
    import matplotlib.pyplot as plt
    assert img_data is not None or img_file is not None
    vor_results = voronoi_edge(img_data=img_data, img_file=img_file,
                               threshold=threshold, kernelsize=kernelsize)

    xlim = vor_results['edge_image'].size[0]
    ylim = vor_results['edge_image'].size[1]

    x_data = vor_results['vor_center_x']
    y_data = vor_results['vor_center_y']
    plt.figure()
    plt.scatter(x_data, y_data, s=0.5)
    plt.xlim(0, xlim)
    plt.ylim(0, ylim)
    if plot_name is None:
        plt.savefig('voronoi_fig.png')
    else:
        plt.savefig(plot_name+'.png')



