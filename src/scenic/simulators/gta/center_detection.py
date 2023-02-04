'''
This file contains helper functions
'''

import itertools
from typing import NamedTuple, Tuple
from copy import deepcopy

from scenic.simulators.gta.img_modf import *

'''
Compute neighbors of a given point/pixel
'''
def generate_neighbors(x):
    delta = np.arange(-1, 2)
    grid_delta = np.array(list(itertools.product(delta, delta)))
    return np.array(x)+ grid_delta


'''
Generate circle of a given radius and transformed by a center and a theta
'''
def generate_circle(radius, num_samples, theta, center):
    transformed_center = transform_center(center, radius, theta)
    thetas = np.linspace(0, 2*np.pi+0.01, num_samples)
    return transformed_center[0] + radius*np.cos(thetas), \
           transformed_center[1] + radius*np.sin(thetas)

'''
Transform the circle center as x+r*cos(90+theta) and y-r*sin(90+theta)
'''
def transform_center(center, radius, theta):
    return [center[0] + radius * np.cos(theta), center[1] - radius * np.sin(
        theta)]


'''
Compute image gradients from sobel edge detector
'''
def compute_gradient_sobel(img_data=None, img_file=None, threshold=100,
                     kernelsize=1, bw_kernelsize=3):
    assert img_data is not None or img_file is not None
    if img_data is None:
        img_data = Image.open(img_file)

    img_copy = img_data.copy()

    # Get the black and white image
    img_bw = convert_black_white(img_data=img_copy, img_file=img_file,
                                 threshold=threshold)
    cv_bw = cv2.cvtColor(np.array(img_bw), cv2.COLOR_RGB2BGR)

    # Gradients along x, y. The laplacian used for edge detection is actually
    #  a combination of these two
    sobelx = cv2.Sobel(cv_bw, cv2.CV_64F, 1, 0, ksize=kernelsize)
    sobely = cv2.Sobel(cv_bw, cv2.CV_64F, 0, 1, ksize=kernelsize)

    angle_sobel = np.arctan(sobelx / (sobely + 1e-6))

    edge_image = get_edges(img_data=img_copy, img_file=img_file,
                           threshold=threshold, kernelsize=bw_kernelsize)

    edge_x, edge_y = np.nonzero(np.asarray(edge_image))

    angle_along_edge = {}
    for x, y in zip(edge_x, edge_y):
        angle_along_edge[(x,y)] = angle_sobel[x][y][0]

    return angle_along_edge

'''
This is very coarse, avoid using this
'''
def compute_gradient_manual(img_data=None, img_file=None, threshold=100,
                     kernelsize=1):
    assert img_data is not None or img_file is not None
    if img_data is None:
        img_data = Image.open(img_file)

    img_copy = img_data.copy()

    # Directly work with the edges
    edge_image = get_edges(img_data=img_copy, img_file=img_file,
                           threshold=threshold, kernelsize=kernelsize)

    edge_x, edge_y = np.nonzero(np.asarray(edge_image))
    grad_x, grad_y = np.gradient(np.asarray(edge_image))
    angle_grad = np.arctan(grad_x/(grad_y+1e-6))

    angle_along_edge = {}
    for x, y in zip(edge_x, edge_y):
        angle_along_edge[(x,y)] = angle_grad[x][y]

    return angle_along_edge

'''
Compute all connected edges
'''

def generate_connected_edges(edge_locs):

    # This is a list of collected edges
    collected_edges = []
    while len(edge_locs) > 0:
        start_point = edge_locs.pop()
        new_edge = set({start_point})
        neighbors = set(tuple(map(tuple, generate_neighbors(start_point))))
        to_visit = neighbors.intersection(edge_locs)
        new_edge.update(neighbors.intersection(edge_locs))
        edge_locs.difference_update(new_edge.intersection(edge_locs))
        while len(to_visit) > 0:
            k = to_visit.pop()
            neighbors = set(tuple(map(tuple, generate_neighbors(k))))
            to_visit.update(neighbors.intersection(edge_locs))
            new_edge.update(neighbors.intersection(edge_locs))
            edge_locs.difference_update(new_edge.intersection(edge_locs))

        collected_edges.append(new_edge)
    return collected_edges

'''
To find the center of the road perpendicular to a given point
num_samples is function which can potentially change with the radius of the
circle
'''

def find_center(x, theta, collected_edges, all_edges,  num_samples, bw_image):

    '''
    Find which edge x lies in
    '''
    for edge in collected_edges:
        if tuple([x[1], x[0]]) in edge:
            x_edge = edge
            break

    possible_opp_edge = all_edges.difference(x_edge)

    r = 1

    '''
    Find direction of computing the growing circles
    '''
    if theta < 0:
        theta = theta + 2*np.pi
    theta_plus = theta + np.pi/2.
    theta_minus = theta - np.pi / 2.

    center_plus = transform_center(center=x, radius=2, theta=theta_plus)

    center_plus = np.array(np.around(center_plus), dtype=int)

    if center_plus[0] >= bw_image.shape[1]:
        center_plus[0] = bw_image.shape[1] - 1
    if center_plus[1] >= bw_image.shape[0]:
        center_plus[1] = bw_image.shape[0] - 1
    if bw_image[center_plus[1]][center_plus[0]] == 0:
        theta = theta_plus
        tangent_theta = theta - np.pi/2.
    else:
        theta = theta_minus
        tangent_theta = theta + np.pi / 2.

    while True:
        circle_x, circle_y = generate_circle(radius=r, center=x, theta=theta,
                                             num_samples=num_samples(r))

        int_circle = set(itertools.product(np.array(np.around(circle_y),
                        dtype=int), np.array(np.around(circle_x), dtype=int)))


        if len(possible_opp_edge.intersection(int_circle)) > 0:
            opp_edge = possible_opp_edge.intersection(int_circle)
            break

        r += 1
    if len(opp_edge) > 1:
        arg_min = np.linalg.norm(np.array(list(opp_edge)) - np.array([x[1],
                                x[0]]), axis=1).argmin()
        opp_point = np.array(list(opp_edge))[arg_min]
    else:
        opp_point = opp_edge.pop()

    mid_loc = np.array(np.around((np.array([x[1],x[0]]) +
                                  np.array(opp_point))/2.), dtype=int)

    orig_theta = tangent_theta

    # When there is a clear greater than and less than
    if mid_loc[0] > x[1] and mid_loc[1] < x[0]:
        if tangent_theta > np.pi:
            tangent_theta = tangent_theta - np.pi
        if tangent_theta < np.pi/2.:
            tangent_theta = tangent_theta + np.pi
    elif mid_loc[0] > x[1] and mid_loc[1] > x[0]:
        if tangent_theta < np.pi:
            tangent_theta = tangent_theta + np.pi
    elif mid_loc[0] < x[1] and mid_loc[1] < x[0]:
        if tangent_theta > np.pi/2.:
            tangent_theta = tangent_theta - np.pi
    elif mid_loc[0] < x[1] and mid_loc[1] > x[0]:
        if tangent_theta < 3.*np.pi/2. and tangent_theta > 0:
            tangent_theta = tangent_theta + np.pi


    # When there exists one equality
    if mid_loc[0] == x[1]:
        if mid_loc[1] < x[0]:
            if tangent_theta > np.pi:
                tangent_theta = tangent_theta - np.pi
        else:
            if tangent_theta < 3./2.*np.pi:
                tangent_theta = tangent_theta + np.pi

    if mid_loc[1] == x[0]:
        if mid_loc[0] < x[1]:
            if tangent_theta > np.pi/2.:
                tangent_theta = tangent_theta - np.pi
        else:
            if tangent_theta < np.pi:
                tangent_theta = tangent_theta + np.pi


    if tangent_theta < 0:
        tangent_theta = tangent_theta + 2 * np.pi




    return orig_theta, tangent_theta, tuple(opp_point), tuple(mid_loc)


'''
Update all edges with the bounding box of the image
'''

def compute_bb(img_dimensions):
    im_x_min = -1
    im_y_min = -1
    im_x_max = img_dimensions[1]
    im_y_max = img_dimensions[0]

    '''
    Compute the bounding box
    '''
    bb = set()
    bb.update({(im_x_min, y) for y in range(im_y_min, im_y_max+1)})
    bb.update({(im_x_max, y) for y in range(im_y_min, im_y_max + 1)})
    bb.update({(x, im_y_min) for x in range(im_x_min, im_x_max + 1)})
    bb.update({(x, im_y_max) for x in range(im_x_min, im_x_max + 1)})

    return bb

class EdgeData(NamedTuple):
    init_theta: float
    tangent: float
    opp_loc: Tuple[float, float]
    mid_loc: Tuple[float, float]

'''
Compute the dictionary with key is the start state and the values are a tuple
consisting of the angle, mid point and the the opposite edge
'''

def compute_midpoints(img_data, bw_kernelsize=1, kernelsize=3, threshold=100,
                      num_samples = lambda r:16):
    # First compute the black and white image and store it
    img_bw = convert_black_white(img_data=img_data)
    img_bw = img_bw.convert('L')
    img_bw_int = np.array(img_bw, dtype=int)

    # Compute the point to the tangent angle dictionary
    cae = compute_gradient_sobel(img_data=img_data,
                                 bw_kernelsize=bw_kernelsize,
                                 kernelsize=kernelsize,
                                 threshold=threshold)

    # Collect all edges
    all_edges = set(cae.keys())

    # Collect all roads
    connected_edges = generate_connected_edges(edge_locs=deepcopy(all_edges))

    # Compute bounding box
    bb = compute_bb(img_dimensions=img_data.size)

    all_edges.update(bb)

    # Compute midpoints
    point_data = {}
    for edge in connected_edges:
        for x in edge:
            init_theta, theta, opp_loc, mid_loc = find_center(x=[x[1], x[0]],
                                                    theta=cae[x],
                                           collected_edges=connected_edges,
                                           all_edges=all_edges,
                                           num_samples=num_samples,
                                           bw_image=img_bw_int)

            point_data[x] = EdgeData(init_theta=init_theta, tangent=theta,
                             opp_loc=opp_loc, mid_loc=mid_loc \
                    if opp_loc not in bb else (np.nan, np.nan))

    return point_data

'''
Compute the heading at a random sample
'''

def compute_heading(x, point_data):
    road_edges = tuple(point_data.keys())
    edge_distance = np.linalg.norm(np.array(list(road_edges)) - np.array(x),1,
                                   axis=1)
    closest_edge = list(road_edges)[edge_distance.argmin()]

    return closest_edge, point_data[closest_edge]

'''
Sampling from road
'''

def sample_from_road(img_data, num_samples):
    img_bw = convert_black_white(img_data=img_data)
    img_bw = img_bw.convert('L')
    img_bw_int = np.array(img_bw, dtype=int)

    road_y, road_x = np.where(img_bw_int == 0)
    all_roads = list(zip(road_y, road_x))

    locs = np.random.choice(len(all_roads), num_samples, replace=False)

    return np.array(all_roads)[locs]
