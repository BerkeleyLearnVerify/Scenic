import math
import numpy as np

EARTH_RADIUS = 6371008.0


def euclidean_dist(desired_x_y, comparative_x_y):
    d_x, d_y = desired_x_y[0], desired_x_y[1]
    c_x, c_y = comparative_x_y[0], comparative_x_y[1]
    return math.hypot(d_x - c_x, d_y - c_y)


def quaternion_for(theta, phi, psi):
    # (conversion math from http://www.xsquawkbox.net/xpsdk/mediawiki/MovingThePlane)
    sin_theta, cos_theta = math.sin(theta/2), math.cos(theta/2)
    sin_phi, cos_phi = math.sin(phi/2), math.cos(phi/2)
    sin_psi, cos_psi = math.sin(psi/2), math.cos(psi/2)
    quaternion = [
        cos_psi * cos_theta * cos_phi + sin_psi * sin_theta * sin_phi,
        cos_psi * cos_theta * sin_phi - sin_psi * sin_theta * cos_phi,
        cos_psi * sin_theta * cos_phi + sin_psi * cos_theta * sin_phi,
        -cos_psi * sin_theta * sin_phi + sin_psi * cos_theta * cos_phi
    ]
    return quaternion


def compute_heading_error(desired, real):
    '''
    param desired: desired heading
    param real: current heading
    return: heading error
    '''
    error, sign = desired - real, 1
    phi = abs(error) % 360
    if not (0 <= error <= 180 or -360 <= error <= -180):
        sign = -1
    if phi > 180:
        return (360 - phi) * sign
    return phi * sign


def cross_track_distance(start_lat, start_lon, end_lat, end_lon, d_lat, d_lon):
    # ref: http://www.movable-type.co.uk/scripts/latlong.html
    d13 = great_circle_distance_haversine(start_lat, start_lon, d_lat, d_lon) / EARTH_RADIUS
    theta12 = np.radians(initial_bearing(start_lat, start_lon, end_lat, end_lon))
    theta13 = np.radians(initial_bearing(start_lat, start_lon, d_lat, d_lon))
    return -(np.arcsin(np.sin(d13) * np.sin(theta13 - theta12)) * EARTH_RADIUS)


def great_circle_distance_haversine(start_lat, start_lon, end_lat, end_lon):
    phi1, phi2 = np.radians(start_lat), np.radians(end_lat)
    dphi, dlambda = np.radians(start_lat - end_lat), np.radians(start_lon - end_lon)
    a = np.sin(dphi / 2) * np.sin(dphi / 2) + np.cos(phi1) * np.cos(phi2) * \
                np.sin(dlambda / 2) * np.sin(dlambda / 2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return c * EARTH_RADIUS


def initial_bearing(start_lat, start_lon, end_lat, end_lon):
    lat1, lat2 = np.radians(start_lat), np.radians(end_lat)
    lon1, lon2 = np.radians(start_lon), np.radians(end_lon)
    y = np.sin(lon2 - lon1) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * \
                np.cos(lat2) * np.cos(lon2 - lon1)
    return np.degrees(np.arctan2(y, x)) % 360
