
try:
    import cv2
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires opencv-python to be installed')

from calibration_utils import calibrate_camera, undistort
from binarization_utils import binarize
from perspective_utils import birdeye
from line_utils import get_fits_by_sliding_windows, draw_back_onto_the_road, Line, get_fits_by_previous_fits
from globals import xm_per_pix, time_window
import math
from scipy.special import *
import numpy as np

try:
    from PIL import Image
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires Pillow to be installed')

import sys
import squeezedet as nn

from vehicle import Driver

read_samples_from = "../../data/gaussian_lc.pickle"

processed_frames = 0                    # counter of frames processed (when processing video)
line_lt = Line(buffer_len=time_window)  # line on the left of the lane
line_rt = Line(buffer_len=time_window)  # line on the right of the lane


def prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, line_lt, line_rt, offset_meter):
    """
    Prepare the final pretty pretty output blend, given all intermediate pipeline images

    :param blend_on_road: color image of lane blend onto the road
    :param img_binary: thresholded binary image
    :param img_birdeye: bird's eye view of the thresholded binary image
    :param img_fit: bird's eye view with detected lane-lines highlighted
    :param line_lt: detected left lane-line
    :param line_rt: detected right lane-line
    :param offset_meter: offset from the center of the lane
    :return: pretty blend with all images and stuff stitched
    """
    h, w = blend_on_road.shape[:2]

    thumb_ratio = 0.2
    thumb_h, thumb_w = int(thumb_ratio * h), int(thumb_ratio * w)

    off_x, off_y = 20, 15

    # add a gray rectangle to highlight the upper area
    mask = blend_on_road.copy()
    mask = cv2.rectangle(mask, pt1=(0, 0), pt2=(w, thumb_h+2*off_y), color=(0, 0, 0), thickness=cv2.FILLED)
    blend_on_road = cv2.addWeighted(src1=mask, alpha=0.2, src2=blend_on_road, beta=0.8, gamma=0)

    # add thumbnail of binary image
    thumb_binary = cv2.resize(img_binary, dsize=(thumb_w, thumb_h))
    thumb_binary = np.dstack([thumb_binary, thumb_binary, thumb_binary]) * 255
    blend_on_road[off_y:thumb_h+off_y, off_x:off_x+thumb_w, :] = thumb_binary

    # add thumbnail of bird's eye view
    thumb_birdeye = cv2.resize(img_birdeye, dsize=(thumb_w, thumb_h))
    thumb_birdeye = np.dstack([thumb_birdeye, thumb_birdeye, thumb_birdeye]) * 255
    blend_on_road[off_y:thumb_h+off_y, 2*off_x+thumb_w:2*(off_x+thumb_w), :] = thumb_birdeye

    # add thumbnail of bird's eye view (lane-line highlighted)
    thumb_img_fit = cv2.resize(img_fit, dsize=(thumb_w, thumb_h))
    blend_on_road[off_y:thumb_h+off_y, 3*off_x+2*thumb_w:3*(off_x+thumb_w), :] = thumb_img_fit

    # add text (curvature and offset info) on the upper right of the blend
    mean_curvature_meter = np.mean([line_lt.curvature_meter, line_rt.curvature_meter])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(blend_on_road, 'Curvature radius: {:.02f}m'.format(mean_curvature_meter), (860, 60), font, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(blend_on_road, 'Offset from center: {:.02f}m'.format(offset_meter), (860, 130), font, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

    return blend_on_road


def compute_offset_from_center(line_lt, line_rt, frame_width):
    """
    Compute offset from center of the inferred lane.
    The offset from the lane center can be computed under the hypothesis that the camera is fixed
    and mounted in the midpoint of the car roof. In this case, we can approximate the car's deviation
    from the lane center as the distance between the center of the image and the midpoint at the bottom
    of the image of the two lane-lines detected.

    :param line_lt: detected left lane-line
    :param line_rt: detected right lane-line
    :param frame_width: width of the undistorted frame
    :return: inferred offset
    """
    if line_lt.detected and line_rt.detected:
        line_lt_bottom = np.mean(line_lt.all_x[line_lt.all_y > 0.95 * line_lt.all_y.max()])
        line_rt_bottom = np.mean(line_rt.all_x[line_rt.all_y > 0.95 * line_rt.all_y.max()])
        lane_width = line_rt_bottom - line_lt_bottom
        midpoint = frame_width / 2
        #offset_pix = abs((line_lt_bottom + lane_width / 2) - midpoint)
        offset_pix = (line_lt_bottom + lane_width / 2) - midpoint
        offset_meter = xm_per_pix * offset_pix

    else:
        offset_meter = -1

    return offset_meter


def process_pipeline(frame, keep_state=True):
    """
    Apply whole lane detection pipeline to an input color frame.
    :param frame: input color frame
    :param keep_state: if True, lane-line state is conserved (this permits to average results)
    :return: output blend with detected lane overlaid
    """

    global line_lt, line_rt, processed_frames

    # undistort the image using coefficients found in calibration
    img_undistorted = undistort(frame, mtx, dist, verbose=False)

    # binarize the frame s.t. lane lines are highlighted as much as possible
    img_binary = binarize(img_undistorted, verbose=False)

    # compute perspective transform to obtain bird's eye view
    img_birdeye, M, Minv = birdeye(img_binary, verbose=False)

    # fit 2-degree polynomial curve onto lane lines found
    if processed_frames > 0 and keep_state and line_lt.detected and line_rt.detected:
        line_lt, line_rt, img_fit = get_fits_by_previous_fits(img_birdeye, line_lt, line_rt, verbose=False)
    else:
        try:
            line_lt, line_rt, img_fit = get_fits_by_sliding_windows(img_birdeye, line_lt, line_rt, n_windows=9, verbose=False)
        except:
            print('Lane assistant: Skipping frame')

    # compute offset in meter from center of the lane
    offset_meter = compute_offset_from_center(line_lt, line_rt, frame_width=frame.shape[1])

    # draw the surface enclosed by lane lines back onto the original frame
    #blend_on_road = draw_back_onto_the_road(img_undistorted, Minv, line_lt, line_rt, keep_state)

    # stitch on the top of final output images from different steps of the pipeline
    #blend_output = prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, line_lt, line_rt, offset_meter)

    processed_frames += 1

    #return blend_output

    return offset_meter

driver = Driver()

camera_lk = driver.getCamera("camera_lk")
camera_lk.enable(1)
#camera_lk.recognitionEnable(1)

"""CNN initialization"""

camera_cnn = driver.getCamera("camera_cnn")
camera_cnn.enable(1)
#camera_cnn.recognitionEnable(1)

CHECKPOINT = './data/checkpoints/model-traffic-cones.ckpt-1150'
print('Loading neural net...')
net = nn.init(CHECKPOINT)
print('Done')

speed = 10
DETECT_FREQ = 8
IMG_W_CNN = 1242
IMG_H_CNN = 375

NN_PROB_THRESH = 0.5

IMG_W_LK = 1280//2
IMG_H_LK = 720//2

ret, mtx, dist, rvecs, tvecs = calibrate_camera(calib_images_dir='camera_cal')
delta_t = 0.032
cruising_speed = 10.

def normal_steering(x, mean, var):
    return np.sign(x-mean+1e-6)*np.exp(-(x-mean)**2/(2*var))/np.sqrt(2*np.pi*var)

def estimate_dist(pred):
    '''Estimate distance from closest detected traffic cone'''

    if not pred:
        return float('+Inf')

    K = 771.31
    EXP = -0.587

    # Extract areas of predicted bounding boxes
    box_areas = [w*h for (_,_,(_,_,w,h)) in pred]

    # Get max area, i.e., clostest object
    max_area = max(box_areas)

    # Estimate distance (K, EXP determined via exp regression)
    dist = K*math.pow(max_area,EXP)

    return dist

def get_camera_image(camera,img_w,img_h):
    '''Get image from a camera'''
    image = camera.getImage()
    frame = Image.frombytes('RGBA', (img_w,img_h), image, 'raw', "BGRA")
    return frame



def main(argv):
    print(argv)


    TEST_NN=True
    reaction_time = 0.75 if TEST_NN else float(argv[1])
    cruising_speed = 15.0 if TEST_NN else float(argv[2])
    print(reaction_time)
    steering_constant = 35.0*math.pow(cruising_speed,-1.442)
    mean=0.55
    var = 0.5


    # First the car does lane keeping for about 1.5 secs (50 iters)
    driver.setCruisingSpeed(cruising_speed)
    driver.setSteeringAngle(0)
    driver.step()
    # * np.sqrt(2 * np.pi * var)
    x_start = mean + np.sqrt(2 * var) * erfinv(-0.99)
    x_end = mean + np.sqrt(2 * var) * erfinv(0.99)
    max_iters = int(math.ceil((x_end - x_start) / delta_t))
    reaction_iters = int(math.ceil(reaction_time / delta_t))
    print("Mean, var, reaction time: ", mean, var, reaction_time)

    d = np.inf
    iter = 0
    k = 2
    offset_center = 0

    print("Mode: LANE KEEPING")
    #while d > k*cruising_speed:
    while d > 15.:
    #while True:
        if iter%DETECT_FREQ == 1:

            frame = get_camera_image(camera_lk,IMG_W_LK,IMG_H_LK)
            frame = np.array(frame.convert('RGB'))
            try:
                offset_center = process_pipeline(frame, keep_state=False)
            except:
                print('Lane assistant: Skipping frame')
                offset_center = 0
            steer_angle = offset_center*0.5

            driver.setSteeringAngle(steer_angle)

            image_cones = get_camera_image(camera_cnn,IMG_W_CNN,IMG_H_CNN)


            # convert to a openCV2 image
            cv2_image = cv2.cvtColor(np.array(image_cones), cv2.COLOR_RGB2BGR)

            # call neural net
            pred = nn.classify(cv2_image, net, NN_PROB_THRESH)
            d = estimate_dist(pred)

            print("CNN estimated distance from cones: ", d)
        iter += 1
        driver.step()

    # Now start the gaussian maneuver
    x_current = x_start
    for _ in range(reaction_iters):
        driver.setCruisingSpeed(cruising_speed)
        driver.setSteeringAngle(0)
        driver.step()

    driver.setCruisingSpeed(cruising_speed)
    driver.setSteeringAngle(normal_steering(x=x_current, mean=mean, var=var))


    print("Mode: LANE CHANGE")
    for iter in range(max_iters+1):
        driver.setCruisingSpeed(cruising_speed)
        if iter % 8 == 0:
            x_current = x_start + iter * delta_t
            print(x_current, steering_constant * normal_steering(x=x_current, mean=mean,
                                                                 var=var), x_current - mean)
            driver.setSteeringAngle(-steering_constant *
                                    normal_steering(x=x_current, mean=mean, var=var))
        driver.step()

    # Now do lane keeping again
    print("Mode: LANE KEEPING")
    iter = 0
    while driver.step() != -1:
        if iter % 8 == 0:

            frame = get_camera_image(camera_lk,IMG_W_LK,IMG_H_LK)
            frame = np.array(frame.convert('RGB'))

            try:
                offset_center = process_pipeline(frame, keep_state=False)
            except:
                print('Lane assistant: Skipping frame')
                offset_center = 0
            steer_angle = offset_center * 0.5

            driver.setSteeringAngle(steer_angle)
        iter += 1

if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(sys.argv)
