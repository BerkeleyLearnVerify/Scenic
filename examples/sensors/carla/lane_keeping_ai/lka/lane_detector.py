import copy

from .camera_geometry import CameraGeometry
import numpy as np
import cv2
import torch
from fastseg import MobileV3Small
import warnings

warnings.simplefilter('ignore', np.RankWarning)


class LaneDetector:
    def __init__(self, model, cam_geometry=CameraGeometry(), device="cuda"):
        self.camera_geometry = cam_geometry
        self.cut_v, self.grid = self.camera_geometry.precompute_grid()
        self.model = model
        if device == "cuda":
            assert torch.cuda.is_available()
        self.device = device
        self.status = {}

    def _predict(self, img):
        with torch.no_grad():
            image_tensor = img.transpose(2, 0, 1).astype('float32') / 255
            x_tensor = torch.from_numpy(image_tensor).to(self.device).unsqueeze(0)
            model_output = torch.softmax(self.model.forward(x_tensor), dim=1).cpu().numpy()
        return model_output

    def detect(self, img_array):
        model_output = self._predict(img_array)
        background, left, right = model_output[0, 0, :, :], model_output[0, 1, :, :], model_output[0, 2, :, :]
        return background, left, right

    def fit_poly(self, probs):
        probs_flat = np.ravel(probs[self.cut_v:, :])
        mask = probs_flat > 0.3
        if mask.sum() > 0:
            coeffs = np.polyfit(self.grid[:, 0][mask], self.grid[:, 1][mask], deg=3, w=probs_flat[mask])
        else:
            coeffs = np.array([0., 0., 0., 0.])
        return np.poly1d(coeffs)

    def get_fit_and_probs(self, img):
        _, left, right = self.detect(img)
        left_poly = self.fit_poly(left)
        right_poly = self.fit_poly(right)
        return left_poly, right_poly, left, right

    def get_center_lane_trajectory(self, image):
        poly_left, poly_right, left_mask, right_mask = self.get_fit_and_probs(image)
        x = np.arange(-2, 60, 1.0)
        y = -0.5 * (poly_left(x) + poly_right(x))
        # x,y is now in coordinates centered at camera, but camera is 0.5 in front of vehicle center
        # hence correct x coordinates
        x += self.camera_geometry.displacement
        traj = np.stack((x, y)).T
        self.status["left_lane_seg"] = copy.deepcopy(left_mask)
        self.status["right_lane_seg"] = copy.deepcopy(right_mask)
        return traj
