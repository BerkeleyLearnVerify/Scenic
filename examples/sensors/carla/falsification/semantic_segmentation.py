import numpy as np
import torchmetrics

from scenic.core.simulators import Action
import torch
from torchvision import transforms as T


def carla_to_np_rgb(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))  # RGBA format
    array = array[:, :, :3]  # Take only RGB
    array = array[:, :, ::-1]  # Revert order
    return array


def carla_to_np_ss(seg):
    array = np.frombuffer(seg.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (seg.height, seg.width, 4))  # RGBA format
    array = array[:, :, 2]  # Take only R
    return array


class SemanticSegmentationModel:

    def __init__(self, model_path):
        self.device = "cpu"
        self.model = torch.load(model_path, map_location=self.device)
        self.model.eval()
        self.model.to(self.device)
        self.jaccard_index = torchmetrics.JaccardIndex(13)

    def prepare_observations(self, rgb, ss):
        image_np = carla_to_np_rgb(rgb).copy()
        mask_np = carla_to_np_ss(ss).copy()
        mask_np[mask_np >= 13] = 3
        t = T.Compose([T.ToTensor()])
        image = t(image_np)
        mask = torch.from_numpy(mask_np)
        return image, mask

    def get_miou(self, image, mask):
        image = image.to(self.device)
        mask = mask.to(self.device)
        with torch.no_grad():
            image = image.unsqueeze(0)
            mask = mask.unsqueeze(0)

            output = self.model(image)
            pred_mask = torch.argmax(output, dim=1)
            score = self.jaccard_index(pred_mask, mask)
        return score, output

    def compute_miou(self, agent, rgb_key="front_rgb", ss_key="front_ss"):
        image, ground_truth = self.prepare_observations(agent.observations[rgb_key],
                                                        agent.observations[ss_key])
        score, _ = self.get_miou(image, ground_truth)
        score = score.detach().numpy()
        return score
