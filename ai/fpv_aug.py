"""
Augmentation that closes the gap between the training photos and what the
drone's camera actually delivers.

Every image in the dataset is a sharp 2048 px phone photo taken from standing
height. The Pi sees a 640x480 frame off a small rolling-shutter sensor, shaken
by the props, then JPEG-compressed for the MJPEG stream. A model trained only
on clean photos has never seen the input it will be asked to run on.

Each degradation below corresponds to something specific in that path:

    motion blur   props shake the airframe and the drone translates during the
                  exposure; blur is directional, not symmetric
    defocus       fixed-focus lens, pad off the focal plane during descent
    downscale     cheap sensor and a heavily upscaled crop; destroys the fine
                  tape texture the network might otherwise lean on
    noise         high ISO indoors and in low light
    JPEG          drone_pi.py encodes the stream at quality 70

Applied after RandomPerspective, so the image is already at the network's input
size and the blur radii are in the same units the network will see.
"""

import cv2
import numpy as np


class FPVDegrade:
    """Randomly degrade an image the way the FPV/Pi capture path would."""

    def __init__(self, p=0.8, seed=None):
        self.p = p
        self.rng = np.random.default_rng(seed)

    def _motion_blur(self, img):
        k = int(self.rng.integers(3, 12))
        kern = np.zeros((k, k), np.float32)
        kern[k // 2, :] = 1.0
        M = cv2.getRotationMatrix2D((k / 2 - 0.5, k / 2 - 0.5),
                                    float(self.rng.uniform(0, 180)), 1.0)
        kern = cv2.warpAffine(kern, M, (k, k))
        s = kern.sum()
        return cv2.filter2D(img, -1, kern / s) if s > 0 else img

    def _defocus(self, img):
        k = int(self.rng.integers(1, 4)) * 2 + 1
        return cv2.GaussianBlur(img, (k, k), 0)

    def _downscale(self, img):
        h, w = img.shape[:2]
        f = float(self.rng.uniform(0.35, 0.8))
        small = cv2.resize(img, (max(8, int(w * f)), max(8, int(h * f))),
                           interpolation=cv2.INTER_AREA)
        return cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)

    def _noise(self, img):
        sigma = float(self.rng.uniform(3, 14))
        out = img.astype(np.float32) + self.rng.normal(0, sigma, img.shape)
        return np.clip(out, 0, 255).astype(np.uint8)

    def _jpeg(self, img):
        q = int(self.rng.integers(25, 75))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
        return cv2.imdecode(buf, cv2.IMREAD_COLOR) if ok else img

    def __call__(self, labels):
        img = labels.get("img")
        if img is None or self.rng.random() > self.p:
            return labels

        # at most one blur — stacking them just erases the pad
        blur = self.rng.random()
        if blur < 0.45:
            img = self._motion_blur(img)
        elif blur < 0.65:
            img = self._defocus(img)

        if self.rng.random() < 0.35:
            img = self._downscale(img)
        if self.rng.random() < 0.35:
            img = self._noise(img)
        if self.rng.random() < 0.45:
            img = self._jpeg(img)

        labels["img"] = np.ascontiguousarray(img)
        return labels


def install(trainer):
    """Insert FPVDegrade just before Format in the training pipeline.

    Re-checked every epoch because close_mosaic rebuilds the transform list
    partway through training and would otherwise drop it.
    """
    dataset = trainer.train_loader.dataset
    steps = dataset.transforms.transforms
    if not any(isinstance(t, FPVDegrade) for t in steps):
        steps.insert(len(steps) - 1, FPVDegrade())


def attach(model):
    model.add_callback("on_train_epoch_start", install)
    return model
