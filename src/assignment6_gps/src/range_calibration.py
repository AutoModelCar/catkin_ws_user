#!/usr/bin/env python2

import sys

import cv2
import os
from collections import deque

from balloon_detector import BalloonDetector

class RangeCalibrator(object):
    def __init__(self, window='result'):
        self.detector = BalloonDetector()
        self.img = None

        self.window = window
        cv2.namedWindow(window)

        cv2.createTrackbar('blur', window, 0, 50, self.set_blur)
        cv2.setTrackbarPos('blur', window, self.detector.blur_amount)

        cv2.createTrackbar('min-sat', window, 0, 255, self.set_min_sat)
        cv2.setTrackbarPos('min-sat', window, self.detector.hsv_range_min[1])

        cv2.createTrackbar('min-val', window, 0, 255, self.set_min_val)
        cv2.setTrackbarPos('min-val', window, self.detector.hsv_range_min[2])

        cv2.createTrackbar('min-samples', window, 0, 255, self.set_min_samples)
        cv2.setTrackbarPos('min-samples', window, self.detector.min_samples)

    def set_img(self, img):
        self.img = img
        self.update()

    def update(self):
        if self.img is not None:
            img = self.img.copy()  # avoid overwriting original
            self.detector.calculate_best_position(img, max_iters=1)
            cv2.imshow('mask', self.detector.mask)
            cv2.moveWindow('mask', img.shape[1] + 2, 0)
            cv2.imshow(self.window, img)

    def set_blur(self, val):
        self.detector.blur_amount = val if val % 2 == 1 or val == 0 else val + 1
        self.update()

    def set_min_sat(self, val):
        self.detector.hsv_range_min[1] = val
        self.update()

    def set_min_val(self, val):
        self.detector.hsv_range_min[2] = val
        self.update()

    def set_min_samples(self, val):
        self.detector.min_samples = val
        self.update()


def main():
    if len(sys.argv) != 2:
        print('need a picture / directory as the first argument')
        sys.exit(1)

    path = sys.argv[1]
    if os.path.isfile(path):
        path, fname = os.path.split(path)
        files = deque([fname])
    else:
        files = deque(sorted(fn for fn in os.listdir(path) if fn.endswith('.png') or fn.endswith('.jpg')))

    calib = RangeCalibrator()

    rotate_key = {ord('n'): 1, ord('p'): -1, ord('r'): 0}
    k = ord('r')
    while k not in {27, ord('q')}:
        r = rotate_key.get(k, None)
        if r is not None:
            files.rotate(r)
            calib.img = cv2.imread(os.path.join(path, files[0]))
            calib.update()

        k = cv2.waitKey(0) & 0xFF

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
