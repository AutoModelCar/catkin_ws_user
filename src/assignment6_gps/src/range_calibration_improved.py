#!/usr/bin/env python2

import numpy as np
import os
import sys
from collections import deque

import cv2
import scipy.ndimage

from balloon_detector import draw_text

size_min, size_max = 3, 15


class RangeCalibrator(object):
    def __init__(self, window='result'):
        self.img = None
        self.hsv = None
        self.hue = self.hue_cos = self.hue_sin = None

        self.window = window
        cv2.namedWindow(window)

        self.close_iter = 10

        self.hsv_range_min = np.array((0, 90, 98))
        self.hsv_range_max = np.array((181, 255, 255))

        cv2.createTrackbar('min-sat', window, 0, 255, self.set_min_sat)
        cv2.setTrackbarPos('min-sat', window, self.hsv_range_min[1])

        cv2.createTrackbar('min-val', window, 0, 100, self.set_min_val)
        cv2.setTrackbarPos('min-val', window, self.hsv_range_min[2])

        cv2.createTrackbar('close-iter', window, 0, 255, self.set_close_iter)
        cv2.setTrackbarPos('close-iter', window, self.close_iter)

    def set_img(self, img):
        self.img = img
        self.hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hue = self.hsv[:, :, 0].astype(np.float)
        hue_radians = np.pi / 90 * hue
        self.hue_sin, self.hue_cos = np.sin(hue_radians), np.cos(hue_radians)
        self.update()

    def update(self):
        if self.img is None:
            return

        # interpret min-val as a percentiles
        range_min = self.hsv_range_min.copy()
        range_min[2] = int(np.percentile(self.hsv[:, :, 2], self.hsv_range_min[2]))

        img = self.img.copy()  # avoid overwriting original
        mask = cv2.inRange(self.hsv, range_min, self.hsv_range_max)

        if self.close_iter > 0:
            scipy.ndimage.binary_closing(mask, iterations=self.close_iter, output=mask)
            cv2.imshow('mask', mask * 255)
        else:
            cv2.imshow('mask', mask)

        cv2.moveWindow('mask', img.shape[1] + 2, 0)

        labelled, nfeatures = scipy.ndimage.label(mask)

        # print('percentiles %s -> ranges: %s' % (self.hsv_range_min, range_min))

        selected = []
        for label, yx in enumerate(scipy.ndimage.find_objects(labelled), 1):
            y, x = yx
            w, h = x.stop - x.start, y.stop - y.start
            if size_min <= w <= size_max and size_min <= h <= size_max:
                selected.append(label)
                cv2.rectangle(img, (x.start, y.start), (x.stop, y.stop), (0, 255, 0))

        centers = scipy.ndimage.center_of_mass(np.ones(labelled.shape), labelled, selected)

        # average hue by summing the unit vectors and measuring the resulting vector angle
        hues = np.arctan2(scipy.ndimage.sum(self.hue_sin, labelled, selected),
                          scipy.ndimage.sum(self.hue_cos, labelled, selected))

        for center, hue in zip(centers, hues):
            cy, cx = int(center[0]), int(center[1])
            cv2.rectangle(img, (cx - 1, cy - 1), (cx + 1, cy + 1), (255, 255, 255))
            draw_text(img, '%d, %d' % (cx, cy), (cx + 10, cy + 5), hue_to_bgr(90 / np.pi * hue), f_size=0.3)

        draw_text(img, 'nfeatures: %d' % nfeatures, (10, mask.shape[0] - 20), (0, 255, 0))

        cv2.imshow(self.window, img)

    def set_min_sat(self, val):
        self.hsv_range_min[1] = val
        self.update()

    def set_min_val(self, val):
        self.hsv_range_min[2] = val
        self.update()

    def set_close_iter(self, val):
        self.close_iter = val
        self.update()


def hue_to_bgr(hue):
    if hue < 0:  # convert negative hues to positive
        hue += 180
    hsv_tuple = np.uint8([[[hue, 255, 255]]])
    return tuple(cv2.cvtColor(hsv_tuple, cv2.COLOR_HSV2BGR)[0, 0].astype(np.int))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'img.png'
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
            calib.set_img(cv2.imread(os.path.join(path, files[0])))

        k = cv2.waitKey(0) & 0xFF

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
