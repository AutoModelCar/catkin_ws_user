import numpy as np

import cv2

ys, xs = np.ogrid[:256, :256]


class ColorSpace(object):
    def __init__(self, channels, conversion):
        self.channels = channels
        self.conversion = conversion

        self.free_channel = 2
        self.free_val = 127

        self.img = np.zeros((256, 256, 3), np.uint8)

        cv2.namedWindow(channels)
        cv2.createTrackbar('channel', self.channels, 0, 2, self.set_free_channel)
        cv2.createTrackbar('value', self.channels, 0, 255, self.set_free_val)
        self.update()

    def update(self):
        for channel, val in enumerate((xs, ys)):
            if channel >= self.free_channel:
                channel += 1
            self.img[:, :, channel] = val

        self.img[:, :, self.free_channel] = self.free_val
        cv2.imshow(self.channels, cv2.cvtColor(self.img, self.conversion))

    def set_free_channel(self, val):
        self.free_channel = val
        self.update()

    def set_free_val(self, val):
        self.free_val = val
        self.update()


def main():
    ColorSpace('HSV', cv2.COLOR_HSV2BGR)
    ColorSpace('Lab', cv2.COLOR_Lab2BGR)
    ColorSpace('HLS', cv2.COLOR_HLS2BGR)

    while cv2.waitKey(0) & 0xFF != 27:
        pass

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
