import numpy as np

import cv2


def value_callback(update, values, i):
    def callback(value):
        values[0, 0, i] = value
        update()

    return callback


def create_color_picker(window, channel_def, conversion):
    img = np.zeros((300, 512, 3), np.uint8)
    cv2.namedWindow(window)

    values = np.uint8([[[0, 0, 0]]])

    def update():
        img[:] = cv2.cvtColor(values, conversion)[0][0]
        cv2.imshow(window, img)

    for i, (channel, (start, end)) in enumerate(channel_def):
        cv2.createTrackbar(channel, window, start, end, value_callback(update, values, i))

    update()


def create_hsv_picker(window='HSV'):
    create_color_picker(window, zip('HSV', ((0, 179), (0, 255), (0, 255))), cv2.COLOR_HSV2BGR)
    return window


def create_lab_picker(window='Lab'):
    create_color_picker(window, zip('Lab', ((0, 255), (0, 255), (0, 255))), cv2.COLOR_Lab2BGR)
    return window


def main():
    create_hsv_picker()
    create_lab_picker()

    while cv2.waitKey(0) & 0xFF != 27:
        pass

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
