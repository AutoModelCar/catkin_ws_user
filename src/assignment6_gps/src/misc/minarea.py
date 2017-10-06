import numpy as np

import cv2

img_size = 500


def main():
    key = -1
    while key not in (27, ord('q'), ord('Q')):
        count = np.random.randint(1, 101)
        points = np.random.randint(img_size / 4, img_size * 3 / 4, 2 * count).reshape((count, 2))

        # Find the minimum area enclosing circle
        center, radius = cv2.minEnclosingCircle(points)

        img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

        # Draw the points
        for point in points:
            cv2.circle(img, tuple(point), 3, (0, 0, 255), cv2.FILLED, cv2.LINE_AA)

        # Draw the circle
        cv2.circle(img, tuple(np.round(center).astype(np.int)), int(radius), (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('Enclosing Circle', img)
        key = cv2.waitKey(0) & 0xFF

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
