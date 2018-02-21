# import the necessary packages
import argparse

import cv2

# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
start = None
down = False


def click_and_crop(event, x, y, flags, param):
    global start, down

    if event == cv2.EVENT_LBUTTONDOWN:
        start = (x, y)
        down = True
        return

    elif event == cv2.EVENT_LBUTTONUP:
        down = False
    elif down and event == cv2.EVENT_MOUSEMOVE:
        pass
    else:
        return  # unrelated event

    end = (x, y)
    img = image.copy()
    cv2.rectangle(img, start, end, (0, 255, 0), 1)
    cv2.imshow("image", img)

    xmin, xmax = start[0], end[0]
    ymin, ymax = start[1], end[1]

    if xmin == xmax or ymin == ymax:
        return

    if xmin > xmax:
        xmin, xmax = xmax, xmin
    if ymin > ymax:
        ymin, ymax = ymax, ymin

    cv2.imshow("ROI", image[ymin:ymax, xmin:xmax])


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
args = ap.parse_args()

image = cv2.imread(args.image)
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)

cv2.imshow("image", image)
# keep looping until the 'q' key is pressed
while True:
    # display the image and wait for a keypress
    key = cv2.waitKey(0) & 0xFF

    if key == ord("r"):
        cv2.imshow("image", image)  # remove markings from image
    elif key == ord('q') or key == 27:
        break  # esc or q was pressed

cv2.destroyAllWindows()
