#!/usr/bin/env python2
import functools
import numpy as np
from collections import namedtuple
from itertools import combinations
from time import clock

import cv2
import scipy.optimize
from sklearn.cluster import DBSCAN


# data class for balloons (color specified as the hue from HSV)
Balloon = namedtuple('Balloon', ['name', 'hue', 'position'])

# estimates for balloon positions and hues
# hue is divided by two since opencv uses hues in ranges [0, 180)
balloon_x_base, balloon_x_dist = 2.33, 1.24
balloon_y_base, balloon_y_dist = 1.15, 1.85
balloons = (Balloon('lilac', 260 / 2, (balloon_x_base, balloon_y_base + balloon_y_dist)),
            Balloon('red',   354 / 2, (balloon_x_base + balloon_x_dist, balloon_y_base + balloon_y_dist)),
            Balloon('blue',  222 / 2, (balloon_x_base + balloon_x_dist, balloon_y_base)),
            Balloon('green', 120 / 2, (balloon_x_base, balloon_y_base)),)

# a balloon hues in radians with shape (1, 4)
# angles in range [-pi, pi)
balloon_hues = np.array([[_b.hue for _b in balloons]])


def timed(f):
    """
    decorator to display how long a function took to execute
    from: https://stackoverflow.com/a/27737385/1453080
    """

    @functools.wraps(f)
    def wrap(*args, **kw):
        ts = clock()
        result = f(*args, **kw)
        te = clock()
        print('%-30s  %8.2fms' %
              (f.__name__, (te - ts) * 1000))
        return result

    return wrap


class BalloonDetector(object):
    def __init__(self, blur_amount=0, max_cluster_dist=2, min_samples=5,
                 hsv_range_min=(0, 130, 110), hsv_range_max=(181, 255, 255)):
        # how much to blur saturation before thresholding
        self.blur_amount = blur_amount

        # greatest allowed gap in a cluster
        self.max_cluster_dist = max_cluster_dist

        # minimum number of pixels in a
        self.min_samples = min_samples

        self.hsv_range_min = np.array(hsv_range_min)
        self.hsv_range_max = np.array(hsv_range_max)

        # the following attributes are set during detection
        self.balloon_angles = None
        self.balloon_positions = None
        self.mask = None
        self.centers = None
        self.radii = None
        self.xy = None
        self.iterations = None
        self.residual = np.inf

    def detect_balloons(self, hsv, hue, widen_mask=0):
        """
        detect balloons in the image and draw crosses on the detected points
        :param widen_mask: how much to widen the range of HSV values that are considered
        """

        if self.blur_amount > 0:
            hsv = cv2.GaussianBlur(hsv, (self.blur_amount, self.blur_amount), 0)

        mask = cv2.inRange(hsv, self.hsv_range_min - widen_mask, self.hsv_range_max)

        self.mask = mask  # save the mask so the caller can plot it if desired

        # by, bu, bv = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
        # show_image(by, 'by component of YUV', 'gray')
        # show_image(bu, 'bu component of YUV', 'gray')
        # show_image(bv, 'bv component of YUV', 'gray')

        # show_image(saturation, 'saturation channel (HSV)', 'gray')
        # show_image(value, 'value channel (HSV)', 'gray')

        # blur_amount = self.blur_amount
        # blur_sat = cv2.GaussianBlur(saturation, (blur_amount, blur_amount), 0)
        # show_image(blur_sat, 'saturation + gaussian blur: %d' % blur_amount, 'gray')

        # _, sat_thresh = cv2.threshold(blur_sat, self.threshold, 255, cv2.THRESH_BINARY)
        # show_image(sat_thresh, 'threshold %d' % self.threshold, 'gray')

        ys, xs = np.where(mask)

        if len(ys) < 1:
            return {}

        X = np.column_stack((xs, ys))
        db = DBSCAN(eps=self.max_cluster_dist, min_samples=self.min_samples).fit(X)
        labels = db.labels_

        # Number of clusters that DBSCAN found (ignoring noise label)
        n_clusters = np.max(labels) + 1

        if n_clusters < 1:
            return {}

        # first two columns: x, y coordinates, remaining columns are hue distances
        clusters = np.zeros((n_clusters, 2))
        hues = np.zeros((n_clusters, 1))
        for k in xrange(n_clusters):
            points = X[labels == k]

            center = np.average(points, axis=0)
            clusters[k] = center

            x, y = center

            hues[k] = hue[int(y), int(x)]  # get hues of pixels in original image

        # map from each balloon to the best angle
        balloon_angles = {}
        balloon_positions = {}
        max_y, max_x = hue.shape
        origin = np.array((max_x, max_y)) * .5

        balloon_hue_dist = angle_diff(hues, balloon_hues)

        # iterate over balloons in the order of minimal hue distance
        max_count = min(len(balloons), n_clusters)
        while len(balloon_angles) < max_count:
            # find best match for this balloon
            row, col = np.unravel_index(balloon_hue_dist.argmin(), balloon_hue_dist.shape)
            balloon = balloons[col]

            pos = clusters[row]
            balloon_positions[balloon] = pos

            # calculate the angle from the center of the picture
            x, y = pos - origin
            balloon_angles[balloon] = np.arctan2(-y, x)

            balloon_hue_dist[:, col] = np.inf  # prevent getting this balloon again
            balloon_hue_dist[row] = np.inf  # set hue distances in this row to inf to prevent further matches

        self.balloon_positions = balloon_positions
        return balloon_angles

    def calculate_position(self, hsv, hue, widen_mask=(0, 0, 0)):
        """ :returns the position of the car in world coordinates or None if it isn't available """
        balloon_angles = self.detect_balloons(hsv, hue, widen_mask)
        self.balloon_angles = balloon_angles

        if len(balloon_angles) < 3:  # not enough balloons detected
            return None

        centers = []
        radii = []

        for (a_bal, alpha_a), (b_bal, alpha_b) in combinations(balloon_angles.items(), 2):
            a_to_b = np.array(b_bal.position) - a_bal.position

            alpha = alpha_a - alpha_b
            beta = np.pi / 2 - alpha

            stretch_factor = np.linalg.norm(a_to_b)
            radius = 0.5 * stretch_factor / np.cos(beta)

            circle_o = np.tan(beta) * np.array((-a_to_b[1], a_to_b[0]))

            circle_world = a_bal.position + 0.5 * (a_to_b + circle_o)

            centers.append(circle_world)
            radii.append(radius)

        centers = self.centers = np.array(centers)
        radii = self.radii = np.array(radii)

        # we derived a set of linear equations by subtracting the first circle equation from the rest
        # thereby eliminating the quadratic terms

        # one of the circle equations is the base that gets subtracted from the others
        base_center, other_centers = centers[0], centers[1:]
        base_radii, other_radii = radii[0], radii[1:]

        a = 2 * (other_centers - base_center)
        b = np.sum(other_centers ** 2 - base_center ** 2, axis=-1) + (base_radii ** 2 - other_radii ** 2)

        # find least-squares solution for the linear equation system
        xy, self.residual = np.linalg.lstsq(a, b)[:2]

        return xy

    def calculate_best_position(self, img, max_residual=.5, widen_mask_iter=(0, 14, 17), max_iters=1):
        """ tries to adjust the mask parameters until a solution with a low residual is found """

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0].astype(np.float)

        widen_mask = np.array((0, 0, 0))
        for i in xrange(max_iters):
            self.xy = self.calculate_position(hsv, hue, widen_mask)
            if self.xy is not None and not (self.residual > max_residual):
                self.iterations = i
                return self.xy
            widen_mask += widen_mask_iter

        self.iterations = None
        return None

    @staticmethod
    def angle_mean(angles):
        return np.arctan2(np.sin(angles).sum(), np.cos(angles).sum())

    def calculate_angle(self):
        """ calculates the yaw angle based on the last parsed image """
        full_circle = 2 * np.pi

        angles = [vector_to_angle(balloon.position - self.xy) + angle
                  for balloon, angle in self.balloon_angles.items()]
        return BalloonDetector.angle_mean(angles) - np.pi / 2  # median of angle offsets for each detected balloon

    def res_fun(self, xy):
        """ returns the true residual of the specified solution """
        return (np.sum((self.centers - xy) ** 2, axis=-1) - self.radii ** 2) ** 2

    def compare_nonlinear(self, xy):
        """ compare the linear solution to the nonlinear one and return the optimized position """
        # find least-squares solution for the non linear function
        opt = scipy.optimize.least_squares(self.res_fun, xy)

        res_linear, res_nonlinear = self.res_fun(xy).sum(), self.res_fun(opt.x).sum()
        print('residuals: [linear: %8.5f, nonlinear: %8.5f], relative residual: %.4f, dist xy: %s' %
              (res_linear, res_nonlinear, res_nonlinear / res_linear, np.linalg.norm(xy - opt.x)))

        return opt.x

    def draw_markers(self, img):
        """ draw markers for the last detected positions """
        max_y, max_x, _ = img.shape

        # draw coordinate system
        cv2.line(img, (max_x / 2, 0), (max_x / 2, max_y), (255, 255, 255))
        cv2.line(img, (0, max_y / 2), (max_x, max_y / 2), (255, 255, 255))

        # mark position with a cross
        for balloon, pos in self.balloon_positions.items():
            draw_cross(img, pos, balloon.name, balloon.hue)

        if self.iterations is not None:
            label = 'iteration %d, residual: %s' % (self.iterations + 1, self.residual)
            draw_text(img, label, (10, max_y - 20), (0, 255, 0))


def draw_cross(img, pos, label, hue, l_width=1, l_len=8):
    """ draw a cross at the specified position on the image as well as a label """
    pos = np.round(np.asarray(pos)).astype(np.int)
    off_x, off_y = [l_len, 0], [0, l_len]
    # for the line / text color convert from hue to bgr
    color = tuple(cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR)[0, 0].astype(np.int))
    cv2.line(img, tuple(pos - off_x), tuple(pos + off_x), color, l_width)
    cv2.line(img, tuple(pos - off_y), tuple(pos + off_y), color, l_width)
    draw_text(img, label, tuple(pos + [l_len + 3, 3]), color)


def draw_text(img, text, pos, color, font=cv2.FONT_HERSHEY_SIMPLEX, f_size=0.5, line=cv2.LINE_AA):
    cv2.putText(img, text, pos, font, f_size, color, lineType=line)


def angle_diff(angle_a, angle_b, half_circle=90):
    """ computes the minimum angle distance between two hue angles in range [0, 180) """
    return half_circle - np.abs(np.abs(angle_a - angle_b) - half_circle)


def vector_to_angle(vec):
    """ find the angle to the x axis of a vector """
    return np.arctan2(vec[1], vec[0])


# for testing: a main method
def main(img_fname, use_mpl=True, show_cv2=True, block=True):
    np.set_printoptions(precision=5, suppress=True)  # less verbose numpy printing

    if use_mpl:
        import matplotlib.pyplot as plt
        from matplotlib.colors import hsv_to_rgb

        axes = list(plt.subplots(1, 3, num='balloon detector: %s' % img_fname,
                                 figsize=(20, 5), facecolor='w')[1].reshape(-1))

        ax = axes.pop()
        ax.set_title('world coordinate triangulation')
        for b in balloons:
            ax.plot(*b.position, color=hsv_to_rgb((b.hue / 180.0, 1, 1)), marker='o', markersize=10)

        # dynamically grabs a new subplot from the available ones
        def show_image(img, title, cmap=None):
            """ shows an image in matplotlib, either with the specified color map or converted to RGB """
            ax = axes.pop(0)
            ax.set_title(title)
            ax.axis('off')
            if cmap:
                ax.imshow(img, cmap)
            else:
                ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    elif show_cv2:
        def show_image(img, title, cmap=None):
            """ shows an image in cv2 """
            cv2.imshow(title, img)
    else:
        def show_image(i, title, cmap=None):
            pass

    img = cv2.imread(img_fname)

    if img is None:
        print('could not read image at %s' % img_fname)
        sys.exit(1)

    # show_image(img, 'source image')

    detector = BalloonDetector()
    xy = detector.calculate_best_position(img)
    angle = detector.calculate_angle() if xy is not None else 0
    print('Position in world coordinates: %s, angle: %6.2f, residual: %s' %
          (xy, np.rad2deg(angle), detector.residual))

    show_image(detector.mask, 'mask', 'gray')
    detector.draw_markers(img)
    show_image(img, 'cluster midpoints')

    if use_mpl:
        if detector.centers is not None:
            for center, radius in zip(detector.centers, detector.radii):
                ax.plot(*center, marker='x')
                ax.add_artist(plt.Circle(center, radius, fill=False, alpha=0.3))

        if xy is not None:
            x, y = xy
            ax.plot(x, y, marker='*', alpha=0.6)

            arrow_size = 0.2
            ax.arrow(x, y, np.sin(angle + np.pi / 2) * arrow_size, -np.cos(angle + np.pi / 2) * arrow_size,
                     head_width=0.05, head_length=0.1, fc='k', ec='k')

        scale = 1.25 * max(balloon_x_dist, balloon_y_dist)
        x_orig = balloon_x_base + balloon_x_dist / 2
        y_orig = balloon_y_base + balloon_y_dist / 2
        ax.set_xlim((x_orig - scale, x_orig + scale))
        ax.set_ylim((y_orig - scale, y_orig + scale))
        ax.set_aspect(1, 'datalim')  # keep circles as circles
        plt.tight_layout()
        plt.show(block=block)

    elif show_cv2 and block:
        modifiers = {225, 227, 231, 233}
        while cv2.waitKey(0) in modifiers:  # ignore modifier keys
            print('ignoring modifier')
        cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys

    main(sys.argv[1] if len(sys.argv) > 1 else 'img.png')
