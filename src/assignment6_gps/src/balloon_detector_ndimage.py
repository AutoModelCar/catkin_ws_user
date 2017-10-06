#!/usr/bin/env python2
import functools
import numpy as np
from collections import namedtuple
from itertools import combinations
from time import clock

import cv2
import scipy.ndimage
import scipy.optimize

# data class for balloons (color specified as the hue from HSV)
Balloon = namedtuple('Balloon', ['name', 'hue', 'position'])

# estimates for balloon positions and hues
# hue is divided by two since opencv uses hues in ranges [0, 180)
balloon_x_base, balloon_x_dist = 2.33, 1.24
balloon_y_base, balloon_y_dist = 1.15, 1.85
balloons = (Balloon('lilac', 260 / 2, (balloon_x_base, balloon_y_base + balloon_y_dist)),
            Balloon('red', 354 / 2, (balloon_x_base + balloon_x_dist, balloon_y_base + balloon_y_dist)),
            Balloon('blue', 235 / 2, (balloon_x_base + balloon_x_dist, balloon_y_base)),
            Balloon('green', 130 / 2, (balloon_x_base, balloon_y_base)),)

# a balloon hues in radians with shape (1, 4)
# angles in range [-pi, pi)
balloon_hues = np.pi / 90 * ((np.array([[balloon.hue for balloon in balloons]], dtype=np.float32) + 90) % 180 - 90)


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
    def __init__(self, min_diameter=3, max_diameter=15, min_value_percentile=99, close_iter=10,
                 hsv_range_min=(0, 77, 99), hsv_range_max=(181, 255, 255)):
        self.min_diameter = min_diameter
        self.max_diameter = max_diameter
        self.min_value_percentile = min_value_percentile
        self.hsv_range_min = np.array(hsv_range_min)
        self.hsv_range_max = np.array(hsv_range_max)

        self.close_iter = close_iter

        # the following attributes are set during detection
        self.balloon_angles = None
        self.balloon_positions = None
        self.mask = None
        self.centers = None
        self.radii = None
        self.xy = None
        self.residual = np.inf

    def detect_balloons(self, hsv, hue_radians, widen_value_mask=0):
        """
        detect balloons in the image and draw crosses on the detected points
        :param widen_value_mask: how much to widen the range of values that are considered
        """

        range_min = self.hsv_range_min
        range_min[2] = int(np.percentile(hsv[:, :, 2], self.min_value_percentile - widen_value_mask))

        mask = cv2.inRange(hsv, range_min, self.hsv_range_max)

        self.mask = mask  # save the mask so the caller can plot it if desired

        if self.close_iter > 0:
            mask = scipy.ndimage.binary_closing(mask, iterations=self.close_iter)

        labelled, nfeatures = scipy.ndimage.label(mask)  # find connected components and assigns a label to each of them

        if nfeatures < 1:
            print('did not detect enough features')
            return {}

        selected = []  # selected labels
        hues = []
        centers = []
        size_min, size_max = self.min_diameter, self.max_diameter

        for label, yx in enumerate(scipy.ndimage.find_objects(labelled), 1):
            y, x = yx
            w, h = x.stop - x.start, y.stop - y.start
            if size_min <= w <= size_max and size_min <= h <= size_max:
                selected.append(label)

                labelled_masked = labelled[yx]

                hue_radians_masked = hue_radians[yx][labelled_masked == label]

                hue = np.arctan2(np.sin(hue_radians_masked).sum(), np.cos(hue_radians_masked).sum())

                hues.append(hue)

                # center = scipy.ndimage.center_of_mass(np.ones(labelled_masked.shape), labelled_masked, label)
                centers.append((y.start + h * .5, x.start + w * .5))

                # cv2.rectangle(img, (x.start, y.start), (x.stop, y.stop), (0, 255, 0))

        if len(selected) < 1:
            print('did not find enough features with the desired size')
            return {}

        balloon_hue_dist = angle_diff(np.array(hues).reshape(len(hues), 1), balloon_hues)

        # map from each balloon to the best angle
        balloon_angles = {}
        balloon_positions = {}
        max_y, max_x, _ = hsv.shape
        origin = np.array((max_y, max_x)) * .5

        valid_rows = set(range(len(selected)))

        # iterate over balloons in the order of minimal hue distance
        while valid_rows and len(balloon_angles) < len(balloons):
            row, col = np.unravel_index(balloon_hue_dist.argmin(), balloon_hue_dist.shape)

            balloon = balloons[col]

            pos = centers[row]
            balloon_positions[balloon] = pos

            # calculate the angle from the center of the picture
            y, x = pos - origin
            balloon_angles[balloon] = np.arctan2(-y, x)

            balloon_hue_dist[row] = np.inf  # set hue values in this row to inf to prevent further matches
            balloon_hue_dist[:, col] = np.inf  # prevent getting this balloon again
            valid_rows.remove(row)

        self.balloon_positions = balloon_positions
        return balloon_angles

    def calculate_position(self, hsv, hue_radians, widen_value_mask, max_residual=1.0):
        """ :returns the position of the car in world coordinates or None if it isn't available """
        balloon_angles = self.detect_balloons(hsv, hue_radians, widen_value_mask)
        self.balloon_angles = balloon_angles

        if len(balloon_angles) < 3:  # not enough balloons detected
            return None

        xy, self.residual = self.calculate_position_from_angles(balloon_angles.items())

        if self.residual > max_residual and len(balloon_angles) > 3:
            # try to find the best 3 subset
            for subset in combinations(balloon_angles.items(), 3):
                xy, self.residual = self.calculate_position_from_angles(subset)
                if self.residual < max_residual:
                    return xy

        return xy

    def calculate_position_from_angles(self, balloon_angles):
        centers = []
        radii = []

        for (a_bal, alpha_a), (b_bal, alpha_b) in combinations(balloon_angles, 2):
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
        return np.linalg.lstsq(a, b)[:2]

    # @timed
    def calculate_best_position(self, img, max_residual=1.0, max_iters=5):
        """ tries to adjust the mask parameters until a solution with a low residual is found """

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0].astype(np.float)
        hue_radians = np.pi / 90 * hue

        for i in xrange(max_iters):
            self.xy = self.calculate_position(hsv, hue_radians, i)
            if self.xy is not None and not (self.residual > max_residual):
                self.draw_markers(img, i)  # draw markers for successful attempt
                return self.xy

        self.draw_markers(img, max_iters)  # draw markers for the last (unsuccessful iteration)
        return None

    def calculate_angle(self):
        """ calculates the yaw angle based on the last parsed image """
        full_circle = 2 * np.pi
        angles = [(vector_to_angle(balloon.position - self.xy) + angle + full_circle) % full_circle
                  for balloon, angle in self.balloon_angles.items()]
        return np.median(angles) - np.pi / 2  # median of angle offsets for each detected balloon

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

    def draw_markers(self, img, i):
        """ draw markers for the last detected positions """
        max_y, max_x, _ = img.shape

        # draw coordinate system
        cv2.line(img, (max_x / 2, 0), (max_x / 2, max_y), (255, 255, 255))
        cv2.line(img, (0, max_y / 2), (max_x, max_y / 2), (255, 255, 255))

        # mark position with a cross
        for balloon, pos in self.balloon_positions.items():
            draw_cross(img, pos[::-1], balloon.name, balloon.hue)

        draw_text(img, 'iteration %d, residual: %s' % (i + 1, self.residual), (10, max_y - 20), (0, 255, 0))


def draw_cross(img, pos, label, hue, l_width=1, l_len=8, font=cv2.FONT_HERSHEY_SIMPLEX, f_size=0.5):
    """ draw a cross at the specified position on the image as well as a label """
    pos = np.round(np.asarray(pos)).astype(np.int)
    off_x, off_y = [l_len, 0], [0, l_len]
    # for the line / text color convert from hue to bgr
    color = tuple(cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR)[0, 0].astype(np.int))
    cv2.line(img, tuple(pos - off_x), tuple(pos + off_x), color, l_width)
    cv2.line(img, tuple(pos - off_y), tuple(pos + off_y), color, l_width)
    draw_text(img, label, tuple(pos + [l_len + 3, 3]), color)


def draw_text(img, text, pos, color, font=cv2.FONT_HERSHEY_SIMPLEX, f_size=0.5, lineType=cv2.LINE_AA):
    cv2.putText(img, text, pos, font, f_size, color, lineType=lineType)


def angle_diff(angle_a, angle_b, half_circle=np.pi):
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
