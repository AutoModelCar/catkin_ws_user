#!/usr/bin/env python2

import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from collections import deque

import balloon_detector


def main():
    if len(sys.argv) != 2:
        print('need a directory of pictures as the first argument')
        sys.exit(1)

    np.set_printoptions(precision=2, suppress=True)  # less verbose numpy printing

    folder = sys.argv[1]
    fnames = deque(sorted(fname for fname in os.listdir(folder) if fname.endswith('png') or fname.endswith('jpg')))

    def event_handler(event):
        if event.key == 'n':
            show_next(-1)
        elif event.key == 'p':
            show_next(1)
        elif event.key == 'q':
            plt.close()

    def show_next(rotate_dir):
        plt.close()
        fnames.rotate(rotate_dir)
        balloon_detector.main(os.path.join(folder, fnames[0]), True, False, False)
        plt.gcf().canvas.mpl_connect('key_press_event', event_handler)

    show_next(0)
    plt.show()


if __name__ == '__main__':
    main()
