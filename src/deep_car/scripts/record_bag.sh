#! /usr/bin/env sh

rosbag record  \
	/manual_control/speed \
	/manual_control/steering \
	/model_car/yaw \
    /deepcar/resize_img80x60/compressed
