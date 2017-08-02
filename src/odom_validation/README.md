## Odometry Validation Package

This package helps to validate the odometry of the car using external camera. 

### Initial Setup. 
* Mount camera on ceiling facing downwards. 
* Attach red square on the car in center of rear axle such that it is visible all the time from ceiling camera.
* Measure lenth of camera coverage and calculate the pixels per meter. It is currently 279
* Currently car is placed on width axis of camera facing the (0,0) pixel. Please adjust code in lines (97-100) of [odom_camera.py](https://github.com/AutoModelCar/catkin_ws_user/tree/master/src/odom_validation/src/odom_camera.py) if the initial orientation of car is different. 

### Run

Run code:
    Use [usb_cam](https://github.com/AutoModelCar/model_car/tree/version-1/catkin_ws/src/usb_cam) package to obtain images from the ceiling camers. 
 
Note: Do not run any other camera(realsense) while using for odometry validation, you may have images from different sources to process

To record odometry values calculated from the ceiling camera:

rosrun odom_validation odom_camera.py

To record odometry from car:

rosrun odom_validation odom_car.py

The above two programs will save coordinates to odom_camera.txt and odom_car.txt files. Use python script 'graph_plotter.py' in src folder to plot them in 3D. 
Note: Update text files names and path in the graph_plotter.py script. Reference text files are provided in the the src folder.

    
### Trouble shooting:

### Adjust the focus and other parameters of the camera for proper image
v4l2-ctl -d 1 --set-ctrl focus_auto=0
v4l2-ctl -d 1 --set-ctrl focus_absolute=0
v4l2-ctl -d 1 --set-ctrl contrast=0
v4l2-ctl -d 1 --set-ctrl brightness=0

If the timestamps in odometry of camera and car does not match, synchronize both the clocks. 
