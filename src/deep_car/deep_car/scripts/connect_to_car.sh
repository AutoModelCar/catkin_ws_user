# run with $ source connect_to_car.sh

my_ip=$(hostname -i)
export DISPLAY
export QT_X11_NO_MITSHM=1
export ROS_MASTER_URI=http://192.168.43.102:11311
export ROS_IP=${my_ip}
export ROS_HOSTNAME=${my_ip}