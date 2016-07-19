echo "replace root path"
pwd
cd ..
var=$(pwd)
echo "The catkin_ws_user directory $var."
sed -i 's#'$var'#/root#'g $var/catkin_ws_user/odroid-devel/.catkin $var/catkin_ws_user/odroid-devel/setup.sh $var/catkin_ws_user/odroid-devel/_setup_util.py $var/catkin_ws_user/odroid-devel/.rosinstall
sed -i 's#/opt/odroid-x2/sdk/#/#'g $var/catkin_ws_user/odroid-devel/_setup_util.py
sed -i 's#'$var'#/root#'g $var/catkin_ws_user/odroid-build/.catkin_tools.yaml
scp -r $var/catkin_ws_user/src/ root@192.168.1.199:./catkin_ws_user/
scp -r $var/catkin_ws_user/odroid-devel/ root@192.168.1.199:./catkin_ws_user/
echo "finished"
