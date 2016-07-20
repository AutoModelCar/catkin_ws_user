

echo "replace root path"
var=$(pwd)
tmpfile=/tmp/magicReplaceTempFile
target=root@192.168.1.199:./catkin_ws_user/
echo "The catkin_ws_user directory is $var."


rsync -r  $var/src $target
rsync -r $var/odroid-devel $target
rsync -r $var/odroid-build $target

sed 's#'$var'#/root#'g $var/odroid-devel/_setup_util.py > $tmpfile
sed -i 's#/opt/odroid-x2/sdk/#/#'g $tmpfile
sed -i 's#/catkin_ws/catkin_ws_user/#'g $tmpfile
sed -i 's#/opt/ros/indigo;//opt/ros/indigo#/opt/ros/indigo;/root/catkin_ws_user/devel/#'g $tmpfile
rsync $tmpfile $target/odroid-devel/_setup_util.py

sed 's#'$var'#/root#'g $var/odroid-build/.catkin_tools.yaml > $tmpfile
rsync $tmpfile $target/odroid-build/.catkin_tools.yaml

sed 's#'$var'#/root#'g $var/odroid-devel/.catkin > $tmpfile
rsync $tmpfile $target/odroid-devel/.catkin

sed 's#'$var'#/root#'g $var/odroid-devel/setup.sh > $tmpfile
rsync $tmpfile $target/odroid-devel/setup.sh

sed 's#'$var'#/root#'g $var/odroid-devel/.rosinstall > $tmpfile
rsync $tmpfile $target/odroid-devel/.rosinstall
echo "finished"

