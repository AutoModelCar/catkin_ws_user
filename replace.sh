echo "replace root path"
var=$(pwd)
ws_name=$(basename $var)
tmpfile=/tmp/magicReplaceTempFile
target=root@192.168.1.199:./$ws_name/
echo "The workspace directory is $var."


echo rsync -r  $var/src $target
echo rsync -r $var/odroid-devel $target
echo rsync -r $var/odroid-build $target

sed 's#'$var'#/root#'g $var/odroid-devel/_setup_util.py > $tmpfile
sed -i 's#/opt/odroid-x2/sdk/#/#'g $tmpfile
sed -i 's#/catkin_ws/#/'$ws_name'/#'g $tmpfile
sed -i 's#/opt/ros/indigo;//opt/ros/indigo#/opt/ros/indigo;/root/catkin_ws/devel/#'g $tmpfile
echo rsync $tmpfile $target/odroid-devel/_setup_util.py

sed 's#'$var'#/root/'$ws_name'#'g $var/odroid-build/.catkin_tools.yaml > $tmpfile
echo rsync $tmpfile $target/odroid-build/.catkin_tools.yaml

sed 's#'$var'#/root/'$ws_name'#'g $var/odroid-devel/.catkin > $tmpfile
echo rsync $tmpfile $target/odroid-devel/.catkin

sed 's#'$var'#/root/'$ws_name'#'g $var/odroid-devel/setup.sh > $tmpfile
echo rsync $tmpfile $target/odroid-devel/setup.sh

sed 's#'$var'#/root/'$ws_name'#'g $var/odroid-devel/.rosinstall > $tmpfile
echo rsync $tmpfile $target/odroid-devel/.rosinstall
echo "finished"
