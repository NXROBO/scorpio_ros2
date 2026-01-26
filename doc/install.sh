echo 'Scorpio driver is installing'

echo 'Setting udev rules'
BASEPATH=$(cd `dirname $0`; pwd)
sudo cp $BASEPATH/rules/scorpio-usb-serial.rules /etc/udev/rules.d/
sudo $BASEPATH/rules/eai_gx.sh
sudo udevadm trigger

echo 'Scorpio driver is installed'

