#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

#=================================================
#	System Required: Ubuntu 22.04
#        litian.zhuang@nxrobo.com
#	Site: http://www.nxrobo.com/
#    scorpio技术讨论与反馈QQ群：6646169  8346256
#=================================================

GAME_ENABLE="yes"
sh_ver="2.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"
password="scorpio"
Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)
calibra_default="${filepath}/../.ros/camera_info"


calibration="calibration"
color_block="color_block"
# 配置文件的路径
CONFIG_FILE="/opt/nxrobo/device_config.yaml"

check_motor_offset(){
	# 检查文件是否存在
	if [ ! -f "$CONFIG_FILE" ]; then
		echo -e "${Error} 检测到前轮的偏移量未校准！"
	fi
}


#检查系统要求
check_sys(){
        if [[ "${Version}" == "20.04" ]]; then
                ROS_Ver="foxy"
        elif [[ "${Version}" == "22.04" ]]; then
                ROS_Ver="humble"
        else
                echo -e "${Error} 当前系统 ${OSDescription} ,此版本暂未被适配!" && exit 1
        fi
}

#检查设备连接
check_dev(){
	#检查摄像头
	check_camera
	#检查雷达
	check_lidar

	check_motor_offset
}

#检查雷达设备
check_lidar(){
	BASEPATH=$(cd `dirname $0`; pwd)
	TYPE_LIDAR=$(cat /opt/lidar.txt)

	if [[ "${TYPE_LIDAR}" == "ydlidar_g6" ]]; then
		LIDARTYPE="ydlidar_g6"
	elif [[ "${TYPE_LIDAR}" == "ydlidar_g2" ]]; then
		LIDARTYPE="ydlidar_g2"
	elif [[ "${TYPE_LIDAR}" == "ydlidar_g4" ]]; then
		LIDARTYPE="ydlidar_g4"
	else
		echo "暂不支持的雷达：${TYPE_LIDAR}，使用默认的EAI-G6雷达运行"
		LIDARTYPE="ydlidar_g6"
	fi
	lidar_flag=0

	#检查使用哪种设备
	if [ -n "$(lsusb -d 10c4:ea60)" ]; then
		lidar_flag=$[$lidar_flag + 1]
	fi

	if [ $lidar_flag -ge 2 ]; then
		echo -e "${Warn} 正在使用多个雷达设备，请退出并拔掉其中一个再使用!"
		echo -e "${Warn} 退出请输入：Ctrl + c！"
	elif [ $lidar_flag -eq 1 ]; then
		echo -e "${Info} 正在使用${LIDARTYPE}雷达"
	elif [ $lidar_flag -eq 0 ]; then
		echo -e "${Error} 没有找到雷达，请确认雷达已正确连接！！"
	fi	
}
#检查摄像头设备
check_camera(){

	camera_flag=0
	calibra_backup="/opt/nxrobo/camera_info"

	#检查使用哪种设备
	if [ -n "$(lsusb -d 2bc5:0403)" ]; then
		CAMERATYPE="astrapro"

		camera_flag=$[$camera_flag + 1]
	fi
	if [ -n "$(lsusb -d 2bc5:0401)" ]; then
		CAMERATYPE="astra"

		camera_flag=$[$camera_flag + 1]
	fi
	if [ -n "$(lsusb -d 8086:0b07)" ]; then
		CAMERATYPE="d435"

		camera_flag=$[$camera_flag + 1]
	fi

	if [ $camera_flag -ge 2 ]; then
		echo -e "${Warn} 正在使用多个摄像头设备，请退出并拔掉其中一个再使用!"
		echo -e "${Warn} 退出请输入：Ctrl + c！"
	elif [ $camera_flag -eq 1 ]; then
		echo -e "${Info} 正在使用${CAMERATYPE}摄像头"
	elif [ $camera_flag -eq 0 ]; then
		echo -e "${Error} 没有找到摄像头，请确认摄像头已正确连接！！"
	fi	
	
	#根据摄像头类型检查标定文件和备份文件
	case $CAMERATYPE in
		astrapro) 
			#检查是否有默认文件夹和备份文件夹
			if [ ! -d "$calibra_default" ]; then
				mkdir -p "$calibra_default"
			fi

			if [ ! -d "$calibra_backup" ]; then
				echo -e "${Info} 创建标定文件备份文件夹..."
				sudo mkdir  -p "$calibra_backup"
			fi
			#如果没有就拷贝一份
			if [ ! -f "$calibra_backup/camera.yaml" ]||[ ! -f "$calibra_backup/depth_Astra_Orbbec.yaml" ]; then
				if [ -f "$calibra_default/camera.yaml" ]&&[ -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Info} 备份标定文件..."
					sudo cp -r "$calibra_default/." "$calibra_backup/."
				elif [ ! -f "$calibra_default/camera.yaml" ]||[ ! -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Warn} 标定文件缺少其中一个，请确认rgb和depth标定文件都存在"
				fi
			elif [ -f "$calibra_backup/camera.yaml" ]&&[ -f "$calibra_backup/depth_Astra_Orbbec.yaml" ]; then
				if [ ! -f "$calibra_default/camera.yaml" ]; then
					echo -e "${Info} 缺少彩色摄像头标定文件，已拷贝"
					cp "$calibra_backup/camera.yaml" $calibra_default
				fi
				if [ ! -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Info} 缺少深度摄像头标定文件，已拷贝"
					cp "$calibra_backup/depth_Astra_Orbbec.yaml" $calibra_default	
				fi
			fi
			;;
		astra)
			#astra自带标定文件			
			#如果检查到有标定文件就删除
			if [ -d "$calibra_default" ]; then
				rm -rf "$calibra_default"
			fi
			;;
	esac

}

#清除所有ROS节点。
killall_ros_nodes(){
	print_command "Kill all ros nodes!"
	ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
	exit 0
}

#编译scorpio
install_scorpio(){
	source /opt/ros/${ROS_Ver}/setup.bash
	colcon build
}



print_command()
{
	echo -e "${Yellow_background_prefix}${Red_font_prefix}${1}${Font_color_suffix}"
}

#让机器人动起来
let_robot_go(){
	echo -e "${Info}" 
	echo -e "${Info}      让机器人动起来" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}    q左前   w前进  e右前    "
	echo -e "${Info}            s后退          "
	echo -e "${Info}    z左后          c右后   " 
	echo -e "${Info}                           " 
	echo -e "${Info}    退出请输入：Ctrl + c    " 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	print_command "ros2 launch scorpio_teleop teleop.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} "
	ros2 launch scorpio_teleop teleop.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
}




#让scorpio跟着你走
people_follow(){
	echo -e "${Info}                  " 
	echo -e "${Info}让scorpio跟着你走" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}                  " 
	echo -e "${Info}请站在scorpio的正前方，与scorpio保持一米左右的距离，然后走动"
	echo -e "${Info}                  " 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	print_command "ros2 launch scorpio_follower scorpio_follower.launch.py camera_type_tel:=${CAMERATYPE}"
	ros2 launch scorpio_follower scorpio_follower.launch.py camera_type_tel:=${CAMERATYPE} 
}


#让scorpio使用激光雷达进行导航
scorpio_navigation_2d(){
	echo -e "${Info}" 
	echo -e "${Info}让scorpio使用激光雷达进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}       A.激光雷达已上电连接"
	echo -e "${Info}       B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
	echo -e "${Info}       C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，机器人将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase '^?' && read -p "按回车键（Enter）开始：" 
	print_command "ros2 launch scorpio_navigation2 start_scorpio_navigation2.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}"
	ros2 launch scorpio_navigation2 start_scorpio_navigation2.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
}
#让scorpio使用深度摄像头进行导航
scorpio_navigation_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让scorpio使用深度摄像头进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}请选择导航方式："
	echo -e "${Info}1.使用rtab_map地图"
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" slamnum

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}A.摄像头已连接"
	case "$slamnum" in
		1)
		SLAMTYPE="rtab_map"
		echo -e "${Info}B.把机器人放到原来建图的原点。导航正常启动后，如需查看原来建立的３Ｄ地图，点击rviz的Display->Rtabmap cloud->Download map加载３Ｄ地图。"
		;;
		*)
		echo -e "${Error} 错误，默认使用rtab_map地图"
		SLAMTYPE="rtab_map"
		echo -e "${Info}B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
		;;
	esac

	echo -e "${Info}C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，机器人将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	if [[ "${SLAMTYPE}" == "rtab_map" ]]; then
		print_command "ros2 launch scorpio_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} localization:='true'"	
		ros2 launch scorpio_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} localization:='true'

	fi	
}

#深度学习
scorpio_DeepLearn(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	#source ${PROJECTPATH}/tensorflow/bin/activate
	source ${PROJECTPATH}/install/setup.bash	
	export PYTHONPATH=$PYTHONPATH:~/models/research
	print_command "${PROJECTPATH}/install/setup.bash	"
	echo -e "${Info}运行深度学习的方式：
	1.软件tensorflow
	2.软件yolov8,物品识别
	3.软件yolov8,肢体识别"
	echo && stty erase ^? && read -p "请选择 1 , 2 或 3 ：" chnum
 	case "$chnum" in
		1)
		print_command "ros2 launch ros2_tensorflow scorpio_tensorflow_launch.py camera_type_tel:=${CAMERATYPE}"	
		ros2 launch ros2_tensorflow scorpio_tensorflow_launch.py camera_type_tel:=${CAMERATYPE}		
		;;
		2)
		print_command "ros2 launch scorpio_yolov8 scorpio_yolo_object.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}"
		ros2 launch scorpio_yolov8 scorpio_yolo_object.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
		;;
		3)
		print_command "ros2 launch scorpio_yolov8 scorpio_yolo_pose.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}"
		ros2 launch scorpio_yolov8 scorpio_yolo_pose.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
		;;
		*)
		echo -e "${Error} 错误，将运行默认的软件tensorflow"
		print_command "ros2 launch ros2_tensorflow scorpio_tensorflow_launch.py camera_type_tel:=${CAMERATYPE}"	
		ros2 launch ros2_tensorflow scorpio_tensorflow_launch.py camera_type_tel:=${CAMERATYPE}		
		;;
	esac
}


#让scorpio使用激光雷达绘制地图(gmapping)
scorpio_build_map_2d(){
	echo -e "${Info}" 
	echo -e "${Info}让scorpio使用激光雷达绘制地图" 
	echo -e "${Info}" 
	echo -e "${Info}请选择SLAM的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} gmapping
	  ${Green_font_prefix}2.${Font_color_suffix} cartographer
	  ${Green_font_prefix}3.${Font_color_suffix} slam_toolbox
	  ${Green_font_prefix}4.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-4]：" slamnum
	case "$slamnum" in
		1)
		SLAMTYPE="gmapping"
		;;
		2)
		SLAMTYPE="cartographer"
		;;
		3)
		SLAMTYPE="slam_toolbox"
		;;
		*)
		echo -e "${Error} 错误，默认使用gmapping"
		SLAMTYPE="gmapping"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}    q左前   w前进  e右前    "
	echo -e "${Info}            s后退          "
	echo -e "${Info}    z左后          c右后   " 
	echo -e "${Info}                           " 
	echo -e "${Info}退出请输入：Ctrl + c        " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	print_command "ros2 launch scorpio_slam_transfer start_build_map_${SLAMTYPE}.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} "
	ros2 launch scorpio_slam_transfer start_build_map_${SLAMTYPE}.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} 	
}



scorpio_test_mode(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	ROSVER=`/usr/bin/rosversion -d`
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}老化测试程序，请将机器人放在一个50X50CM的方格中间进行测试，请选择：
	1.执行5分钟的检测功能；
        2.执行老化检测功能，直至断电。"
        echo -e "${Info}退出请输入：Ctrl + c " 
	echo && stty erase ^? && read -p "请选择 1 或 2 ：" chnum
 	case "$chnum" in
		1)
		print_command "ros2 launch scorpio_test scorpio_test_five_minute.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}"
		ros2 launch scorpio_test scorpio_test_five_minute.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
		;;
		2)
		print_command "ros2 launch scorpio_test scorpio_test_aging.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}"
		ros2 launch scorpio_test scorpio_test_aging.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
		;;
		*)
		echo -e "${Error} 退出!"	
		;;
	esac
}

#让scorpio使用深度摄像头绘制地图
scorpio_build_map_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让scorpio使用深度摄像头绘制地图" 
	echo -e "${Info}"
	echo -e "${Info}请选择SLAM的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} rtab_map
	  ${Green_font_prefix}2.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-5]：" slamnum
	case "$slamnum" in
		1)
		SLAMTYPE="rtab_map"
		;;
		*)
		echo -e "${Error} 错误，默认使用rtab_map"
		SLAMTYPE="rtab_map"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}    q左前   w前进  e右前    "
	echo -e "${Info}            s后退          "
	echo -e "${Info}    z左后          c右后   " 
	echo -e "${Info}                           " 
	echo -e "${Info}退出请输入：Ctrl + c        " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	if [[ "${SLAMTYPE}" == "rtab_map" ]]; then
		echo -e "${Tip}" 
		echo -e "${Tip}现在使用rtab_map建图，将会删除之前保存的地图，选择‘y’继续建图，其它键直接退出。" 
		echo -e "${Tip}" 
		echo && stty erase ^? && read -p "请选择是否继续y/n：" choose
		if [[ "${choose}" == "y" ]]; then
			print_command "ros2 launch scorpio_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} localization:='false'"	
			ros2 launch scorpio_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} localization:='false'
		else
			return
		fi
	fi
	
}


qrcode_transfer_files(){
	eth_ip=`/sbin/ifconfig eth0|grep 'inet '|awk '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep 'inet '|awk '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep 'inet '|awk '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep 'inet '|awk '{print $2}'`
	enp3s_ip=`/sbin/ifconfig enp3s0|grep 'inet '|awk '{print $2}'`
	wlp0s_ip=`/sbin/ifconfig wlp0s20f3|grep 'inet '|awk '{print $2}'`
	wlo1_ip=`/sbin/ifconfig wlo1|grep 'inet '|awk '{print $2}'`
	if [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp1s0" 
	  	net_interface="wlp1s0"
	elif [ $wlo1_ip ]; then
		echo -e "${Info}使用无线网络wlo1" 
	  	net_interface="wlo1"	  	
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	net_interface="wlp2s0"
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	net_interface="wlan0"
	elif [ $enp3s_ip ]; then
		echo -e "${Info}使用无线网络enp3s0" 
		net_interface="enp3s0"
	elif [ $wlp0s_ip ]; then
		echo -e "${Info}使用无线网络wlp0s20f3" 
		net_interface="wlp0s20f3"		
	elif [ $eth_ip ]; then
		echo -e "${Info}使用有线网络eth0" 
		net_interface="eth0"
	fi
	echo -e "${Info}" 
	echo -e "${Info}通过局域网收发文件" 
	echo -e "${Info}" 
	echo -e "${Info}请选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 发送文件（文件名，带上文件绝对路径）
	  ${Green_font_prefix}2.${Font_color_suffix} 接收文件（默认存放在~/Downloads路径中）
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" cnum
	case "$cnum" in
		1)
		echo -e "${Info}请输入文件名，带上文件绝对路径，如 /home/${USER}/a.jpg：
		 退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入要发送的文件：" s_file
		if [ -f "$s_file" ]; then
			echo -e "${Info}本机即将发送文件：${Green_font_prefix}"$s_file"${Font_color_suffix}，请接收端扫码或者直接输入下面的网址接收文件"
		else 
			echo -e "${Info}请输入带绝对路径的文件名"
			exit
		fi
		
		qrcp send  -i $net_interface $s_file
		;;
		2)
		echo -e "${Info}请输入接收到的文件存放的路径，默认为 /home/${USER}/Downloads：
		退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入文件存放的文件夹路径：" s_file
		if [ -d "$s_file" ]; then
			echo ""
		else 
			echo -e "${Info}${Red_font_prefix}文件夹不存在，将存放在默认文件夹/home/${USER}/Downloads中${Font_color_suffix}"
			s_file="/home/${USER}/Downloads"
		fi
		echo -e "${Info}接收的文件将存放在：${Green_font_prefix}"$s_file"${Font_color_suffix}，目录下，请发送端扫码或者直接输入下面的网址选择文件发送"
		qrcp  -i $net_interface receive --output=$s_file
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

coming_soon(){
	echo -e "${Tip} coming_soon!" 
}


#printf
menu_status(){
	echo -e "${Tip} 当前系统版本 ${OSDescription} !" 
	echo -e "${Tip} 当前ROS版本 ${ROS_DISTRO} !"
}

tell_us(){
	echo -e ""
	echo -e "${Tip} --------------分隔线----------------" 
	echo -e "${Tip} 网址：www.nxrobo.com" 
	echo -e "${Tip} scorpio技术讨论与反馈QQ群：一群 8346256；二群 6646169" 
	echo -e "${Tip} ---------QQ扫描加入我们--------------"
	echo 'https://jq.qq.com/?_wv=1027&k=1JV8oyB8'|qrencode -o - -t UTF8
	echo -e ""
}

qrcode_picture()
{
	echo 'www.NXROBO.com'|qrencode -o - -t UTF8
}
check_sys
echo -e "————————————
  scorpio 一键安装管理脚本 ${Red_font_prefix}[v${sh_ver}]${Font_color_suffix}
"
qrcode_picture

echo -e "  
  请根据右侧的功能说明选择相应的序号。
  
  ${Green_font_prefix}  0.${Font_color_suffix} 单独编译scorpio
————————————
  ${Green_font_prefix}  1.${Font_color_suffix} 让机器人动起来
  ${Green_font_prefix}  2.${Font_color_suffix} 让scorpio跟着你走
  ${Green_font_prefix}  3.${Font_color_suffix} 让scorpio使用激光雷达绘制地图
  ${Green_font_prefix}  4.${Font_color_suffix} 让scorpio使用深度摄像头绘制地图
  ${Green_font_prefix}  5.${Font_color_suffix} 让scorpio使用激光雷达进行导航
  ${Green_font_prefix}  6.${Font_color_suffix} 让scorpio使用深度摄像头进行导航
  ${Green_font_prefix}  7.${Font_color_suffix} 深度学习进行物品检测

————————————

  ${Green_font_prefix}100.${Font_color_suffix} 问题反馈
  ${Green_font_prefix}101.${Font_color_suffix} 清除所有ROS节点
  ${Green_font_prefix}104.${Font_color_suffix} 文件传输
 "
menu_status
check_dev
echo && stty erase ^? && read -p "请输入数字：" num
case "$num" in
	0)
	install_scorpio	
	;;
	1)
	let_robot_go
	;;
	2)
	people_follow
	;;
	3)
	scorpio_build_map_2d
	;;
	4)
	scorpio_build_map_3d
	;;
	5)
	scorpio_navigation_2d
	;;
	6)
	scorpio_navigation_3d
	;;
	7)
	scorpio_DeepLearn
	;;

	100)
	tell_us
	;;
	101)
	killall_ros_nodes
	;;
	104)
	qrcode_transfer_files
	;;
	98)
	scorpio_test_mode
	;;
	*)
	echo -e "${Error} 请输入正确的数字 "
	;;
esac
