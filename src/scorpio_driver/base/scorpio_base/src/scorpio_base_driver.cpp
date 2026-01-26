/*
 *  Copyright (c) 2022, NXROBO Ltd.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  Authors: Litian Zhuang <litian.zhuang@nxrobo.com>
 */

#include "scorpio_base/serial_port_motor.h"
#include "scorpio_base/serial_port_stm32.h"
#include "scorpio_base/scorpio_base_driver.h"

namespace NxScorpioBase
{

	ScorpioBaseDriver::ScorpioBaseDriver(std::string new_serial_stm32_port) : rclcpp::Node("scorpio_base_node")
	{
		robot_yaw = 0;
		overCurrent = 0;
		current_speed = 0;
		new_vel_bit = 0;
		motor_type = checkMotorType();
		// 声明 motor_angle_offset 参数
		int min_motor_angle_offset;
		int max_motor_angle_offset;
		this->declare_parameter<int>("min_motor_angle_offset", -100);
		this->declare_parameter<int>("max_motor_angle_offset", 100);
		this->declare_parameter<bool>("保存电机配置", false);
		min_motor_angle_offset = this->get_parameter("min_motor_angle_offset").as_int();
		max_motor_angle_offset = this->get_parameter("max_motor_angle_offset").as_int();

		rcl_interfaces::msg::ParameterDescriptor angle_offset_descriptor;
		angle_offset_descriptor.description = "Angle offset for motor";
		angle_offset_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;

		// 设置整数范围，使其在 rqt_reconfigure 中显示为滑动条
		rcl_interfaces::msg::IntegerRange angle_offset_range;
		angle_offset_range.from_value = min_motor_angle_offset;
		angle_offset_range.to_value = max_motor_angle_offset;
		angle_offset_range.step = 1;
		angle_offset_descriptor.integer_range.push_back(angle_offset_range);

		this->declare_parameter<int>("motor_angle_offset", -10, angle_offset_descriptor);
		this->declare_parameter("serial_stm32_port", "/dev/ttyS0");
		this->declare_parameter<bool>("hall_encoder", true);
		// this->declare_parameter("motor_angle_offset", 0);
		int tmp_angle_offset;
		bool save_motor_config;
		std::string str_motor_type;
		this->get_parameter("保存电机配置", save_motor_config);
		RCLCPP_INFO(this->get_logger(), "Parameter 保存电机配置 save_motor_config is: %d", save_motor_config);
		this->get_parameter("motor_angle_offset", tmp_angle_offset);
		RCLCPP_INFO(this->get_logger(), "Parameter motor_angle_offset is: %d", tmp_angle_offset);
		Angular_Offset = tmp_angle_offset;

		this->declare_parameter("battery/status", 0);
		// 创建参数回调
		parameter_callback_handle_ = this->add_on_set_parameters_callback(
			std::bind(&ScorpioBaseDriver::on_parameter_change, this, std::placeholders::_1));
		if (motor_type == MOTOR_TYPE_JZD)
		{
			this->declare_parameter("serial_motor_port", "/dev/canBase");
			std::string new_serial_motor_port = "/dev/canBase";
			this->get_parameter_or<std::string>("serial_motor_port", serial_motor_port, new_serial_motor_port);
			RCLCPP_INFO(this->get_logger(), "serial_motor_port is %s", serial_motor_port.c_str());
		}
		else
		{

			this->declare_parameter("serial_motor_port", "/dev/ttyS3");
			std::string new_serial_motor_port = "/dev/ttyS3";

			this->get_parameter_or<std::string>("serial_motor_port", serial_motor_port, new_serial_motor_port);
			RCLCPP_INFO(this->get_logger(), "serial_motor_port is %s", serial_motor_port.c_str());
		}

		this->get_parameter_or<std::string>("serial_stm32_port", serial_stm32_port, new_serial_stm32_port);

		this->get_parameter_or<bool>("hall_encoder", hall_encoder, true);
		double limited_speed = 0;
		this->declare_parameter("limited_speed", MAX_SPEED);
		this->get_parameter_or<double>("limited_speed", limited_speed, MAX_SPEED);

		if (limited_speed > MAX_SPEED)
			limited_speed = MAX_SPEED;
		RCLCPP_INFO(this->get_logger(), "limited_speed is %f", limited_speed);

		auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

		if (hall_encoder)
		{
			RCLCPP_INFO(this->get_logger(), "hall_encoder is true");

			this->declare_parameter("base_frame_id", "base_footprint");
			this->get_parameter_or<std::string>("base_frame_id", base_frame_id, "base_footprint");
			this->declare_parameter("odom_frame_id", "odom");
			this->get_parameter_or<std::string>("odom_frame_id", odom_frame_id, "odom");

			// pub_fback_cmd_vel =  this->create_publisher<geometry_msgs::msg::Twist>("/scorpio_base/command/velocity", 1);
			//  the velocity of robot's feedback
			pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);
			tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
		}

		pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 1);

		gyro_pub = this->create_publisher<scorpio_base::msg::GyroMessage>("scorpio_base/gyro", 5);

		getPtrFunction();

		pubWheelJointStates(0, 0);
		startComParamInit();
		motor_serial_port_->limited_speed = limited_speed;
		stimer = this->create_wall_timer(1s, std::bind(&ScorpioBaseDriver::checkSerialGoon, this));
		mtimer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ScorpioBaseDriver::motorSendData, this));

		idx = 0;
		for (int x = 0; x < COUNT_TIMES; x++)
		{
			fb_time[x] = 0;
			odom_x[x] = 0;
			odom_y[x] = 0;
			odom_yaw[x] = 0;
			vel_x_list[x] = 0;
		}

		RCLCPP_INFO(this->get_logger(), "inint ComDealDataNode");
	}
	ScorpioBaseDriver::~ScorpioBaseDriver()
	{
	}

	// 检测电机型号
	int ScorpioBaseDriver::checkMotorType(void)
	{
		std::string filePath = "/opt/nxrobo/ScorpioMotorType.txt";
		std::ifstream file(filePath);

		// 检查文件是否存在
		if (!file.is_open())
		{
			std::cout << "文件不存在 " << filePath << std::endl;
			return MOTOR_TYPE_HY;
		}

		// 读取文件内容
		std::string content;
		std::getline(file, content);
		file.close();

		// 去除可能的空白字符
		size_t start = content.find_first_not_of(" \t\n\r");
		size_t end = content.find_last_not_of(" \t\n\r");

		if (start != std::string::npos && end != std::string::npos)
		{
			content = content.substr(start, end - start + 1);
		}

		// 检查内容是否为"JZD"
		if (content == "JZD")
		{
			std::cout << "电机型号为：JZD" << std::endl;
			return MOTOR_TYPE_JZD;
		}
		else if (content == "HY")
		{
			std::cout << "电机型号为：" << content << std::endl;
			return MOTOR_TYPE_HY;
		}
		else
		{
			std::cout << "电机型号为：" << content << std::endl;
			return MOTOR_TYPE_HY;
		}
	}
	bool ScorpioBaseDriver::writeDeviceConfig(const std::string &username, const std::string &password, int new_motor_angle_offset)
	{

		// 配置文件路径
		const std::string config_path = "/opt/nxrobo/device_config.yaml";
		// 生成YAML内容
		std::stringstream yaml_content;
		yaml_content << "scorpio_base_node:\n";
		yaml_content << "  ros__parameters:\n";
		yaml_content << "    motor_angle_offset: " << new_motor_angle_offset << "\n";
		yaml_content << "    max_motor_angle_offset: 150\n";
		yaml_content << "    min_motor_angle_offset: -150\n";
		yaml_content << "    保存电机配置: False\n";

		try
		{
			// 如果提供了用户名和密码，使用sudo命令提升权限
			if (!username.empty() && !password.empty())
			{
				// 创建临时文件
				std::string temp_file = "/tmp/device_config_temp.yaml";
				std::ofstream temp_stream(temp_file);

				if (!temp_stream.is_open())
				{
					std::cerr << "错误: 无法创建临时文件" << std::endl;
					return false;
				}

				temp_stream << yaml_content.str();
				temp_stream.close();

				// 使用sudo命令复制文件到目标位置
				std::string cmd = "echo '" + password + "' | sudo -S cp " + temp_file + " " + config_path;
				int result = system(cmd.c_str());

				// 删除临时文件
				system(("rm -f " + temp_file).c_str());

				if (result != 0)
				{
					std::cerr << "错误: 使用sudo写入文件失败" << std::endl;
					return false;
				}
			}
			else
			{
				// 直接写入（如果当前用户有权限）
				std::ofstream config_file(config_path);

				if (!config_file.is_open())
				{
					std::cerr << "错误: 无法打开文件 " << config_path << std::endl;
					return false;
				}

				config_file << yaml_content.str();
				config_file.close();
			}

			std::cout << "成功: 配置已写入 " << config_path << std::endl;
			return true;
		}
		catch (const std::exception &e)
		{
			std::cerr << "异常: " << e.what() << std::endl;
			return false;
		}
	}
	rcl_interfaces::msg::SetParametersResult ScorpioBaseDriver::on_parameter_change(const std::vector<rclcpp::Parameter> &parameters)
	{
		rcl_interfaces::msg::SetParametersResult result;
		result.successful = true; // 或根据逻辑设置为 false
		result.reason = "Parameter updated successfully.";

		for (const auto &param : parameters)
		{
			if (param.get_name() == "motor_angle_offset")
			{
				Angular_Offset = param.as_int();
				RCLCPP_INFO(this->get_logger(), "Parameter motor_angle_offset changed to: %d", param.as_int());
			}
			else if (param.get_name() == "保存电机配置")
			{
				bool save_flag = param.as_bool();
				if (save_flag == true)
				{
					if (writeDeviceConfig("scorpio", "scorpio", Angular_Offset))
					{
						RCLCPP_ERROR(this->get_logger(), "保存电机偏移量成功 motor_angle_offset = %d", Angular_Offset);
					}
				}
			}
		}
		return result;
	}

	void ScorpioBaseDriver::setCovarianceMatrices(nav_msgs::msg::Odometry &odom_msg)
	{
		// 重置协方差矩阵
		std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
		std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);

		// 设置对角线值（经验值）
		odom_msg.pose.covariance[0] = 0.1;	 // x position
		odom_msg.pose.covariance[7] = 0.1;	 // y position
		odom_msg.pose.covariance[14] = 0.01; // z position (通常较小)
		odom_msg.pose.covariance[21] = 0.01; // roll
		odom_msg.pose.covariance[28] = 0.01; // pitch
		odom_msg.pose.covariance[35] = 0.1;	 // yaw

		odom_msg.twist.covariance[0] = 0.1;	 // linear x
		odom_msg.twist.covariance[7] = 0.1;	 // linear y
		odom_msg.twist.covariance[14] = 0.1; // linear z
		odom_msg.twist.covariance[21] = 0.1; // angular x
		odom_msg.twist.covariance[28] = 0.1; // angular y
		odom_msg.twist.covariance[35] = 0.1; // angular z
	}

	int ScorpioBaseDriver::baseFun(unsigned char *buf, int len)
	{
		static unsigned int timesec, lastsec;
		static int lastpwm, curpwm;
		static double robot_yaw;
		static int idx;
		static double vel_x, vel_y, vel_yaw;
		static double wheel_dist = 0;
		static int first_time = 1;
		double cur_dist;
		static double all_dist = 0;

		static double fb_time[COUNT_TIMES], fb_dist[COUNT_TIMES], fb_dist_x[COUNT_TIMES], odom_x[COUNT_TIMES], odom_y[COUNT_TIMES],
			odom_yaw[COUNT_TIMES], vel_x_list[COUNT_TIMES], vel_y_list[COUNT_TIMES];
		float speed;
		float acvx, acvy, acvz, anvx, anvy, anvz, roll, pitch, yaw;
		float qx, qy, qz, qw;
		sensor_msgs::msg::Imu car_imu;
		tf2::Quaternion q;
		int curr_idx = (idx + COUNT_TIMES - 1) % COUNT_TIMES;
		struct timeval tv;
		gettimeofday(&tv, NULL);
		long long ts = (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
		if (motor_type == MOTOR_TYPE_JZD)
		{
			if (motor_serial_port_->Flag_Get_Motor == 0)
				return 1;
			fb_time[curr_idx] = ts; // set scorpio base time which is different from ros time
			curpwm = motor_serial_port_->Current_PWM;
			if (first_time)
			{
				lastpwm = curpwm;
				first_time = 0;
			}
			cur_pwm = curpwm;
			short encoder_counts_ = curpwm - lastpwm;
			// printf("first=%d ", encoder_counts_);
			if (encoder_counts_ > MAX_ENCODER_COUNTS / 2)
				encoder_counts_ = -(MAX_ENCODER_COUNTS - curpwm + lastpwm);
			else if (encoder_counts_ < -MAX_ENCODER_COUNTS / 2)
				encoder_counts_ = (MAX_ENCODER_COUNTS - lastpwm + curpwm);
			cur_dist = -(double)encoder_counts_ / 360 / 36.0234375 * 3.14159265359 * WD;
			all_dist = all_dist + cur_dist;
		}
		else
		{
			fb_time[curr_idx] = ts; // set scorpio base time which is different from ros time

			timesec = (buf[30] << 24) | (buf[31] << 16) | (buf[32] << 8) | buf[33];
			curpwm = (buf[26] << 24) | (buf[27] << 16) | (buf[28] << 8) | buf[29];
			if (first_time)
			{
				lastpwm = curpwm;
				first_time = 0;
			}
			cur_pwm = curpwm;
			cur_dist = M_PI * WD * (curpwm - lastpwm) * 5 / 574;
		}
		lastpwm = curpwm;
		// RCLCPP_INFO(this->get_logger(), "the speed is %fm/s", speed);
		acvx = (float(short((buf[1] << 8) | buf[0])) / 32768 * 16 * 9.8); // m/s^2
		acvy = (float(short((buf[3] << 8) | buf[2])) / 32768 * 16 * 9.8);
		acvz = (float(short((buf[5] << 8) | buf[4])) / 32768 * 16 * 9.8);

		anvx = (float(short((buf[7] << 8) | buf[6])) / 32768 * 2000);
		anvy = (float(short((buf[9] << 8) | buf[8])) / 32768 * 2000);
		anvz = (float(short((buf[11] << 8) | buf[10])) / 32768 * 2000);

		roll = (float(short((buf[13] << 8) | buf[12]))) / 32768 * M_PI;
		pitch = (float(short((buf[15] << 8) | buf[14]))) / 32768 * M_PI;
		yaw = (float(short((buf[17] << 8) | buf[16]))) / 32768 * M_PI;
		//	printf("yaw is %f\n", yaw);
		qx = (float(short((buf[19] << 8) | buf[18])) / 32768);
		qy = (float(short((buf[21] << 8) | buf[20])) / 32768);
		qz = (float(short((buf[23] << 8) | buf[22])) / 32768);
		qw = (float(short((buf[25] << 8) | buf[24])) / 32768);

		/*	q = tf::createQuaternionFromYaw(yaw);
			car_imu.orientation.x = q.x();
			car_imu.orientation.y = q.y();
			car_imu.orientation.z = q.z();
			car_imu.orientation.w = q.w();
			car_imu.orientation_covariance[8] = pow(0.0017, 2);*/

		//	car_imu.orientation.x = qx;
		//	car_imu.orientation.y = qy;
		//	car_imu.orientation.z = qz;
		//	car_imu.orientation.w = qw;
		q.setRPY(roll, pitch, yaw); // tf2::createQuaternionFromRPY(roll, pitch, yaw);
		car_imu.orientation.x = q.x();
		car_imu.orientation.y = q.y();
		car_imu.orientation.z = q.z();
		car_imu.orientation.w = q.w();
		car_imu.orientation_covariance[0] = pow(0.0017, 2); //
		car_imu.orientation_covariance[4] = pow(0.0017, 2);
		car_imu.orientation_covariance[8] = pow(0.0017, 2);

		car_imu.angular_velocity.x = anvx * M_PI / 180.0; // rad/s
		car_imu.angular_velocity.y = anvy * M_PI / 180.0;
		car_imu.angular_velocity.z = anvz * M_PI / 180.0;
		car_imu.angular_velocity_covariance[0] = pow(0.1, 2);
		car_imu.angular_velocity_covariance[4] = pow(0.1, 2);
		car_imu.angular_velocity_covariance[8] = pow(0.1, 2);

		car_imu.linear_acceleration.x = acvx; // m/s^2
		car_imu.linear_acceleration.y = acvy;
		car_imu.linear_acceleration.z = acvz;
		car_imu.linear_acceleration_covariance[0] = pow(0.1, 2);
		car_imu.linear_acceleration_covariance[4] = pow(0.1, 2);
		car_imu.linear_acceleration_covariance[8] = pow(0.1, 2);
		double now_time = this->get_clock()->now().seconds();

		car_imu.header.stamp = rclcpp::Time(now_time);
		car_imu.header.frame_id = "IMU_link";
		pub_imu->publish(car_imu);

		if (hall_encoder)
		{
			// Update odometry
			odometry_x_ = odometry_x_ + cur_dist * cos(odometry_yaw_); // m
			odometry_y_ = odometry_y_ + cur_dist * sin(odometry_yaw_); // m
			odometry_yaw_ = yaw;
			wheel_dist = wheel_dist + cur_dist;
			// first, we'll publish the transforms over tf
			double est_x = odom_x_kfilter.predict(wheel_dist);

			odom_x[curr_idx] = est_x;
			odom_yaw[curr_idx] = odometry_yaw_;

			dt = (fb_time[curr_idx] - fb_time[idx]) * 0.001;
			vel_x_list[curr_idx] = (odom_x[curr_idx] - odom_x[idx]) / dt;
			vel_x = 0;
			for (int i = 0; i < COUNT_TIMES; i++)
			{
				vel_x += vel_x_list[i];
			}
			vel_x = vel_x / COUNT_TIMES;

			vel_y = 0; //(odom_y[curr_idx] - odom_y[idx])/dt;

			double delodom = (odom_yaw[curr_idx] - odom_yaw[idx]);
			if (delodom > 3.14159265359)
			{
				delodom = delodom - 2 * 3.14159265359;
			}
			if (delodom < -3.14159265359)
			{
				delodom = delodom + 2 * 3.14159265359;
			}
			vel_yaw = delodom / dt;

			double tmp_dist = 0;
			fb_dist[curr_idx] = wheel_dist;
			for (int i = 0; i < COUNT_TIMES; i++)
			{
				tmp_dist += fb_dist[i];
			}

			double fb_x = tmp_dist / dt;

			idx = (idx + 1) % COUNT_TIMES;

			auto message = nav_msgs::msg::Odometry();

			// 设置时间戳
			message.header.stamp = this->now();
			message.header.frame_id = "odom";
			message.child_frame_id = "base_footprint";
			int count_ = 1;
			// 设置位置和姿态（这里使用简单的递增数据作为示例）
			message.pose.pose.position.x = odometry_x_;
			message.pose.pose.position.y = odometry_y_;
			message.pose.pose.position.z = 0.0;

			// 设置四元数姿态（简单的旋转）
			message.pose.pose.orientation = car_imu.orientation;

			// 设置协方差矩阵
			setCovarianceMatrices(message);

			// 设置线速度和角速度
			message.twist.twist.linear.x = vel_x;
			message.twist.twist.linear.y = vel_y;
			message.twist.twist.linear.z = 0;
			message.twist.twist.angular.x = 0.0;
			message.twist.twist.angular.y = 0.0;
			message.twist.twist.angular.z = vel_yaw;

			// 发布odom消息
			pub_odom->publish(message);

			// 发布TF变换
			geometry_msgs::msg::TransformStamped transform;
			transform.header.stamp = this->now();
			transform.header.frame_id = "odom";
			transform.child_frame_id = "base_footprint";
			transform.transform.translation.x = message.pose.pose.position.x;
			transform.transform.translation.y = message.pose.pose.position.y;
			transform.transform.translation.z = message.pose.pose.position.z;
			transform.transform.rotation = message.pose.pose.orientation;
			tf_broadcaster_->sendTransform(transform);
		}
	}

	void ScorpioBaseDriver::startComParamInit()
	{
		if (motor_type == MOTOR_TYPE_JZD)
			motor_serial_port_ = std::shared_ptr<SerialPortMotor>(new SerialPortMotor(motor_type, serial_motor_port, 460800));
		else
			motor_serial_port_ = std::shared_ptr<SerialPortMotor>(new SerialPortMotor(motor_type, serial_motor_port));

		stm32_serial_port_ = std::shared_ptr<SerialPortStm32>(new SerialPortStm32(serial_stm32_port));

		if (motor_serial_port_->openSerialPort() < 0)
		{
			RCLCPP_FATAL(this->get_logger(), "serial_motor_port Could not connect to %s.", serial_motor_port.c_str());
			return;
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "connect to serial_motor_port sucessfully.");
		}

		if (stm32_serial_port_->openSerialPort() < 0)
		{
			RCLCPP_FATAL(this->get_logger(), "stm32_serial_port_ Could not connect to %s.", serial_stm32_port.c_str());
			return;
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "connect to stm32_serial_port_ sucessfully.");
			stm32_serial_port_->startCloseCmd(0x00, 1);
		}
		motor_receive_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&ScorpioBaseDriver::process_motor_receive_thread, this)));
		stm32_receive_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&ScorpioBaseDriver::process_stm32_receive_thread, this)));
	}
	int ScorpioBaseDriver::nullFun(unsigned char *buf, int len)
	{
		RCLCPP_INFO(this->get_logger(), "this is a null function!");
	}
	void ScorpioBaseDriver::getPtrFunction()
	{
		func_map[0x0000] = &ScorpioBaseDriver::nullFun;
		func_map[0x01] = &ScorpioBaseDriver::baseFun;
	}
	void ScorpioBaseDriver::callFunction(int index, unsigned char *recvbuf, int len)
	{
		if (func_map.count(index))
			(this->*(func_map[index]))(recvbuf, len);
		/*else
			RCLCPP_INFO(this->get_logger(), "unknown function:%02x", index);*/
	}

	int ScorpioBaseDriver::pubWheelJointStates(double linear_speed, double angular_speed)
	{

		return 0;
	}

	void ScorpioBaseDriver::hex_printf(unsigned char *buf, int len)
	{
#if 1
		int i;
		for (i = 0; i < len; i++)
		{
			printf("%02x ", buf[i]);
		}
		printf("\n");
#endif
	}
	void ScorpioBaseDriver::process_motor_receive_thread()
	{
		unsigned char buf[1024];
		int len;
		while (rclcpp::ok())
		{
			if (motor_serial_port_->GetDataGram(buf, &len) == 0)
			{
				// hex_printf(buf, len);
				if (motor_type == MOTOR_TYPE_JZD)
				{
					motor_serial_port_->getComCanData((char *)buf, len);
				}
				else // if(motor_type == MOTOR_TYPE_HY)
				{
					overCurrent = motor_serial_port_->getMotorComData((char *)buf, len);
				}
			}
		}
	}
	void ScorpioBaseDriver::process_stm32_receive_thread()
	{
		unsigned char buf[1024];
		int len;
		while (rclcpp::ok())
		{
			if (stm32_serial_port_->GetDataGram(buf, &len) == 0)
			{
				// hex_printf(buf, len);
				getStm32ComData((char *)buf, len);
			}
		}
	}

	unsigned char ScorpioBaseDriver::checkSum(unsigned char *buf)
	{
		unsigned char sum = 0;
		int i;
		int len = (buf[2] << 8) + buf[3];
		for (i = 0; i < len - 1; i++)
		{
			sum += buf[i];
		}
		return sum;
	}

	void ScorpioBaseDriver::getStm32ComData(char *buf, int len)
	{
		int i, j;
		static unsigned int count = 0;
		unsigned int checkcount;
		unsigned char tmpbuf[2550];
		static unsigned char recvbuf[2550];
		struct timeval currenttime;
		long long timediff;

		static struct timeval headertime;
		static int firsttime = 1;
		if (firsttime)
		{
			gettimeofday(&headertime, NULL);
			firsttime = 0;
		}
		gettimeofday(&currenttime, NULL);

		if (count == 0)
		{
			headertime = currenttime;
		}
		timediff = (currenttime.tv_sec - headertime.tv_sec) * 1000000 + (currenttime.tv_usec - headertime.tv_usec);

		if (timediff > ROOMBATIMEOUT)
		{
			count = 0;
			printf("nx-base time out-%lld\n", timediff);
			headertime = currenttime;
		}
		if ((len + count) > 255)
		{
			count = 0;
			printf("nx-base receive data too long! Drop it!\n");
			return;
		}
		memcpy(recvbuf + count, buf, len);
		count += len;
	BACKCHECK:
		if (count > 2)
		{
			checkcount = count - 1;
			for (i = 0; i < checkcount; i++)
			{
				if ((recvbuf[i] == 'N') && (recvbuf[i + 1] == 'X'))
				{
					if (i > 0)
					{
						count = count - i;
						memcpy(tmpbuf, recvbuf + i, count);
						memcpy(recvbuf, tmpbuf, count);
					}
					break;
				}
			}
#if 0
			if (i != 0)
			{
				for (j = 0; j < count; j++)
					printf(L_GREEN "%02X " NONE, (unsigned char)recvbuf[j]);  //
				printf("\n");
			}
#endif
			if (i == checkcount)
			{
				if (recvbuf[checkcount] == 'N')
				{
					count = 1;
					recvbuf[0] = 'N';
				}
				else
				{
					count = 0;
				}
			}
			if (count > 4)
			{
				unsigned int framelen = (recvbuf[2] << 8) + recvbuf[3];
				if (framelen < 6)
				{
					count = 0;
				}
				else
				{
					if (count >= framelen)
					{
#if 1
						if (0)
						{
							for (j = 0; j < framelen; j++)
								printf("%02X ", (unsigned char)recvbuf[j]);
							printf("\n");
						}
#endif
						if ((recvbuf[0] == 'N') && (recvbuf[1] == 'X')) // check the header
						{
							if (checkSum(recvbuf) == recvbuf[framelen - 1])
							{
								callFunction(recvbuf[4], recvbuf + 5, len - 6);
							}
							else
							{
								printf("stm32-base-check sum error\n");
								for (j = 0; j < framelen; j++)
									printf(RED "%02X " NONE, (unsigned char)recvbuf[j]);
								printf("\n");
							}
						}
						else
						{
							printf("carbase-header error\n");
							for (j = 0; j < framelen; j++)
								printf(RED "%02X " NONE, (unsigned char)recvbuf[j]);
							printf("\n");
						}
						if (count > framelen)
						{
							memcpy(tmpbuf, recvbuf + framelen, count - framelen);
							memcpy(recvbuf, tmpbuf, count - framelen);
							count = count - framelen;
							headertime = currenttime;
							goto BACKCHECK;
						}
						count = 0;
					}
				}
			}
		}
	}

	void ScorpioBaseDriver::ackerMannCmdVelReceived(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr ack_vel)
	{
		float vel = ack_vel->drive.speed;
		t_mutex.lock();
		current_speed = vel;
		new_vel_bit = 1;
		t_mutex.unlock();
		stm32_serial_port_->rcvPwmFun(ack_vel->drive.speed, ack_vel->drive.steering_angle, Angular_Offset);
		countSerial++;
		// RCLCPP_INFO(this->get_logger(), "limited_speed is %f", limited_speed);
	}
	void ScorpioBaseDriver::checkSerialGoon()
	{
		static int last_pwm;
		static int first_time = 1;
		static int swap_bit;
		static int ovcnt = 0;
		if (first_time)
		{
			sleep(1);
			if (motor_type == MOTOR_TYPE_JZD) // JZD鐢垫満
			{
				while ((motor_serial_port_->Flag_Get_Motor == 0) || (first_time == 1))
				{
					printf("======init the motor!======\n");
					for (int i = 0; i < 10; i++)
					{
						stm32_serial_port_->startCloseCmd(0x01, 0x01); // open motor power
						usleep(100000);
						motor_serial_port_->write_Can_Start_Data();
						usleep(100000);
						stm32_serial_port_->rcvPwmFun(0, 0, Angular_Offset);
					}

					motor_serial_port_->write_Can_Start_Data();
					usleep(100000);

					first_time = 0;
					motor_serial_port_->write_Can_Clear_Stall();
					usleep(100000);
					stm32_serial_port_->rcvPwmFun(0, 0, Angular_Offset);

					motor_serial_port_->write_Can_Free_Wheel();
					usleep(100000);

					motor_serial_port_->write_Can_Set_Odom_Feedback();
					usleep(100000);

					motor_serial_port_->write_Can_Auto_Send_Odom_Time(10);

					motor_serial_port_->write_Can_Odom_Switch(0x01);
				}
			}
			else
			{
				motor_serial_port_->write_config_Data(0x0006, 0x0001, 0x00);
				first_time = 0;
				sleep(1);
			}
			sub_acker_vel = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
				"/ackermann_cmd", 10, std::bind(&ScorpioBaseDriver::ackerMannCmdVelReceived, this, std::placeholders::_1));
			return;
		}
		else
		{
			if (motor_type == MOTOR_TYPE_JZD)
			{
				float speed = (double)motor_serial_port_->Current_Speed / 360 * 3.14159265359 * WD / 36.0234375;
				// printf("current speed is %d, %f\n", Current_Speed, speed);
				stm32_serial_port_->sendVel2Stm(speed);
			}
		}
		if (countSerial == lastCountSerial)
		{
			stm32_serial_port_->rcvPwmFun(0, 0, Angular_Offset);
			if (swap_bit)
			{
				if (motor_type == MOTOR_TYPE_JZD) // JZD鐢垫満
				{
					if (motor_serial_port_->Flag_Motor_Error)
					{
						motor_serial_port_->write_Can_Clear_Stall(); // 娓呴櫎鍫佃浆鏍囧織
						motor_serial_port_->Flag_Motor_Error = 0;
					}
					else
						motor_serial_port_->write_Can_Get_Motor_Status();
				}
				else
				{
					motor_serial_port_->write_config_Data(0x0006, 0x0001, 0x00);
				}
				swap_bit = 0;
			}
			else
			{
				motor_serial_port_->write_vel2motor(0.0);
				swap_bit = 1;
			}
		}
		else
		{
			lastCountSerial = countSerial;
		}
		if (overCurrent)
		{
			ovcnt++;
			if (ovcnt > 4)
			{
				stm32_serial_port_->startCloseCmd(0x01, 0x01); // open motor power
			}
			else if (ovcnt > 2)
			{
				stm32_serial_port_->startCloseCmd(0x01, 0x00); // close motor power
			}
		}
		else
			ovcnt = 0;
#if 0
		double cur_dist = PI*WD*(cur_pwm-last_pwm)*5/574; //287
		last_pwm = cur_pwm;
		ROS_WARN("current speed is %f", cur_dist);
#endif
	}
	void ScorpioBaseDriver::motorSendData()
	{
		float vel;
		int newbit = 0;
		t_mutex.lock();
		if (new_vel_bit)
		{
			vel = current_speed;
			newbit = 1;
			new_vel_bit = 0;
		}
		t_mutex.unlock();
		if (newbit)
			motor_serial_port_->write_vel2motor(vel);
	}

}
