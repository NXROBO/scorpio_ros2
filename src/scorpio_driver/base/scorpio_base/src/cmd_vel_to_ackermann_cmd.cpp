/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2025, NXROBO.
 *  All rights reserved.
 *
 * Author: litian.zhuang on 7/20/2025
 *********************************************************************/
#define NODE_VERSION 0.01
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

std::string frame_id;
double wheelbase;
rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_acker;

float convert_trans_rot_vel_to_steering_angle(float v, float omega, float wb)
{
	float radius;
	if ((omega == 0) || (v == 0))
		return 0;

	radius = v / omega;
	return atan(wb / radius);
}

void cmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
	float steering;
	ackermann_msgs::msg::AckermannDriveStamped msg;
	auto node = rclcpp::Node::make_shared("temp_node"); // 临时节点，用于获取时钟

	steering = convert_trans_rot_vel_to_steering_angle(cmd_vel->linear.x, cmd_vel->angular.z, wheelbase);
	double now_time = node->get_clock()->now().seconds();
	msg.header.stamp = rclcpp::Time(now_time);
	msg.header.frame_id = frame_id;
	msg.drive.steering_angle = steering;
	msg.drive.speed = cmd_vel->linear.x;
	pub_acker->publish(msg);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("cmd_vel_to_ackermann_cmd");
	node->declare_parameter<std::string>("frame_id", "odom");
	node->get_parameter_or<std::string>("frame_id", frame_id, "odom");
	node->declare_parameter<double>("wheelbase", 0.315);
	wheelbase = node->get_parameter("wheelbase").as_double();
	pub_acker = node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 1);
	auto subscription = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, cmdVelReceived);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
