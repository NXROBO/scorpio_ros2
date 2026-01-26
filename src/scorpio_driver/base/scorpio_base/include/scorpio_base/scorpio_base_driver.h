#ifndef SCORPIO_BASE_DRIVER_H
#define SCORPIO_BASE_DRIVER_H
#include <iostream>
#include <thread>
#include <memory>
#include <atomic>
#include <cstdlib>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <boost/thread/mutex.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include "scorpio_base/serial_port_motor.h"
#include "scorpio_base/serial_port_stm32.h"
#include "scorpio_base/msg/gyro_message.hpp"
#include "scorpio_base/msg/scorpio_base_odom.hpp"
#include "scorpio_base/kfilter.hpp"
#include "scorpio_base/mylock.hpp"
#define NODE_VERSION 0.01
#define SCORPIOBASETIMEOUT (1000 * 1e3) // 超过1s
#define COUNT_TIMES 20
using namespace std::chrono_literals;

namespace NxScorpioBase
{

  class ScorpioBaseDriver : public rclcpp::Node
  {
  public:
    ScorpioBaseDriver(std::string new_serial_stm32_port);
    ~ScorpioBaseDriver();

  private:
    unsigned short CalculateCRC16(unsigned char *msgPtr, unsigned int msgLen);

    int read_write_Data(unsigned short read_addr, unsigned short read_len, unsigned short write_addr, unsigned short write_len, short vel);
    int write_config_Data(unsigned short write_addr, unsigned short write_len, short dat);
    int baseFun(unsigned char *buf, int len);
    void startComParamInit();
    void getPtrFunction();
    void resetOdomCb(const scorpio_base::msg::ScorpioBaseOdom::SharedPtr odom);
    void cmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    int pubWheelJointStates(double linear_speed, double angular_speed);

    typedef int (ScorpioBaseDriver::*pfunc)(unsigned char *buf, int len);
    bool writeDeviceConfig(const std::string& username,  const std::string& password, int mo);
	  int checkMotorType(void);

    int pubGyroMessage(unsigned char *buf, int len);
    void publish_odom();
    void dealMessageSwitch(unsigned char *recvbuf);
    void hex_printf(unsigned char *buf, int len);
    void process_motor_receive_thread();
    void process_stm32_receive_thread();
    void startCloseCmd(char type, char onoff);
    unsigned char checkSum(unsigned char *buf);
    void getMotorComData(char *buf_r, int len);
    void getStm32ComData(char *buf, int len);
    void rcvPwmFun(float x, float z);
    void ackerMannCmdVelReceived(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void checkSerialGoon();
    void motorSendData();
    void callFunction(int index, unsigned char *recvbuf, int len);
    int nullFun(unsigned char *buf, int len);
    void setCovarianceMatrices(nav_msgs::msg::Odometry &odom_msg);
rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);
    void loadMotorConfig();
  private:
    std::string base_frame_id;
    std::string odom_frame_id;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    // tf::TransformBroadcaster tf_broadcaster;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_sub;
    rclcpp::Subscription<scorpio_base::msg::ScorpioBaseOdom>::SharedPtr odom_reset_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_acker_vel;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    std::shared_ptr<SerialPortMotor> motor_serial_port_;
    std::shared_ptr<SerialPortStm32> stm32_serial_port_;
    std::string serial_motor_port;
    std::string serial_stm32_port;
    std::map<int, pfunc> func_map;
    int overCurrent;
    int new_vel_bit;
    // rclcpp::Timer stimer;
    rclcpp::TimerBase::SharedPtr stimer;
    rclcpp::TimerBase::SharedPtr mtimer;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr fback_cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_pub;
    rclcpp::Publisher<scorpio_base::msg::GyroMessage>::SharedPtr gyro_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_fback_cmd_vel;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		int motor_type;
    unsigned short Current_PWM;
    int Flag_Get_Motor = 0;
    int Flag_Motor_Error = 0;
    int Current_Speed;
    int Flag_Motor_Switch;


    double last_x, last_y, last_yaw;
    double vel_x, vel_y, vel_yaw;
    double dt;
    int idx;
    unsigned int countSerial, lastCountSerial;
    double fb_time[COUNT_TIMES], fb_dist[COUNT_TIMES], fb_dist_x[COUNT_TIMES], odom_x[COUNT_TIMES], odom_y[COUNT_TIMES],
        odom_yaw[COUNT_TIMES], vel_x_list[COUNT_TIMES], vel_y_list[COUNT_TIMES];
    double robot_yaw;
    double left_wheel_position, right_wheel_position;
    int cur_pwm;
    bool hall_encoder;
    double odometry_x_;
    double odometry_y_;
    double odometry_yaw_;
    float current_speed;
    int Angular_Offset;
    NxScorpioBase::KFilter odom_x_kfilter, odom_y_kfilter;
    boost::mutex t_mutex;
    boost::mutex s3_mutex;
    std::shared_ptr<std::thread> motor_receive_thread_;
    std::shared_ptr<std::thread> stm32_receive_thread_;
  };

}

#endif // SCORPIO_BASE_DRIVER_H
