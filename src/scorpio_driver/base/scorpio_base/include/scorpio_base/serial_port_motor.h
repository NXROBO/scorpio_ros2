#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <boost/asio.hpp>
#include <algorithm>
#include <iterator>
#include "scorpio_base/scorpio_base_constants.h"

#include "scorpio_base/scorpio_base_serial.h"

#ifndef SERIAL_PORT_MOTOR_H_
#define SERIAL_PORT_MOTOR_H_

namespace NxScorpioBase
{


  class SerialPortMotor
  {
  public:
    SerialPortMotor(int motor_type_tmp, std::string dev_name, int baudrate=57600);
    ~SerialPortMotor();
    int openSerialPort();
    int closeSerialPort();
    int getMotorComData(char *buf_r, int len);
    int GetDataGram(unsigned char* r_buffer, int *length);
    unsigned short CalculateCRC16(unsigned char *msgPtr, unsigned int msgLen);
    int read_write_Data(unsigned short read_addr, unsigned short read_len, unsigned short write_addr, unsigned short write_len,  short  vel);
    int write_config_Data(unsigned short write_addr, unsigned short write_len,  short  dat);
    int write_Can_Start_Data();
    int write_Can_Free_Wheel();
    int write_Can_Auto_Send_Odom_Time(unsigned char time);
    int write_Can_Get_Motor_Status();
    int write_Can_Odom_Switch(unsigned char onoff);
    int write_Can_Clear_Stall();
    int write_Can_Set_Odom_Feedback();
    void write_vel2motor(float speed);
  	void getComCanData(char *buf_r, int len);

    int motor_type;
    std::shared_ptr<ScorpioSerial> serial_port_;
	  double limited_speed;
    int overCurrent;
    unsigned short Current_PWM;
    int Flag_Get_Motor = 0;
    int Flag_Motor_Error = 0;
    int Current_Speed;
    int Flag_Motor_Switch;
  };
}

#endif  // SERIAL_PORT_MOTOR_H_

// EOF
