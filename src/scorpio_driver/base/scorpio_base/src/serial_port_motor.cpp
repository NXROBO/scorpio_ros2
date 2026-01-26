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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include <cstdlib>
#include "scorpio_base/serial_port_motor.h"
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

unsigned char CRCH[256] =
	{
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

unsigned char CRCL[256] =
	{
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
		0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
		0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
		0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
		0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
		0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
		0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
		0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
		0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
		0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
		0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
		0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
		0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
		0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};
// *****************************************************************************
// Constructor
NxScorpioBase::SerialPortMotor::SerialPortMotor(int motor_type_tmp, std::string dev_name, int baudrate)
{
	motor_type = motor_type_tmp;
	serial_port_ = std::shared_ptr<ScorpioSerial>(new ScorpioSerial(dev_name, baudrate));
}

// *****************************************************************************
// Destructor
NxScorpioBase::SerialPortMotor::~SerialPortMotor()
{
	// Clean up!
	// delete serial_port_;
}

// *****************************************************************************
// Open the serial port
int NxScorpioBase::SerialPortMotor::openSerialPort()
{
	if (serial_port_->OpenSerial() < 0)
	{
		return -1;
	}

	return (0);
}

int NxScorpioBase::SerialPortMotor::GetDataGram(unsigned char *r_buffer, int *length)
{
	return serial_port_->GetDataGram(r_buffer, length);
}
// *****************************************************************************

unsigned short NxScorpioBase::SerialPortMotor::CalculateCRC16(unsigned char *msgPtr, unsigned int msgLen)
{
	unsigned char crcHigh = 0xFF;
	unsigned char crcLow = 0xFF;
	unsigned char index;
	while (msgLen--)
	{
		index = crcLow ^ (*(msgPtr++));
		crcLow = crcHigh ^ CRCH[index];
		crcHigh = CRCL[index];
	}
	return (unsigned short)((unsigned short)(crcHigh << 8) | crcLow);
}

int NxScorpioBase::SerialPortMotor::read_write_Data(unsigned short read_addr, unsigned short read_len, unsigned short write_addr, unsigned short write_len, short vel)
{
	unsigned int i;
	unsigned char sum = 0;
	unsigned char buffer[40];
	unsigned short crc_word;
	vel = -vel;
	Char2Float uvel;
	uvel.value = vel;
	buffer[0] = 0x01;
	buffer[1] = 0x17;
	buffer[2] = read_addr >> 8;
	buffer[3] = read_addr;
	buffer[4] = read_len >> 8;
	buffer[5] = read_len;
	buffer[6] = write_addr >> 8;
	buffer[7] = write_addr;
	buffer[8] = write_len >> 8;
	buffer[9] = write_len;
	buffer[10] = 0x02;
	buffer[11] = vel >> 8;
	buffer[12] = vel;
	crc_word = CalculateCRC16(buffer, 13);
	buffer[13] = crc_word;
	buffer[14] = crc_word >> 8;
	//	    for(int i=0; i<15; i++)
	//			printf("%02x ",buffer[i]);
	//	    printf("\n");
	serial_port_->WriteBuffer(buffer, 15);

	return (0);
}
// void NxScorpioBase::SerialPortMotor::write_vel2motor(float vel)
// {
// 	short mv ;
// 	if(vel>limited_speed)
// 		vel = limited_speed;
// 	else if (vel<-limited_speed)
// 		 vel = -limited_speed;
// 	mv = vel *11800;
// 	read_write_Data(0x002a, 0x0001, 0x002B, 0x0001, mv);
// }
int NxScorpioBase::SerialPortMotor::write_config_Data(unsigned short write_addr, unsigned short write_len, short dat)
{
	unsigned int i;
	unsigned char sum = 0;
	unsigned char buffer[40];
	unsigned short crc_word;

	buffer[0] = 0x01;
	buffer[1] = 0x10;
	buffer[2] = write_addr >> 8;
	buffer[3] = write_addr;
	buffer[4] = write_len >> 8;
	buffer[5] = write_len;
	buffer[6] = write_len * 2;
	buffer[7] = dat >> 8;
	buffer[8] = dat;
	crc_word = CalculateCRC16(buffer, 9);
	buffer[9] = crc_word;
	buffer[10] = crc_word >> 8;
	//	   for(int i=0; i<11; i++)
	//			printf("%02x ",buffer[i]);
	//	   printf("\n");
	serial_port_->WriteBuffer(buffer, 11);

	return (0);
}

int NxScorpioBase::SerialPortMotor::getMotorComData(char *buf_r, int len)
{
	int i;
	unsigned short crc_word;
	unsigned char *buf;
	buf = (unsigned char *)buf_r;
	/*for(i=0; i<len; i++)
		printf("%02x ", (buf[i]));
	printf("\n");*/
	if ((buf[0] == 1) && (len > 3))
	{
		if (buf[1] == 0x17)
		{
			if ((buf[2] + 5) == len)
			{
				crc_word = CalculateCRC16((unsigned char *)buf, 5);
				//	printf("%04x\n", crc_word);
				if ((unsigned short)(buf[len - 2] + (buf[len - 1] << 8)) == crc_word)
				{
					if (buf[4] & 0x01)
					{
						printf("the motor is overcurrent!");
						overCurrent = 1;
					}
					else
						overCurrent = 0;
				}
				else
				{
					//	RCLCPP_INFO(this->get_logger(), "check crc error");
				}
			}
		}
	}
	return overCurrent;
}
//====================JZD start============================

void NxScorpioBase::SerialPortMotor::getComCanData(char *buf_r, int len)
{
	int i;
	unsigned short crc_word;
	unsigned char *buf;
	buf = (unsigned char *)buf_r;
	/*for(i=0; i<len; i++)
		printf("%02x ", (buf[i]));
	printf("\n");*/
	if ((buf[0] == 0x08) && (len > 10))
	{
		if ((buf[1] == 0x01) && (buf[2] == 0x83))
		{
			if ((buf[3]) == 0x43)
			{
				Current_PWM = (buf[8] << 8) | buf[7];
				int cs = (buf[5] << 8) | buf[4];
				if (buf[6] == 0)
					Current_Speed = -cs;
				else
					Current_Speed = cs;
				// printf("current speed is %d, %f\n", Current_Speed,speed);
				// printf("%02X%02X\n", buf[8], buf[7]);
				Flag_Get_Motor = 1;
			}
		}
		else if ((buf[1] == 0x05) && (buf[2] == 0x83))
		{
			// ROS_ERROR("the motor status is %02X, %02X, %d", buf[9], buf[10], len);

			if ((buf[3] == 0x43) && (buf[9] == 0x04C) && (buf[10] == 0x4D))
			{
				if ((buf[7] != 0x00) || (buf[8] != 0x00))
				{
					Flag_Motor_Error = 1;
					// ROS_ERROR("the motor status is %02X, %02X", buf[7], buf[8]);
				}
			}
		}
	}
}

int NxScorpioBase::SerialPortMotor::write_Can_Start_Data()
{
	unsigned int i;
	unsigned char buffer[10];
	if (Flag_Motor_Switch == 1)
	{
		return 1;
	}
	buffer[0] = 0x2B;
	buffer[1] = 0x40;
	buffer[2] = 0x60;
	buffer[3] = 0x01;
	buffer[4] = 0x0F;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	// for (int i = 0; i < 8; i++)
	// 	printf("%02x ", buffer[i]);
	// printf("\n");

	serial_port_->WriteBuffer(buffer, 8);

	Flag_Motor_Switch = 1;
	return (0);
}

int NxScorpioBase::SerialPortMotor::write_Can_Free_Wheel()
{
	unsigned int i;
	unsigned char buffer[10];
	// return 1;
	if (Flag_Motor_Switch == 0)
	{
		return 1;
	}
	buffer[0] = 0x2B;
	buffer[1] = 0x40;
	buffer[2] = 0x60;
	buffer[3] = 0x01;
	buffer[4] = 0x06;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	// for (int i = 0; i < 8; i++)
	// 	printf("%02x ", buffer[i]);
	// printf("\n");
	serial_port_->WriteBuffer(buffer, 8);

	Flag_Motor_Switch = 0;
	return (0);
}
// time:鍗曚綅10ms
int NxScorpioBase::SerialPortMotor::write_Can_Auto_Send_Odom_Time(unsigned char time)
{
	unsigned int i;
	unsigned char buffer[10];
	buffer[0] = 0x2F;
	buffer[1] = 0x40;
	buffer[2] = 0x60;
	buffer[3] = 0x00;
	buffer[4] = 0x34;
	buffer[5] = time;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	// for (int i = 0; i < 8; i++)
	// 	printf("%02x ", buffer[i]);
	// printf("\n");
	serial_port_->WriteBuffer(buffer, 8);
	return (0);
}
int NxScorpioBase::SerialPortMotor::write_Can_Get_Motor_Status()
{
	unsigned int i;
	unsigned char buffer[10];

	buffer[0] = 0x40;
	buffer[1] = 0x01;
	buffer[2] = 0x26;
	buffer[3] = 0x00;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	// for (int i = 0; i < 8; i++)
	// 	printf("%02x ", buffer[i]);
	// printf("\n");
	serial_port_->WriteBuffer(buffer, 8);
	return (0);
}

int NxScorpioBase::SerialPortMotor::write_Can_Odom_Switch(unsigned char onoff)
{
	unsigned int i;
	unsigned char buffer[10];

	buffer[0] = 0x2F;
	buffer[1] = 0x40;
	buffer[2] = 0x60;
	buffer[3] = 0x00;
	buffer[4] = 0x10;
	buffer[5] = onoff; // 0x01琛ㄧず鎵撳紑锛?x00琛ㄧず鍏抽棴
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	// for (int i = 0; i < 8; i++)
	// 	printf("%02x ", buffer[i]);
	// printf("\n");

	serial_port_->WriteBuffer(buffer, 8);
	return (0);
}
int NxScorpioBase::SerialPortMotor::write_Can_Clear_Stall()
{
	unsigned int i;
	unsigned char buffer[10];
	buffer[0] = 0x2F;
	buffer[1] = 0x40;
	buffer[2] = 0x60;
	buffer[3] = 0x00;
	buffer[4] = 0x39;
	buffer[5] = 0x01;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	// for (int i = 0; i < 8; i++)
	// 	printf("%02x ", buffer[i]);
	// printf("\n");
	serial_port_->WriteBuffer(buffer, 8);
	return (0);
}
int NxScorpioBase::SerialPortMotor::write_Can_Set_Odom_Feedback()
{
	unsigned int i;
	unsigned char buffer[10];
	buffer[0] = 0x2F;
	buffer[1] = 0x40;
	buffer[2] = 0x60;
	buffer[3] = 0x00;
	buffer[4] = 0x0F;
	buffer[5] = 0x01;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	// for (int i = 0; i < 8; i++)
	// 	printf("%02x ", buffer[i]);
	// printf("\n");
	serial_port_->WriteBuffer(buffer, 8);
	return (0);
}

void NxScorpioBase::SerialPortMotor::write_vel2motor(float speed)
{
	if (motor_type == MOTOR_TYPE_JZD)
	{
		unsigned int i;
		unsigned char buffer[10];
		unsigned char result = std::abs(static_cast<int>(speed / 0.0212137559910961));
		if (result != 0)
		{
			write_Can_Start_Data();
		}
		// printf("speed is %f, result is %d\n", speed, result);
		buffer[0] = 0x2B;
		buffer[1] = 0xF0;
		buffer[2] = 0x2F;
		buffer[3] = 0x09;
		if (speed >= 0)
		{
			buffer[4] = 0x01;
			buffer[5] = result;
		}
		else
		{
			buffer[4] = 0x00;
			buffer[5] = result;
		}
		buffer[6] = 0x00;
		buffer[7] = 0x00;
		// for (int i = 0; i < 8; i++)
		// 	printf("%02x ", buffer[i]);
		// printf("\n");

		serial_port_->WriteBuffer(buffer, 8);

		if (result == 0)
		{
			write_Can_Free_Wheel();
		}
	}
	else
	{
		short mv;
		if (speed > limited_speed)
			speed = limited_speed;
		else if (speed < -limited_speed)
			speed = -limited_speed;
		mv = speed * 11800;
		read_write_Data(0x002a, 0x0001, 0x002B, 0x0001, mv);
	}
}
//====================JZD end============================

// *****************************************************************************

// EOF
