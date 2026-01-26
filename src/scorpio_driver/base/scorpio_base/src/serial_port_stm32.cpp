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
#include "scorpio_base/serial_port_stm32.h"
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

// *****************************************************************************
// Constructor
NxScorpioBase::SerialPortStm32::SerialPortStm32(std::string dev_name, int baudrate)
{

	serial_port_ = std::shared_ptr<ScorpioSerial>(new ScorpioSerial(dev_name, baudrate));
}

// *****************************************************************************
// Destructor
NxScorpioBase::SerialPortStm32::~SerialPortStm32()
{
	// Clean up!
	// delete serial_port_;
}

// *****************************************************************************
// Open the serial port
int NxScorpioBase::SerialPortStm32::openSerialPort()
{
	if (serial_port_->OpenSerial() < 0)
	{
		return -1;
	}

	return (0);
}

int NxScorpioBase::SerialPortStm32::GetDataGram(unsigned char *r_buffer, int *length)
{
	return serial_port_->GetDataGram(r_buffer, length);
}
// *****************************************************************************

int NxScorpioBase::SerialPortStm32::writeData(unsigned char cmd, unsigned char *buf, unsigned int len)
{
	// Compose comand
	unsigned int i;
	unsigned char sum = 0;
	unsigned char buffer[5024];
	buffer[0] = 'N';
	buffer[1] = 'X';
	buffer[2] = (len + 6) >> 8;
	buffer[3] = len + 6;
	buffer[4] = cmd;
	for (i = 0; i < len; i++)
	{
		buffer[5 + i] = buf[i];
	}
	for (i = 0; i < len + 5; i++)
	{
		sum = sum + buffer[i];
	}
	buffer[5 + len] = sum;
	// for(int i=0; i<length; i++)
	//    printf("%02x ",buffer[i]);
	//    printf("\n");
	serial_port_->WriteBuffer(buffer, len + 6);

	return (0);
}
// send car vel to stm32
void NxScorpioBase::SerialPortStm32::sendVel2Stm(float vel)
{
	unsigned char buf[10];
	int velx100 = vel * 100;
	buf[0] = velx100 >> 24;
	buf[1] = velx100 >> 16;
	buf[2] = velx100 >> 8;
	buf[3] = velx100;
	writeData(0x07, buf, 4);
}
void NxScorpioBase::SerialPortStm32::rcvPwmFun(float x, float z, int angular_offset)
{
	int angular_middle_point = 1080 + 20 - angular_offset;
	int pwml = 1080, pwma = angular_middle_point;
	unsigned char buf[20];
	float dz;
	if (z > 1)
		z = 1;
	dz = -180 * z / M_PI * 6;
	if (dz > 0) // right转
	{
		pwma = angular_middle_point - dz;
		if (pwma < 800) // 770
			pwma = 800; // 770
	}
	else if (dz < 0) // left转
	{
		pwma = angular_middle_point - dz;
		if (pwma > 1460) // 1230
			pwma = 1460; // 1230
	}
	buf[0] = pwml >> 8;
	buf[1] = pwml;
	buf[2] = pwma >> 8;
	buf[3] = pwma;
	writeData(0x01, buf, 4);
}
// type:00 is bottom switch,01 is motor power
void NxScorpioBase::SerialPortStm32::startCloseCmd(char type, char onoff)
{
	unsigned char buf[10];
	buf[0] = type;
	buf[1] = onoff;
	writeData(0x06, buf, 2);
}
// *****************************************************************************

// EOF
