#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <boost/asio.hpp>
#include <algorithm>
#include <iterator>
#include "scorpio_base/scorpio_base_constants.h"
#include "scorpio_base/scorpio_base_serial.h"


#ifndef SERIAL_PORT_STM32_H_
#define SERIAL_PORT_STM32_H_

namespace NxScorpioBase
{
  class SerialPortStm32
  {
  public:
    SerialPortStm32(std::string dev_name, int baudrate=115200);
    ~SerialPortStm32();
    int openSerialPort();
    int closeSerialPort();
    int GetDataGram(unsigned char* r_buffer, int *length);
    int writeData(unsigned char cmd, unsigned char *buf, unsigned int len);
    void rcvPwmFun(float x, float z, int angular_offset);
    void startCloseCmd(char type, char onoff);
    std::shared_ptr<ScorpioSerial> serial_port_;
    void sendVel2Stm(float vel);

  };
}

#endif  // SERIAL_PORT_STM32_H_

// EOF
