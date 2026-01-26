

#ifndef SCORPIO_BASE_COMMON_SERIAL_H
#define SCORPIO_BASE_COMMON_SERIAL_H
#define RECV_BUFFER_SIZE      512

#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <boost/asio.hpp>
#include <algorithm>
#include <iterator>
namespace NxScorpioBase {

class ScorpioSerial {
public:
    ScorpioSerial(std::string dev_name, uint32_t baudrate);

    ~ScorpioSerial();

    int OpenSerial();
    int CloseSerial();

    int WriteBuffer(uint8_t *buf, uint16_t length);

    int GetDataGram(unsigned char* r_buffer, int *length);
    void hex_printf(unsigned char *buf, int len);

private:
    std::string serial_name_;
    uint32_t baudrate_;
    int fd_;
};

}

#endif // SCORPIO_BASE_COMMON_SERIAL_H
