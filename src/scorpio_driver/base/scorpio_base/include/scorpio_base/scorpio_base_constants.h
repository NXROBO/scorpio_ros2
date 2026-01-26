

#ifndef SCORPIO_BASE_CONSTANTS_H
#define SCORPIO_BASE_CONSTANTS_H

#include <stdint.h>
union Char2Float
{
	float value;
	unsigned char buffer[4];
};
#define NONE "\e[0m"
#define BLACK "\e[0;30m"
#define RED "\e[0;31m"
#define GREEN "\e[0;32m"
#define YELLOW "\e[1;33m"
#define BLUE "\e[1;34m"
#define WHITE "\e[1;37m"
#define GRAY "\e[0;37m"
#define CLEAR "\033[2J"
#define CYAN "\e[0;36m"
#define MOVETO(x,y) printf("\033[%d;%dH", (x), (y))
#define ANGLE_MIDDLE_POINT 1118			//right--1080++left
#define ROOMBATIMEOUT (3000*1e6)
#define WD 0.105
#define COUNT_TIMES 20
#define MAX_SPEED 1.00
#define MOTOR_TYPE_HY 0
#define MOTOR_TYPE_JZD 1
#define MAX_ENCODER_COUNTS 0xFFFF


namespace NxScorpioBase {


}

#endif  // SCORPIO_BASE_CONSTANTS_H
