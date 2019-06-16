#ifndef _KAMELI2C_H
#define _KAMELI2C_H

#include <cstdint>

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define MOTOR_BOTH 2

#define MOTOR_FORWARD 0
#define MOTOR_BACKWARD 1
#define MOTOR_OFF 2

int kamelI2Copen(int devId);
int writeMotor(int &fd, uint8_t side, uint8_t direction, uint8_t pwm);

#endif
