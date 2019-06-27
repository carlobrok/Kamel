#ifndef _KAMELI2C_H
#define _KAMELI2C_H

#include <cstdint>

// WHAT TO DO
#define MOTOR_DIR_PWM 0
#define MOTOR_DIR_PWM_BOTH 1
#define MOTOR_STATE 2

// WICH MOTOR
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define MOTOR_BOTH 2

// WICH DIRECTION
#define MOTOR_FORWARD 0
#define MOTOR_BACKWARD 1

// WICH MOTOR STATE
#define MOTOR_OFF 1
#define MOTOR_FORWARD_NORMAL 2
#define MOTOR_BACKWARD_NORMAL 3

int kamelI2Copen(int devId);

int setMotorDirPwm(int &fd, uint8_t side, uint8_t direction, uint8_t pwm);
int setMotorDirPwmBoth(int &fd, uint8_t direction_left, uint8_t pwm_left, uint8_t direction_right, uint8_t pwm_right);
int setMotorState(int &fd, uint8_t side, uint8_t state);

#endif
