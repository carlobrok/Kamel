#ifndef KAMELI2C_H
#define KAMELI2C_H

#include <stdint.h>       	// int8_t, uint8_t, uint16_t, ...

// WHAT TO DO   (DO NOT USE!)
#define MOTOR_DIR_PWM 0
#define MOTOR_DIR_PWM_BOTH 1
#define MOTOR_STATE 2

// WHICH DATA   (DO NOT USE!)
#define ALL_SENSOR_VALUES 1
#define DIGITAL_SENSOR_VALUES 2
#define ANALOG_SENSOR_VALUES 3



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

// SENSOR INDEX
// Definition: <Sensortyp>_<Ausrichtung>_<Seite>[_Entfernung/Größe/Kennzeichnung]
#define IR_VORNE_L 0
#define IR_VORNE_R 1
#define IR_LINKS_V 2
#define IR_LINKS_H 3
#define IR_RECHTS_V 4
#define IR_RECHTS_H 5
#define T_HINTEN_L 6
#define T_HINTEN_R 7



int kamelI2Copen(int devId);

int setMotorDirPwm(int &fd, uint8_t side, uint8_t direction, uint8_t pwm);
int setMotorDirPwmBoth(int &fd, uint8_t direction_left, uint8_t pwm_left, uint8_t direction_right, uint8_t pwm_right);
int setMotorState(int &fd, uint8_t side, uint8_t state);

bool get_bit(uint8_t byte, uint8_t bit_number);

int readBytes(int &fd, uint8_t *in_data, uint16_t data_length, uint8_t command);

int getSensorData(int &fd, bool (&digital_sensor_data)[8], uint16_t (&analog_sensor_data)[1]);
int getDigitalSensorData(int &fd, bool (&digital_sensor_data)[8]);
int getAnalogSensorData(int &fd, uint16_t (&analog_sensor_data)[1]);

void get_imu_data(float (&imu_data)[3]);
void m_imu(void);

#endif
