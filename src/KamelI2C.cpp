#include "KamelI2C.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstdint>


int kamelI2Copen(int devId) {
	const char *device ;
	int fd;

	//device = "/dev/i2c-0" ;
	device = "/dev/i2c-1" ;

	if((fd = open(device, O_RDWR)) < 0)
		return -1;

	if (ioctl (fd, I2C_SLAVE, devId) < 0)
		return -1;

	return fd;
}

int setMotorDirPwm(int &fd, uint8_t side, uint8_t direction, uint8_t pwm) {
	uint8_t data[3] = {side, direction, pwm};
	return i2c_smbus_write_block_data(fd, MOTOR_DIR_PWM, 3, data);
}

int setMotorDirPwmBoth(int &fd, uint8_t direction_left, uint8_t pwm_left, uint8_t direction_right, uint8_t pwm_right) {
	uint8_t data[4] = {direction_left, pwm_left, direction_right, pwm_right};
	return i2c_smbus_write_block_data(fd, MOTOR_DIR_PWM_BOTH, 4, data);
}

int setMotorState(int &fd, uint8_t side, uint8_t state) {
	uint8_t data[2] = {side, state};
	return i2c_smbus_write_block_data(fd, MOTOR_STATE, 2, data);
}

