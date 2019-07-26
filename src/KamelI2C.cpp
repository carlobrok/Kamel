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

/* Returns a specific bit from a single byte.
 * The bit_index has to be between 0 and 7
 * credit of this function: https://stackoverflow.com/questions/4854207/get-a-specific-bit-from-byte
 */

bool get_bit(int8_t byte, uint8_t bit_index) {
	return (byte & (1 << bit_index - 1)) != 0;
}


/*  function reads 3 bytes of sensordata and fetches it to the output arrays.
 *  first 8 bit is for 8 digital sensors, last to bits are lowbyte and highbyt if 1 analog  10bit sensor.
 *
 *  Sequence of digital sensors is:
 *	IR_VORNE_L, IR_VORNE_R, IR_LINKS_V, IR_LINKS_H, IR_RECHTS_V, IR_RECHTS_H, T_HINTEN_L, T_HINTEN_R
 */

int getSensorData(int &fd, bool (&digital_sensor_data)[8], uint16_t (&analog_sensor_data)[1]) {
	uint8_t in_data[10];
	int ret = i2c_smbus_read_block_data(fd, 0, 3, in_data);

	for(int i = 0; i < 8; i++) {
		digital_sensor_data[i] = get_bit(in_data[0], 7-i);
	}

	analog_sensor_data[0] = in_data[8] | (in_data[9] << 8);
	return ret;
}
