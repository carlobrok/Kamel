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

int writeMotor(int &fd, uint8_t side, uint8_t direction, uint8_t pwm) {
	return i2c_smbus_write_block_data(fd, 0, 3, {side, direction, pwm});
}
