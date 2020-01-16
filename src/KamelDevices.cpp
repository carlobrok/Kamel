#include <sys/ioctl.h>				// ioctl
#include <fcntl.h>						// fcntl, O_RDWR
#include <cstdint>						// int8_t, uint8_t, uint16_t, ...
#include <cerrno>							// errno
#include <cstring>						// strerror
#include <string>							// string
#include <cctype>							// isdigit
#include <mutex>							// mutex
#include <linux/i2c-dev.h>		// i2c_smbus_...
#include <wiringSerial.h>

#include "KamelDevices.h"
#include "config.h"
#include "util.h"							// thread_delay

// returns a file descriptor of the opened I2C device

int kamelI2Copen(int devId) {
	const char *device;
	int fd;

	//device = "/dev/i2c-0";	// Older Raspberry Pi models
	device = "/dev/i2c-1";	// Raspberry Pi 3B+

	if((fd = open(device, O_RDWR)) < 0)	{			// open "/dev/i2c-1"
		std::cout << "Unable to open " << device << ": " << strerror(errno) << std::endl;
		return -1;
	}

	if (ioctl (fd, I2C_SLAVE, devId) < 0) {			// set device address of fd to devId
		std::cout << "Unable to open device " << devId << ": " << strerror(errno) << std::endl;
		return -1;
	}

	return fd;
}


// ================== Motoren ==================


/* struct für die Daten des vergangenen Sendens
	 - last_time:		time_point des letzten Mals senden
	 - last_data:		die bytes/Daten des letzten Mals senden
*/
template <size_t N>
struct last_values {
  std::chrono::high_resolution_clock::time_point last_time;
  uint8_t last_data[N];
	uint8_t last_protocol = 255;
};


/* prüft, ob es erforderlich ist die Daten zu senden,
	 - true:  wenn das letzte Mal senden zu lange her ist oder die neuen Daten nicht die alten sind
	 - false: wenn die neuen Daten gleich den alten Daten sind
*/
template <size_t N>
bool send_req(uint8_t (&data)[N], last_values<N> &l_data, uint8_t protocol) {

	if(l_data.last_protocol != protocol) {
		std::copy(std::begin(data), std::end(data), std::begin(l_data.last_data));
		l_data.last_time = std::chrono::high_resolution_clock::now();							// aktualisiert den time_point des letzten Sendens
		l_data.last_protocol = protocol;							// aktualisiert das zuletzt verwendete Protokoll
		return true;
	}

	else if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - l_data.last_time).count() > I2C_MOTOR_REFRESH_TIME) {				// prüft ob das letzte Mal senden länger als 100ms her ist
		std::copy(std::begin(data), std::end(data), std::begin(l_data.last_data));
		l_data.last_time = std::chrono::high_resolution_clock::now();							// aktualisiert den time_point des letzten Sendens
		l_data.last_protocol = protocol;							// aktualisiert das zuletzt verwendete Protokoll
		return true;
  }

	else if (!std::equal(std::begin(data), std::end(data), std::begin(l_data.last_data))) {			// prüft ob die Daten array gleich sind
    std::copy(std::begin(data), std::end(data), std::begin(l_data.last_data));
		l_data.last_time = std::chrono::high_resolution_clock::now();							// aktualisiert den time_point des letzten Sendens
		l_data.last_protocol = protocol;							// aktualisiert das zuletzt verwendete Protokoll
		return true;
  }

  return false;
}


// sets the direction and pwm rate of the given motors by sending the values over I2C to the motor-arduino

last_values<3> last_dirPwm;
int setMotorDirPwm(int &fd, uint8_t side, uint8_t direction, uint8_t pwm) {
	uint8_t data[3] = {side, direction, pwm};
	if(send_req(data, last_dirPwm, MOTOR_DIR_PWM))
		return i2c_smbus_write_block_data(fd, MOTOR_DIR_PWM, 3, data);
	else
		return 0;
}


// sets the direction and pwm rate of both motors by sending the values over I2C to the motor-arduino

last_values<4> last_dirPwmBoth;
int setMotorDirPwmBoth(int &fd, uint8_t direction_left, uint8_t pwm_left, uint8_t direction_right, uint8_t pwm_right) {
	uint8_t data[4] = {direction_left, pwm_left, direction_right, pwm_right};
	if(send_req(data, last_dirPwmBoth, MOTOR_DIR_PWM_BOTH))
		return i2c_smbus_write_block_data(fd, MOTOR_DIR_PWM_BOTH, 4, data);
	else
		return 0;
}


// sets the given motors to the given state by sending the values over I2C to the motor-arduino

last_values<2> last_state;
int setMotorState(int &fd, uint8_t side, uint8_t state) {
	uint8_t data[2] = {side, state};
	if(send_req(data, last_state, MOTOR_STATE))
		return i2c_smbus_write_block_data(fd, MOTOR_STATE, 2, data);
	else
		return 0;
}

/* Returns a specific bit from a single byte.
 * The bit_index has to be between 0 and 7
 * credit of this function: https://stackoverflow.com/questions/4854207/get-a-specific-bit-from-byte
 */


// ================== Sensoren ==================

bool get_bit(uint8_t byte, uint8_t bit_index) {
	return (byte & (1 << (bit_index - 1))) != 0;
}


int readBytes(int &fd, uint8_t *in_data, uint16_t data_length, uint8_t command) {
	return i2c_smbus_read_i2c_block_data(fd, command, data_length, in_data);
}


/* function reads 3 bytes of sensordata and fetches it to the output arrays.
 * first 8 bit is for 8 digital sensors, last to bits are lowbyte and highbyt if 1 analog  10bit sensor.
 *
 * Sequence of digital sensors is:
 * IR_VORNE_L, IR_VORNE_R, IR_LINKS_V, IR_LINKS_H, IR_RECHTS_V, IR_RECHTS_H, T_HINTEN_L, T_HINTEN_R
 */
int getSensorData(int &fd, bool (&digital_sensor_data)[8], uint16_t (&analog_sensor_data)[1]) {
	uint8_t in_data[3];
	int ret = i2c_smbus_read_i2c_block_data(fd, ALL_SENSOR_VALUES, 3, in_data);

	for(int i = 0; i < 8; i++) {
		digital_sensor_data[i] = get_bit(in_data[0], 7-i);
	}

	analog_sensor_data[0] = in_data[1] | (in_data[2] << 8);
	return ret;
}

int getDigitalSensorData(int &fd, bool (&digital_sensor_data)[8]) {
	uint8_t in_data[1];
	int ret = i2c_smbus_read_i2c_block_data(fd, DIGITAL_SENSOR_VALUES, 1, in_data);

	for(int i = 0; i < 8; i++) {
		digital_sensor_data[i] = get_bit(in_data[0], 7-i);
	}
	return ret;
}

int getAnalogSensorData(int &fd, uint16_t (&analog_sensor_data)[1]) {
	uint8_t in_data[2];
	int ret = i2c_smbus_read_i2c_block_data(fd, ANALOG_SENSOR_VALUES, 2, in_data);

	analog_sensor_data[0] = in_data[0] | (in_data[1] << 8);
	return ret;
}


// ================== IMU ===================

std::mutex imu_mutex;
float m_imu_data[3] = {0,0,0};

void get_imu_data(float (&imu_data)[3]) {
	std::lock_guard<std::mutex> m_lock(imu_mutex);			// mutex locken, zugriff auf die nächsten Variablen sperren
	for(int i = 0; i < 3; ++i) {
		imu_data[i] = m_imu_data[i];
	}
}

void set_imu_data(float (&imu_data)[3]) {							// only within KamelDevices.cpp available
	std::lock_guard<std::mutex> m_lock(imu_mutex);			// mutex locken, zugriff auf die nächsten Variablen sperren
	for(int i = 0; i < 3; ++i) {
		m_imu_data[i] = imu_data[i];
	}
}

void m_imu(void) {
	int imu_fd = serialOpen("/dev/serial0", IMU_BAUD);			// Serielle Schnittstelle öffnen, imu_fd ist der file descriptor
	float t_imu_data[AMOUNT_IMU_DATA] = {0,0,0};						// lokaler Buffer mit den aktuellen Werten als Float
	std::string in_str ("");																// Buffer der zueletzt gelesenen Bytes
	int in_idx = 0;																					// Index d. Variable

	while(true) {
		if(serialDataAvail(imu_fd) > 0) {									// sobald Daten verfügbar sind alle durchgehen
			int inByte = serialGetchar(imu_fd);      				// READ INCOMING BYTE

	    if (inByte == ':') {              							// ":"  ->  Zeichen für Beginn der Datenreihe
	      in_str = "";																	// Buffer zurücksetzen
	      in_idx = 0;																		// Index zurücksetzen
	    }
			else if (inByte == '!') {       								// "!" ->  Zeichen für Ende der Datenreihe
	      t_imu_data[in_idx] = std::stof(in_str); 			// write number from String into the array

				if(in_idx == AMOUNT_IMU_DATA - 1)
					set_imu_data(t_imu_data);										// ARRAY COMPLETE -> SAVE DATA TO GLOBAL ARRAY

	      in_idx = 0;																		// Index zurücksetzen
	    }
			else if (isdigit(inByte) || inByte == '.' || inByte == '-') {				// Zeichen, welche sich im Float befinden
	      in_str += (char)inByte;             					// add character to string
	    }
			else if (inByte == ',') {      									// number is complete
	      t_imu_data[in_idx++] = std::stof(in_str);    	// write number from String into the array, increment i
	      in_str = "";                        					// reset string
	    }
			else {                          								// unknown / error -> reset array and index -> return 0
	      for (auto& data : t_imu_data) data = 0;				// t_imu_data resetten

				// index und buffer zurücksetzen
	      in_idx = 0;
	      in_str = "";
	    }
		}
		thread_delay(IMU_REFRESH_DELAY);																	// delay für andere threads
	}
}
