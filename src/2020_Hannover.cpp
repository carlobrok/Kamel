#include <iostream>		// cout
#include <fstream>    // std::ifstream std::ofstream
#include <stdint.h>       	// int8_t, uint8_t, uint16_t, ...

#include <vector>
#include <array>			// letzte sensorwerte
#include <thread>			// multithreading

#include <boost/circular_buffer.hpp>			// speichern der letzten n werte
#include <opencv2/opencv.hpp>				// opencv für bildauswertung
#include <math.h>					// abs, cos, sin, ...

#include "Logger.h"			// Logger class
#include "gruen.h"			// alles mit grünpunkt
#include "line.h"				// alles mit linie
#include "util.h"				// sonstige funktionen
#include "config.h"			// defines
#include "config_file.h"	// config datei

#include "CameraCapture.h"		// thread zum kamera einlesen
#include "VideoServer.h"			// thread für Videoausgabe über IP
#include "KamelDevices.h"					// kommunikation mit Arduino und Servosteuerung



using namespace std;
using namespace cv;

namespace lvl = spdlog::level;

Logger debug_lg("debug");			// logger class for log file 'debug.log'
Logger behavior_lg("behavior");				// logger class for log file 'behavior.log'

configuration::data configdata;

int flasche_fahren = 200;			// Wert, ab dem der Roboter nah genug dran ist und die Flasche umfährt
bool flasche_links = true;			// Auf true, wenn der Roboter an der Flasche links vorbei soll


void turn_angle(int & motor_fd, float (&imu_data)[3], float angle) {
	float target_angle = imu_data[YAW] + angle;
	float correction;
	bool correct_angle = false;

	std::cout << "Turn angle; current / target:" << imu_data[YAW] << "/" << target_angle << std::endl;

	while(!correct_angle) {
		get_imu_data(imu_data);
		correction = fmod((target_angle - imu_data[YAW] + 180 + 720), 360) - 180;

		std::cout << "correction: " << correction << std::endl;

		if(correction > 30) {
			setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 220, MOTOR_BACKWARD, 220);
		}
		else if(correction < -30) {
			setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 220, MOTOR_FORWARD, 220);
		}
		else if(correction > 15) {
			setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 100, MOTOR_BACKWARD, 100);
		}
		else if(correction < -15) {
			setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 100, MOTOR_FORWARD, 100);
		}
		else if(correction > 1) {
			setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 90, MOTOR_BACKWARD, 90);
		}
		else if(correction < -1) {
			setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 90, MOTOR_FORWARD, 90);
		}
		else if(correction <= 1 && correction >= -1) {
			setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
			thread_delay(100);
			get_imu_data(imu_data);
			correction = fmod((target_angle - imu_data[YAW] + 180 + 720), 360) - 180;

			// Hysterese für Messwertschwankungen
			if(correction < 1.75 && correction > -1.75) {
				correct_angle = true;
			}
		}

	}
}





void m_drive() {


	Logger sensor_lg("sensors");					// logger class for log file 'sensors.log'

	debug_lg << "init i2c devices" << lvl::debug;

	// init i2c devices

	int motor_fd = kamelI2Copen(0x08); 					// I2C Schnittstelle vom Motor-Arduino mit der Adresse 0x08 öffnen
	if(motor_fd == -1) {
		debug_lg << "error opening motor_arduino" << lvl::off;
		cout << "error opening motor_arduino" << endl;
		return;
	}

//	setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF); 		// Beide Motoren ausschalten

	int sensor_fd = kamelI2Copen(0x09);
	if(sensor_fd == -1) {
		debug_lg << "error opening sensor_arduino" << lvl::off;
		cout << "error opening sensor_arduino" << endl;
		return;
	}

	debug_lg << "successfully initialized i2c devices" << lvl::debug;
	debug_lg << "init sensor / camera variables" << lvl::debug;


	// init line variables
	vector<Point> m_line_points;			// m_line_points enthält lokale line_points, gilt nur im drive thread
	boost::circular_buffer<vector<Point>> last_line_points(50);		// circular_buffer last_line_points initialisieren


	// init green variables
	Point m_grcenter;				// temporäre kopie für den drive thread
	int m_grstate;				// temporäre kopie für den drive thread
	boost::circular_buffer<Point> last_grcenter(50);		// circular_buffer initialisieren

	// init digital sensor variables

	bool digital_sensor_data[8] = {1,1,1,1,1,1,1,1};
	array<boost::circular_buffer<bool>, 8> last_digital_data;
//	array<boost::circular_buffer<double>, 8> last_digital_time;
	for (auto& cb : last_digital_data) {
		cb.resize(100);
	}
//	for (auto& cb : last_digital_time) {
//		cb.resize(100);
//	}


	// init analog sensor variables

	uint16_t analog_sensor_data = 0;
	boost::circular_buffer<uint16_t> last_analog_data(100);
//	boost::circular_buffer<double> last_analog_time(100);


	// Init IMU variables


	bool rampe_hoch = false;
	bool rampe_runter = false;

	float imu_data[3] = {0,0,0};

	debug_lg << "successfully initialized sensor / camera variables" << lvl::debug;

	setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
	thread_delay(1500);

	while(1) {

		//setMotorDirPwmBoth(sensor_fd, MOTOR_FORWARD, 160, MOTOR_OFF, 0);

		// sensor value updating ======================

		// update values

		get_gruen_data(m_grcenter, m_grstate);		// grün data updaten
		get_line_data(m_line_points);					 		// line data updaten
		get_imu_data(imu_data);									// imu data updaten

		getSensorData(sensor_fd, digital_sensor_data, analog_sensor_data);
		thread_delay(2);
		std::cout << "A0: " << analog_sensor_data << std::endl;
		for(int i = 0; i  < 8; i++) {
			std::cout << "D" << i << ": " << digital_sensor_data[i] << "  ";
		}
		std::cout << std::endl;
		//last_line_points.push_front(m_line_points);	// push_front recent values - recent value is item [0]
		//last_grcenter.push_front(m_grcenter);		// push_front recent values - recent value is item [0]

		// update arduino sensor data
		//cout << "Read data: " << getSensorData(sensor_fd, digital_sensor_data, analog_sensor_data) << endl;

		//getDigitalSensorData(sensor_fd, digital_sensor_data);

		// push_front last values - recent value is item [0]
		/*last_analog_data.push_front(analog_sensor_data[0]);
		for(int i = 0; i < 8; i++) {
			last_digital_data[i].push_front(digital_sensor_data[i]);
		}*/

		cout << "drive routine; line_points: " << m_line_points << endl;

		// main part: drive decisions	=================================


// =========== RAMPE HOCH  ==================

		if (rampe_hoch) {
			// Sichergehen, ob der Robo noch auf der Rampe ist.
			if (imu_data[PITCH] < 5.0) {
				rampe_hoch = false;
				continue;
			}

			// Wenn der Roboter seitlich geneigt ist die wieder gerade ausrichten
			if (imu_data[ROLL] < -4.0) {	// Fährt nach links - linke Seite unten
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 180, MOTOR_FORWARD, 30);
			}
			else if (imu_data[ROLL] > 4.0) {	// Fährt nach rechts - rechte Seite unten
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 30, MOTOR_FORWARD, 180);
			}

			// wenn nur 1 linepoint vorhanden ist
			else if(m_line_points.size() == 1) {
				// Wenn der Roboter nicht seitlich geneigt ist, aber die Linie nicht mehr in der Mitte ist
				if (m_line_points[0].x > 360) {										// linie auf der Rampe zu weit rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 180, MOTOR_FORWARD, 90);
				} else if (m_line_points[0].x < 280) {								// linie auf der Rampe zu weit links
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 90, MOTOR_FORWARD, 180);
				}
				// Ansonsten gerade fahren
				else {
					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);
				}
			}
			// Ansonsten gerade fahren
			else {
				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);
			}
		}


// ========== RAMPE RUNTER ======================

		else if (rampe_runter) {
			if (imu_data[PITCH] > -5.0) {

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 110);
				thread_delay(1500);
				int64 tbegin = cur_ms();

				while((cur_ms() - tbegin) < 3000) {
					get_line_data(m_line_points);					 		// line data updaten

					// der Linie folgen
					if (m_line_points[0].x > 575) {										// line_points[0] rechts außen
						// Wenn sich einmal ein einziger Linepoint weit außen rechts befindet, soll sich der Roboter
						// So lange in diese Richtung drehen, wie entweder kein Linepoint vorhanden ist,
						// Oder alle linepoints noch zu weit rechts sind.
						bool done = false;
						setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 100, MOTOR_BACKWARD, 140);		// Drehung einleiten

						do {
							std::cout << "linie ganz rechts" << std::endl;
							thread_delay(5);
							get_line_data(m_line_points);							// Daten updaten
							for(auto &linepoint : m_line_points) {		// Überspringt die Schleife, wenn m_line_points leer
								if(linepoint.x < 480) done = true;			// wenn einer der line_points weit genug in der Mitte, oder rechts im Bild ist abbrechen
							}
						} while(!done);

					} else if (m_line_points[0].x < 65) {								// line_points[0] links außen
						// Wenn sich einmal ein einziger Linepoint weit außen links befindet, soll sich der Roboter
						// So lange in diese Richtung drehen, wie entweder kein Linepoint vorhanden ist,
						// Oder alle linepoints noch zu weit links sind.
						bool done = false;
						setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 140, MOTOR_FORWARD, 100);		// Drehung einleiten

						do {
							std::cout << "linie ganz links" << std::endl;
							thread_delay(5);
							get_line_data(m_line_points);							// Daten updaten
							for(auto &linepoint : m_line_points) {		// Überspringt die Schleife, wenn m_line_points leer
								if(linepoint.x > 120) done = true;			// wenn einer der line_points weit genug in der Mitte, oder rechts im Bild ist abbrechen
							}
						} while(!done);

					} else if (m_line_points[0].x > 500) {							// line_points[0] rechts
						setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 80);
					} else if (m_line_points[0].x < 140) {							// line_points[0] links
						setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 80, MOTOR_FORWARD, 160);
					} else if ((cur_ms() - tbegin) > 2000) {
						reset_last_movement_change();
						break;
					} else if (m_line_points[0].x > 400) {							// line_points[0] halb rechts
						setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 10);
					} else if (m_line_points[0].x < 240) {							// line_points[0] halb links
						setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 160);
					} else {																					 // line_points[0] mitte
						setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);
					}
				}
				rampe_runter = false;
				continue;
			}

			if(m_line_points.size() == 1) {
				// Wenn die Linie nicht mehr in der Mitte ist
				if (m_line_points[0].x > 360) {										// linie auf der Rampe zu weit rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 60, MOTOR_BACKWARD, 10);
				} else if (m_line_points[0].x < 280) {								// linie auf der Rampe zu weit links
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 60);
				} else {
					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 60);
				}
			} else {
				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 60);
			}
		}


		// Roboter gerade
		else {
			if (imu_data[PITCH] > 10.0) {
				rampe_hoch = true;
				continue;				// Rest der while schleife skippen
			}
			if (imu_data[PITCH] < -10.0) {
				rampe_runter = true;
				continue;				// Rest der while schleife skippen
			}


// ==========================   ROBOTER IST WAAGERECHT =================================00

// ========= FLASCHE ====================

			// Flasche mittig vor dem Roboter
			if(analog_sensor_data >= flasche_fahren && digital_sensor_data[IR_VORNE_L] == 1 && digital_sensor_data[IR_VORNE_R] == 1) {
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);

				int64 tbegin = cur_ms();
				bool flasche = true;

				while(cur_ms() - tbegin < 300) {
					thread_delay(5);
					getAnalogSensorData(sensor_fd, analog_sensor_data);
					if(analog_sensor_data < flasche_fahren - 20){
						flasche = false;
						break;
					}
				}

				if(flasche) {
					std::cout << " > FLASCHE FAHREN" << std::endl;
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD_NORMAL);

					while(analog_sensor_data > 110) {
						thread_delay(5);
						getAnalogSensorData(sensor_fd, analog_sensor_data);
					}
					// Bremsen
					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
					thread_delay(10);
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
					thread_delay(400);

					// Flasche links

					if(flasche_links) {
						// Links drehen
						turn_angle(motor_fd, imu_data, -45);

						// Vor bis hinten rechts an Flasche
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 100);
						do {
							getDigitalSensorData(sensor_fd, digital_sensor_data);
							thread_delay(5);
						} while(digital_sensor_data[IR_RECHTS_H] == 1);
						// Bremsen
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
						thread_delay(10);
						setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
						thread_delay(400);


						// Rechts drehen
						turn_angle(motor_fd, imu_data, 45);

						// Vor bis hinten rechts an Flasche
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 100);
						do {
							getDigitalSensorData(sensor_fd, digital_sensor_data);
							thread_delay(5);
						} while(digital_sensor_data[IR_RECHTS_H] == 1);
						// Bremsen
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
						thread_delay(10);
						setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
						thread_delay(400);
						// Rechts drehen
						turn_angle(motor_fd, imu_data, 55);
					}

					// Flasche rechts

					else {

						// rechts drehen
						turn_angle(motor_fd, imu_data, 45);

						// vor bis hinten links neben der Flasche
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
						do {
							getDigitalSensorData(sensor_fd, digital_sensor_data);
							thread_delay(5);
						} while(digital_sensor_data[IR_LINKS_H] == 1);
						// Bremsen
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
						thread_delay(10);
						setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
						thread_delay(400);


						// links drehen

						turn_angle(motor_fd, imu_data, -45);

						// vor bis hinten links neben der Flasche
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
						do {
							getDigitalSensorData(sensor_fd, digital_sensor_data);
							thread_delay(5);
						} while(digital_sensor_data[IR_LINKS_H] == 1);
						// Bremsen
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
						thread_delay(10);
						setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
						thread_delay(400);

						// links drehen
						turn_angle(motor_fd, imu_data, -55);
					}
					thread_delay(7);
				}
				continue;
			}

			// Objekt, das sehr breit ist direkt vor dem Roboter, oder die Flasche nicht mittig
			else if(analog_sensor_data >= flasche_fahren && (digital_sensor_data[IR_VORNE_L] == 0 || digital_sensor_data[IR_VORNE_R] == 0)) {
				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
				thread_delay(750);
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
				thread_delay(7);
				continue;
			}


//  =========== GRUEN ========================

			if (m_grstate == GRUEN_BEIDE) {

				debug_lg << "green point BOTH" << lvl::info;

				while (m_grstate != GRUEN_NICHT) {		// Solange grstate nicht GRUEN_NICHT ist
					get_gruen_data(m_grcenter, m_grstate);											// gruen werte aktualisieren
					if (m_grcenter.y > 350) {						// nah am grünpunkt; m_grcenter ist im unteren bildbereich
						break;
					} else if (m_grcenter.x < 310) {						// m_grcenter im linken Bildbereich
						setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 0, MOTOR_FORWARD, 110);			// links Kurve
					} else if (m_grcenter.x > 330) {						// m_grcenter im rechten Bildbereich
						setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 110, MOTOR_FORWARD, 0);			// rechts Kurve
					} else {																	// m_grcenter im mittleren, oberen Bildbereich
						setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);								// langsam vorwärts
					}
				}

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);		// beide Motoren vorwärts, pwm: 180
				thread_delay(500);																					// delay 500ms
				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 180);		// beide Motoren vorwärts, pwm: 180
				thread_delay(7);
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);		// beide Motoren vorwärts, pwm: 180
				thread_delay(500);
				turn_angle(motor_fd, imu_data, 180);			// 180° Drehen
				thread_delay(250);												// Warten für aktuelles Bild / damit der Kamerathread hinterher kommt

			} else if (m_grstate == GRUEN_LINKS && m_grcenter.y > 480 - 150) {			// m_grcenter im unteren Bildbereich + m_grstate = GRUEN_LINKS

				debug_lg << "green point LEFT" << lvl::info;					// in debug.log loggen

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 150);				// beide Motoren vorwärts, pwm: 150
				thread_delay(300);						// delay 300ms
				setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 190, MOTOR_FORWARD, 150);		// linkskurve, links schneller
				thread_delay(500);						// delay 500ms

			} else if (m_grstate == GRUEN_RECHTS && m_grcenter.y > 480 - 150) {				// m_grcenter im unteren Bildbereich + m_grstate = GRUEN_RECHTS

				debug_lg << "green point RIGHT" << lvl::info;			// in debug.log loggen

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 150);			// beide Motoren vorwärts, pwm: 150
				thread_delay(300);					// delay 300ms
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 150, MOTOR_BACKWARD, 190);		// rechtskurve, rechts schneller
				thread_delay(500);					// delay 500ms

			}

// ======== LINIE ================

			else if(m_line_points.size() == 1) {							// wenn nur 1 linepoint vorhanden ist

				debug_lg << "single Line point: " << m_line_points[0] << lvl::info;
				//			cout << "Different value -> check motor output for line" << endl;

				// Der Roboter hat sich 5 Sekunden weniger als 5°/Sekunde bewegt, steht aber nicht mittig auf der Linie:
				// Für 0,5 Sekunden mit vollem Tempo zurück
				if(get_last_movement_seconds() > 4.0 && (m_line_points[0].x < 300 || m_line_points[0].x > 340)) {
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 255, MOTOR_BACKWARD, 255);
					thread_delay(400);
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
					reset_last_movement_change();				// resetten, da sonst der Roboter eventuell dauerhaft rückwärts fährt, wenn er sich dabei nicht dreht
					thread_delay(7);		// Delay für i2c bus
				}

				// der Linie folgen
				if (m_line_points[0].x > 575) {										// line_points[0] rechts außen
					// Wenn sich einmal ein einziger Linepoint weit außen rechts befindet, soll sich der Roboter
					// So lange in diese Richtung drehen, wie entweder kein Linepoint vorhanden ist,
					// Oder alle linepoints noch zu weit rechts sind.
					//bool done = false;
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 160);		// Drehung einleiten
					thread_delay(500);
					/*while(!done) {
						std::cout << "linie ganz rechts" << std::endl;
						thread_delay(5);
						get_line_data(m_line_points);							// Daten updaten
						for(auto &linepoint : m_line_points) {		// Überspringt die Schleife, wenn m_line_points leer
							if(linepoint.x < 500) done = true;			// wenn einer der line_points weit genug in der Mitte, oder rechts im Bild ist abbrechen
						}
					}*/

				} else if (m_line_points[0].x < 65) {								// line_points[0] links außen
					// Wenn sich einmal ein einziger Linepoint weit außen links befindet, soll sich der Roboter
					// So lange in diese Richtung drehen, wie entweder kein Linepoint vorhanden ist,
					// Oder alle linepoints noch zu weit links sind.
					//bool done = false;
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 160, MOTOR_FORWARD, 160);		// Drehung einleiten
					thread_delay(500);
					/*while(!done) {
						std::cout << "linie ganz links" << std::endl;
						thread_delay(5);
						get_line_data(m_line_points);							// Daten updaten
						for(auto &linepoint : m_line_points) {		// Überspringt die Schleife, wenn m_line_points leer
							if(linepoint.x > 140) done = true;			// wenn einer der line_points weit genug in der Mitte, oder rechts im Bild ist abbrechen
						}
					}*/

				} else if (m_line_points[0].x > 500) {							// line_points[0] rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 80);
				} else if (m_line_points[0].x < 140) {							// line_points[0] links
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 80, MOTOR_FORWARD, 160);
				} else if (m_line_points[0].x > 400) {							// line_points[0] halb rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 10);
				} else if (m_line_points[0].x < 240) {							// line_points[0] halb links
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 160);
				} else if (m_line_points[0].x > 340) {							// line_points[0] mitte rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_OFF, 0);
				} else if (m_line_points[0].x < 300) {						 // line_points[0] mitte links
					setMotorDirPwmBoth(motor_fd, MOTOR_OFF, 0, MOTOR_FORWARD, 160);
				} else {																					 // line_points[0] mitte
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);
				}

			} else if(m_line_points.size() == 0 && last_line_points[1].size() == 1) {						// linie verloren, vorher nur 1 linepoint
				debug_lg << "lost line, last line singe line point: " << last_line_points[1][0];

				if (last_line_points[1][0].x > 575) {				// letzter linepoint rechts außen: nach rechts fahren, bis linie wieder gefunden
					debug_lg << " -  correction right" << lvl::warn;

					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 120);

					while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x > IMG_WIDTH / 2 + 100)) {
						thread_delay(5);
						get_line_data(m_line_points);
					}

					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
					thread_delay(500);
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);

				} else if (last_line_points[1][0].x < 65) {				// letzter linepoint links außen: nach links fahren, bis linie wieder gefunden
					cout << " -   correction left" << lvl::warn;

					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 120, MOTOR_FORWARD, 160);

					while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x < IMG_WIDTH / 2 - 100)) {
						thread_delay(5);
						get_line_data(m_line_points);
					}

					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
					thread_delay(500);
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);

				} else {							// wenn der letzte linepoint mittig war (Lücke) weiter gerade fahren
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);			// vorwärts mit festgelegtem Standardtempo
					debug_lg << lvl::warn;
				}
			} else {			// wenn linepoints.size() > 1 und kein grünpunkt, grade fahren
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);				// vorwärts mit festgelegtem Standardtempo
				//debug_lg << "driving forward, " << m_line_points.size() << " line points" << lvl::info;
			}
		}

		thread_delay(7);			// delay, da sonst der i2c buffer überschrieben wird und der Motorarduino falsche werte bekommt
	}
}




void image_processing() {

	Logger camera_lg("camera");		// Logger für Dateiname des aktuellen Bilds, sowie andere Bildinformationen

	Mat img_rgb;	// input image
	Mat hsv;			// input Bild im hsv Format
	Mat bin_sw;		// binäres bild schwarz / weiß erkennung; schwarze linie auf bild weiß, alles andere schwarz
	Mat bin_gr;		// binäres bild grünerkennung; grüner punkt weiß, alles andere schwarz


	CameraCapture cam(0);		// Video eingabe der Kamera '0'; 0, wenn nur eine Kamera angeschlossen ist
	VideoServer srv;				// Klasse für den VideoServer

	cam.set(cv::CAP_PROP_FPS, 30);			// Kamera Framerate auf 30 fps
	cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);			// Bildauflösung auf 640 x 480px
	cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	//Füge Fenster zum VideoServer hinzu
	srv.namedWindow("Mask Green");
	srv.namedWindow("Mask SW");
	srv.namedWindow("HSV");
	srv.namedWindow("Input");
	srv.namedWindow("H-S Histogram");



	int64 tloop;

	while(1) {

		tloop = getTickCount();			// Tickcount for whole loop

		while(!cam.read(img_rgb)){}		// warten bis das aktuelle bild einlesbar ist, dann in img_rbg einlesen

		// Filter image and convert to hsv
		GaussianBlur(img_rgb, img_rgb, Size(5,5),2,2);		// Gaussian blur to normalize image
		cvtColor(img_rgb, hsv, COLOR_BGR2HSV);			// Convert to HSV and save in Mat hsv

//#ifdef PLOT_HSV_HISTOGRAM

		// Histogramm der HSV Matrix ausgeben (wenn PLOT_HSV_HISTOGRAM definiert ist)

		vector<Mat> bgr_planes;
		split( hsv, bgr_planes );
		int histSize = 256;
		float range[] = { 0, 256 }; //the upper boundary is exclusive
		const float* histRange = { range };
		bool uniform = true, accumulate = false;
		Mat b_hist, g_hist, r_hist;
		calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
		calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
		calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
		int hist_w = 512, hist_h = 400;
		int bin_w = cvRound( (double) hist_w/histSize );
		Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 255,255,255) );
		normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
		normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
		normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
		for( int i = 1; i < histSize; i++ )
		{
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ),
							Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
							Scalar( 255, 0, 0), 2, 8, 0  );
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ),
							Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
							Scalar( 0, 255, 0), 2, 8, 0  );
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ),
							Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
							Scalar( 0, 0, 255), 2, 8, 0  );
		}


//#endif


		//		log_timing(tlast, "Reading, color covert, gauss: ");

		separate_gruen(hsv, bin_gr);  // grün auf dem Bild erkennen und in bin_gr als weiß schreiben, muss vor line_calc stehen, wenn der grünpunkt nicht als Linie erkannt werden soll

		//		log_timing(tlast, "Green separation: ");

		line_calc(img_rgb, hsv, bin_sw, bin_gr);		// linienpunkte berechnen, in m_line_points schreiben
		//		log_timing(tlast, "Line calculation: ");

		gruen_calc(img_rgb, hsv, bin_sw, bin_gr);		// grstate und grcenter berechnen


		//		log_timing(tlast, "Green calc: ");

		// Alle Bilder an VideoServer übergeben

		srv.imshow("H-S Histogram", histImage );
		srv.imshow("Input", img_rgb);
		srv.imshow("HSV", hsv);
		srv.imshow("Mask SW", bin_sw);
		srv.imshow("Mask Green", bin_gr);
		srv.update();			// VideoServer updaten

		std::cout << "Processing took: " << (getTickCount() - tloop) / getTickFrequency() * 1000.0 << " ms; FPS: " <<  cv::getTickFrequency() / (cv::getTickCount() - tloop) << std::endl;

	}
}



// MAIN FUNCTION TO CALL THREADS

int main() {

	init_clock();			// set start_clock to current ms

	std::ifstream ifs("/home/pi/projects/KamelPi/src/config.info", std::ifstream::in);			// Config datei einlesen
  ifs >> configdata;			// Config laden
  ifs.close();						// Datei wieder schließen

	set_thresh_black(configdata.getintvalue("THRESH_BLACK"));				// Schwarz threshold setzen
	set_gruen_range(configdata.getscalarvalue("LOWER_GREEN"),configdata.getscalarvalue("UPPER_GREEN"));		// Gruen Threshold setzen

	flasche_fahren = configdata.getintvalue("FLASCHE_FAHREN");
	flasche_links = configdata.getboolvalue("FLASCHE_LINKS");

	thread drive_t (m_drive);			// thread starten; ruft void m_drive auf
	thread imu_t (m_imu);					// thread startet; void m_imu in neuem thread
	image_processing();						// start void image_processing

	drive_t.detach();							// drive_t anhalten
	imu_t.detach();							// imu_t anhalten
	cout << "All threads closed" << endl;

	return -1;						// fehler
}
