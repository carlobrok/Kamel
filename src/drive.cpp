#include <iostream>
#include <vector>
#include "util.h"
#include "config.h"
#include "gruen.h"
#include "line.h"


#include "Logger.h"
#include "KamelDevices.h"					// kommunikation mit Arduino und Servosteuerung

using namespace spdlog::level;
using std::cout;
using std::endl;
using std::vector;
using cv::Point;

// Logger for everything what happens in this file
Logger drive_lg("drive");


int kamera_winkel = 65;
int flasche_fahren = 200;			// Wert, ab dem der Roboter nah genug dran ist und die Flasche umfährt
bool flasche_links = true;			// Auf true, wenn der Roboter an der Flasche links vorbei soll


void set_kamera_winkel(int winkel) {
	kamera_winkel = winkel;
}

void set_flasche_fahren(int grenzwert) {
	flasche_fahren = grenzwert;
}
void set_flasche_links(bool links_fahren) {
	flasche_links = links_fahren;
}


void turn_angle(int & motor_fd, float (&imu_data)[3], float angle) {
	float target_angle = imu_data[YAW] + angle;
	float correction;
	bool correct_angle = false;

	drive_lg << "Turn angle; current / target:" << imu_data[YAW] << "/" << target_angle << info;

	while(!correct_angle) {
		sen::get_imu_data(imu_data);
		correction = fmod((target_angle - imu_data[YAW] + 180 + 720), 360) - 180;

		drive_lg << "correction: " << correction << trace;

		if(correction > 30) {
			mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 220, MOTOR_BACKWARD, 220);
		}
		else if(correction < -30) {
			mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 220, MOTOR_FORWARD, 220);
		}
		else if(correction > 15) {
			mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 100, MOTOR_BACKWARD, 100);
		}
		else if(correction < -15) {
			mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 100, MOTOR_FORWARD, 100);
		}
		else if(correction > 1) {
			mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 90, MOTOR_BACKWARD, 90);
		}
		else if(correction < -1) {
			mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 90, MOTOR_FORWARD, 90);
		}
		else if(correction <= 1 && correction >= -1) {
			drive_lg << "correction <= 1 and >= -1: " << correction << " try to quit" << debug;
			mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);
			thread_delay(100);
			sen::get_imu_data(imu_data);
			correction = fmod((target_angle - imu_data[YAW] + 180 + 720), 360) - 180;

			// Hysterese für Messwertschwankungen
			if(correction <= 1 && correction >= -1) {
				drive_lg << "quit turn_angle" << debug;
				correct_angle = true;
			}
		}

	}
}





void m_drive() {

	drive_lg.set_level(debug);

	drive_lg << "init i2c devices" << debug;

	// init i2c devices
	int motor_fd = kamelI2Copen(0x08); 					// I2C Schnittstelle vom Motor-Arduino mit der Adresse 0x08 öffnen
	if(motor_fd == -1) {
		drive_lg << "error opening motor_arduino" << off;
		cout << "error opening motor_arduino" << endl;
		return;
	}

//	mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF); 		// Beide Motoren ausschalten

	int sensor_fd = kamelI2Copen(0x09);
	if(sensor_fd == -1) {
		drive_lg << "error opening sensor_arduino" << off;
		cout << "error opening sensor_arduino" << endl;
		return;
	}

	int servo_fd = mot::pca9685_setup(0x40);
	if(servo_fd == -1) {
		drive_lg << "error opening servo controller" << off;
		cout << "error opening servo controller" << endl;
		return;
	}

	drive_lg << "> COMPLETE - successfully initialized i2c devices" << debug;

	drive_lg << "init servos" << debug;

  mot::Servo cameraservo(servo_fd, 0);
  cameraservo.set_angle(kamera_winkel);

	mot::Servo kaefigservo(servo_fd, 1, 180, 0.7, 2.2);
	kaefigservo.set_angle(180);

	drive_lg << "> COMPLETE" << debug;

	// init line variables
	vector<Point> m_line_points;			// m_line_points enthält lokale line_points, gilt nur im drive thread
	boost::circular_buffer<vector<Point>> last_line_points(30);		// circular_buffer last_line_points initialisieren

	// init green variables
	Point m_grcenter;				// temporäre kopie für den drive thread
	int m_grstate;				// temporäre kopie für den drive thread
	//boost::circular_buffer<Point> last_grcenter(30);		// circular_buffer initialisieren


	// Init IMU variables


	bool rampe_hoch = false;
	bool rampe_runter = false;

	float imu_data[3] = {0,0,0};

	mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);
	thread_delay(2000);

	kaefigservo.off();

	drive_lg << "init COMPLETE - start loop" << debug;

	while(1) {


		// sensor value updating ======================

		// update values

		drive_lg << "update green data" << info;
		get_gruen_data(m_grcenter, m_grstate);		// grün data updaten
		drive_lg << "update line data" << info;
		get_line_data(m_line_points, last_line_points);					 		// line data updaten
		drive_lg << "line_points: " << m_line_points << info;
		drive_lg << "update imu data" << info;
		sen::get_imu_data(imu_data);									// imu data updaten

		drive_lg << "update sensor data" << info;
		sen::update_sensordata();
		//thread_delay(2);
		drive_lg << "A0: " << sen::analog_sensor_data(IR_MITTE);
		for(int i = 0; i  < 8; i++) {
			drive_lg << "  D" << i << ": " << sen::digital_sensor_data(i);
		}
		drive_lg << info;

		// main part: drive decisions	=================================


// =========== RAMPE HOCH  ==================

		if (rampe_hoch) {

			drive_lg << "** RAMPE HOCH **" << info;

			// Sichergehen, ob der Robo noch auf der Rampe ist.
			if (imu_data[PITCH] < 5.0) {
				drive_lg << "oben angekommen" << info;

				int max_l = 320;
				int max_r = 320;

				for(auto & lps : last_line_points) {
					for(auto & point : lps) {
						if(point.x > max_r) {
							max_r = point.x;
						}
						if(point.x < max_l) {
							max_l = point.x;
						}
					}
				}

				if((320 - max_r) > (max_l - 320) && max_r > 500) {
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 120, MOTOR_BACKWARD, 160);

					while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x > IMG_WIDTH / 2 + 100)) {
						thread_delay(5);
						get_line_data(m_line_points);
					}
				} else if((320 - max_r) < (max_l - 320) && max_l < 140) {
					mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 160, MOTOR_FORWARD, 120);

					while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x < IMG_WIDTH / 2 - 100)) {
						thread_delay(5);
						get_line_data(m_line_points);
					}
				}

				else {
					turn_angle(motor_fd, imu_data, 80);
				}

				rampe_hoch = false;
				continue;
			}

			// Wenn der Roboter seitlich geneigt ist die wieder gerade ausrichten
			if (imu_data[ROLL] < -4.0) {	// Fährt nach links - linke Seite unten
				mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 180, MOTOR_FORWARD, 30);
			}
			else if (imu_data[ROLL] > 4.0) {	// Fährt nach rechts - rechte Seite unten
				mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 30, MOTOR_FORWARD, 180);
			}

			// wenn nur 1 linepoint vorhanden ist
			else if(m_line_points.size() == 1) {
				// Wenn der Roboter nicht seitlich geneigt ist, aber die Linie nicht mehr in der Mitte ist
				if (m_line_points[0].x > 360) {										// linie auf der Rampe zu weit rechts
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 180, MOTOR_FORWARD, 90);
				} else if (m_line_points[0].x < 280) {								// linie auf der Rampe zu weit links
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 90, MOTOR_FORWARD, 180);
				}
				// Ansonsten gerade fahren
				else {
					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);
				}
			}
			// Ansonsten gerade fahren
			else {
				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);
			}
		}


// ========== RAMPE RUNTER ======================

		else if (rampe_runter) {
			drive_lg << "** RAMPE RUNTER **" << info;

			if (imu_data[PITCH] > -5.0) {
				drive_lg << "unten angekommen" << info;

				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 110);
				thread_delay(1500);
				auto tbegin = get_cur_time();

				while(get_ms_since(tbegin) < 3000) {
					get_line_data(m_line_points);					 		// line data updaten

					// der Linie folgen
					if (m_line_points[0].x > 575) {										// line_points[0] rechts außen
						// Wenn sich einmal ein einziger Linepoint weit außen rechts befindet, soll sich der Roboter
						// So lange in diese Richtung drehen, wie entweder kein Linepoint vorhanden ist,
						// Oder alle linepoints noch zu weit rechts sind.
						bool done = false;
						mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 100, MOTOR_BACKWARD, 140);		// Drehung einleiten

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
						mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 140, MOTOR_FORWARD, 100);		// Drehung einleiten

						do {
							std::cout << "linie ganz links" << std::endl;
							thread_delay(5);
							get_line_data(m_line_points);							// Daten updaten
							for(auto &linepoint : m_line_points) {		// Überspringt die Schleife, wenn m_line_points leer
								if(linepoint.x > 120) done = true;			// wenn einer der line_points weit genug in der Mitte, oder rechts im Bild ist abbrechen
							}
						} while(!done);

					} else if (m_line_points[0].x > 500) {							// line_points[0] rechts
						mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 80);
					} else if (m_line_points[0].x < 140) {							// line_points[0] links
						mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 80, MOTOR_FORWARD, 160);
					} else if (get_ms_since(tbegin) > 2000) {
						sen::reset_last_movement_change();
						break;
					} else if (m_line_points[0].x > 400) {							// line_points[0] halb rechts
						mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 10);
					} else if (m_line_points[0].x < 240) {							// line_points[0] halb links
						mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 160);
					} else {																					 // line_points[0] mitte
						mot::state(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);
					}
				}
				rampe_runter = false;
				continue;
			}

			if(m_line_points.size() == 1) {
				// Wenn die Linie nicht mehr in der Mitte ist
				if (m_line_points[0].x > 520) {										// linie auf der Rampe zu weit rechts
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 60, MOTOR_BACKWARD, 40);
				} else if (m_line_points[0].x < 120) {								// linie auf der Rampe zu weit links
					mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 40, MOTOR_FORWARD, 60);
				} else if (m_line_points[0].x > 360) {										// linie auf der Rampe zu weit rechts
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 60, MOTOR_BACKWARD, 10);
				} else if (m_line_points[0].x < 280) {								// linie auf der Rampe zu weit links
					mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 60);
				} else {
					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 60);
				}
			} else {
				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 60);
			}
		}


		// Roboter gerade
		else {
			drive_lg << "** Roboter gerade **" << info;
			if (imu_data[PITCH] > 10.0) {
				drive_lg << "Rampe hoch erkannt" << info;
				rampe_hoch = true;
				continue;				// Rest der while schleife skippen
			}
			if (imu_data[PITCH] < -10.0) {
				drive_lg << "Rampe runter erkannt" << info;
				rampe_runter = true;
				continue;				// Rest der while schleife skippen
			}


// ==========================   ROBOTER IST WAAGERECHT =================================

// ======== Endzone erkennen ============

			if (sen::digital_sensor_data(IR_VORNE_L) == 0 && sen::digital_sensor_had_value(IR_VORNE_R, 750, 0, 3)) {
				drive_lg << "KANTE ERKANNT" << warn;
				thread_delay(1000);
			}
			else if (sen::digital_sensor_data(IR_VORNE_R) == 0 && sen::digital_sensor_had_value(IR_VORNE_L, 750, 0, 3)) {
				drive_lg << "KANTE ERKANNT" << warn;
				thread_delay(1000);
			}






// ========= FLASCHE ====================


			// Flasche mittig vor dem Roboter
			if(sen::analog_sensor_data(IR_MITTE) >= flasche_fahren && (sen::digital_sensor_data(IR_VORNE_L) == 0 || sen::digital_sensor_data(IR_VORNE_R) == 0)) {		// AUSKOMMENTIERT WEGEN HARDWAREFEHLER
				mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);
				drive_lg << "Objekt erkannt" << info;
				auto tbegin = get_cur_time();
				bool flasche = true;

				while(get_ms_since(tbegin) < 300) {
					thread_delay(5);
					sen::update_analog_sensordata();

					if(sen::analog_sensor_data(IR_MITTE) < flasche_fahren - 20){
						flasche = false;
						break;
					}
				}


				if(flasche) {

					drive_lg << " > FLASCHE FAHREN" << warn;
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD_NORMAL);

					while(sen::analog_sensor_data(IR_MITTE) > 110) {
						thread_delay(5);
						sen::update_analog_sensordata();
					}
					// Bremsen
					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
					thread_delay(10);
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);
					thread_delay(400);

					// Flasche links

					if(flasche_links) {
						// Links drehen
						turn_angle(motor_fd, imu_data, -50);
					} else {
						// rechts drehen
						turn_angle(motor_fd, imu_data, 50);
					}

					// Vor bis hinten rechts an Flasche
					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);

					thread_delay(2000);
					// Bremsen
					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
					thread_delay(10);
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);
					thread_delay(600);

					if(flasche_links) {
						// Rechts drehen
						turn_angle(motor_fd, imu_data, 45);
					} else {
						// links drehen
						turn_angle(motor_fd, imu_data, -45);
					}

					// Vor bis hinten rechts an Flasche
					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
					thread_delay(1900);

					// Bremsen
					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
					thread_delay(10);
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);
					thread_delay(600);

					if(flasche_links) {
						// Rechts drehen
						turn_angle(motor_fd, imu_data, 55);
					} else {
						// links drehen
						turn_angle(motor_fd, imu_data, -55);
					}
					thread_delay(50);
					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
					thread_delay(200);
				}
				continue;
			}

			/*
			// AUSKOMMENTIERT WEGEN HARDWAREFEHLER

			// Objekt, das sehr breit ist direkt vor dem Roboter, oder die Flasche nicht mittig
			else if(analog_sensor_data >= flasche_fahren && (digital_sensor_data[IR_VORNE_L] == 0 || digital_sensor_data[IR_VORNE_R] == 0)) {
				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 90);
				thread_delay(750);
				mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);
				thread_delay(7);
				continue;
			}

			*/


//  =========== GRUEN ========================

			if (m_grstate == GRUEN_BEIDE) {

				drive_lg << "green BOTH" << warn;

				while (m_grstate != GRUEN_NICHT) {		// Solange grstate nicht GRUEN_NICHT ist
					get_gruen_data(m_grcenter, m_grstate);											// gruen werte aktualisieren
					if (m_grcenter.y > 350) {						// nah am grünpunkt; m_grcenter ist im unteren bildbereich
						break;
					} else if (m_grcenter.x < 310) {						// m_grcenter im linken Bildbereich
						mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 0, MOTOR_FORWARD, 110);			// links Kurve
					} else if (m_grcenter.x > 330) {						// m_grcenter im rechten Bildbereich
						mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 110, MOTOR_FORWARD, 0);			// rechts Kurve
					} else {																	// m_grcenter im mittleren, oberen Bildbereich
						mot::state(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);								// langsam vorwärts
					}
				}

				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);		// beide Motoren vorwärts, pwm: 180
				thread_delay(400);																					// delay 500ms
				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 180);		// beide Motoren vorwärts, pwm: 180
				thread_delay(10);
				mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);		// beide Motoren vorwärts, pwm: 180
				thread_delay(500);
				turn_angle(motor_fd, imu_data, 180);			// 180° Drehen
				thread_delay(250);												// Warten für aktuelles Bild / damit der Kamerathread hinterher kommt

			} else if (m_grstate == GRUEN_LINKS && m_grcenter.y > 480 - 150) {			// m_grcenter im unteren Bildbereich + m_grstate = GRUEN_LINKS

				drive_lg << "green LEFT" << warn;					// in debug.log loggen

				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 150);				// beide Motoren vorwärts, pwm: 150
				thread_delay(300);						// delay 300ms
				mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 190, MOTOR_FORWARD, 150);		// linkskurve, links schneller
				thread_delay(700);						// delay 500ms

				get_line_data(m_line_points);
				for(cv::Point & linepoint : m_line_points) {
					if(linepoint.x < 190) {
						thread_delay(200);
						break;
					}
				}

			} else if (m_grstate == GRUEN_RECHTS && m_grcenter.y > 480 - 150) {				// m_grcenter im unteren Bildbereich + m_grstate = GRUEN_RECHTS

				drive_lg << "green RIGHT" << warn;			// in debug.log loggen

				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 150);			// beide Motoren vorwärts, pwm: 150
				thread_delay(300);					// delay 300ms
				mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 150, MOTOR_BACKWARD, 190);		// rechtskurve, rechts schneller
				thread_delay(700);					// delay 500ms

				get_line_data(m_line_points);
				for(cv::Point & linepoint : m_line_points) {
					if(linepoint.x > 450) {
						thread_delay(200);
						break;
					}
				}
			}

// ======== LINIE ================

			else if(m_line_points.size() == 1) {							// wenn nur 1 linepoint vorhanden ist

				drive_lg << "single Line point: " << m_line_points[0] << info;
				//			cout << "Different value -> check motor output for line" << endl;

				// Der Roboter hat sich 5 Sekunden weniger als 5°/Sekunde bewegt, steht aber nicht mittig auf der Linie:
				// Für 0,5 Sekunden mit vollem Tempo zurück
				if(sen::get_last_movement_seconds() > 4.0 && (m_line_points[0].x < 300 || m_line_points[0].x > 340)) {
					mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 255, MOTOR_BACKWARD, 255);
					thread_delay(400);
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);
					sen::reset_last_movement_change();				// resetten, da sonst der Roboter eventuell dauerhaft rückwärts fährt, wenn er sich dabei nicht dreht
					thread_delay(7);		// Delay für i2c bus
				}

				// der Linie folgen
				if (m_line_points[0].x > 575) {										// line_points[0] rechts außen
					// Wenn sich einmal ein einziger Linepoint weit außen rechts befindet, soll sich der Roboter
					// So lange in diese Richtung drehen, wie entweder kein Linepoint vorhanden ist,
					// Oder alle linepoints noch zu weit rechts sind.
					//bool done = false;
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 140, MOTOR_BACKWARD, 140);		// Drehung einleiten
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
					mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 140, MOTOR_FORWARD, 140);		// Drehung einleiten
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
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 80);
				} else if (m_line_points[0].x < 140) {							// line_points[0] links
					mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 80, MOTOR_FORWARD, 160);
				} else if (m_line_points[0].x > 400) {							// line_points[0] halb rechts
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 10);
				} else if (m_line_points[0].x < 240) {							// line_points[0] halb links
					mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 160);
				} else if (m_line_points[0].x > 340) {							// line_points[0] mitte rechts
					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 160, MOTOR_OFF, 0);
				} else if (m_line_points[0].x < 300) {						 // line_points[0] mitte links
					mot::dir_pwm_both(motor_fd, MOTOR_OFF, 0, MOTOR_FORWARD, 160);
				} else {																					 // line_points[0] mitte
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);
				}

			} else if(m_line_points.size() == 0 && last_line_points[1].size() == 1) {						// linie verloren, vorher nur 1 linepoint
				//drive_lg << "lost line, last line singe line point: " << last_line_points[1][0];

				if (last_line_points[1][0].x > 560) {				// letzter linepoint rechts außen: nach rechts fahren, bis linie wieder gefunden
					drive_lg << " -  correction right" << info;

					mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 120);

					while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x > IMG_WIDTH / 2 + 100)) {
						thread_delay(5);
						get_line_data(m_line_points);
					}

					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
					thread_delay(500);
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);

				} else if (last_line_points[1][0].x < 80) {				// letzter linepoint links außen: nach links fahren, bis linie wieder gefunden
					drive_lg << " -   correction left" << info;

					mot::dir_pwm_both(motor_fd, MOTOR_BACKWARD, 120, MOTOR_FORWARD, 160);

					while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x < IMG_WIDTH / 2 - 100)) {
						thread_delay(5);
						get_line_data(m_line_points);
					}

					mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
					thread_delay(500);
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_OFF);

				} else if (last_line_points[1][0].x >= 140 && last_line_points[1][0].x <= 500) {

					float movement = sen::get_movement();
					drive_lg << "Last movement: " << sen::get_movement() << info;

					if(movement > 10) {
						mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 140, MOTOR_FORWARD, 0);
						thread_delay(500);
					} else if(movement < -10) {
						mot::dir_pwm_both(motor_fd, MOTOR_FORWARD, 0, MOTOR_FORWARD, 140);
						thread_delay(500);
					}

					mot::state(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);

				} else {							// wenn der letzte linepoint mittig war (Lücke) weiter gerade fahren
					mot::state(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);			// vorwärts mit festgelegtem Standardtempo
				}

			} else if(m_line_points.size() == 0 && last_line_points[1].size() > 1) {						// linie verloren, vorher mehr als 1 line_point

				mot::dir_pwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 120);				// vorwärts mit festgelegtem Standardtempo
				thread_delay(600);

			} else {			// wenn linepoints.size() > 1 und kein grünpunkt, grade fahren
				mot::state(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);				// vorwärts mit festgelegtem Standardtempo
				//drive_lg << "driving forward, " << m_line_points.size() << " line points" << info;
			}
		}

		thread_delay(7);			// delay, da sonst der i2c buffer überschrieben wird und der Motorarduino falsche werte bekommt
	}
}
