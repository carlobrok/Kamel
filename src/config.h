#ifndef CONFIG_H
#define CONFIG_H

// Debug defines

#define ON_PI				// Wenn definiert, geht der Videooutput über den VideoServer
#define VISUAL_DEBUG		// Wenn definiert, werden Debuglinien und Konturen in img_rgb gezeichnet
#define LOG_FILE			// Wenn definiert, versucht das Programm alle Sensordaten in eine, Outputdatei zu loggen, wenn der Name übergeben wurde


// Allgemeine defines

#define IMG_HEIGHT 480
#define IMG_WIDTH 640


// I2C define
#define I2C_MOTOR_REFRESH_TIME 100     // Zeit in ms nach der die Daten über I2C beim aufrufen der Funktion zwingend aktualisiert werden

// IMU defines
#define IMU_BAUD 115200
#define IMU_REFRESH_DELAY 10
#define AMOUNT_IMU_DATA 3
#define YAW 2
#define PITCH 1
#define ROLL 0

#endif
