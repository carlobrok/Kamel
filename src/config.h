#ifndef CONFIG_H
#define CONFIG_H

// Debug defines

#define ON_PI				// Wenn definiert, geht der Videooutput über den VideoServer
#define VISUAL_DEBUG		// Wenn definiert, werden Debuglinien und Konturen in img_rgb gezeichnet
#define LOG_FILE			// Wenn definiert, versucht das Programm alle Sensordaten in eine, Outputdatei zu loggen, wenn der Name übergeben wurde


// Grünpunkt defines

#define LOWER_GREEN cv::Scalar(50, 60, 25)
#define UPPER_GREEN cv::Scalar(90, 255, 115)

#define GRUEN_NICHT 0
#define GRUEN_BEIDE 3
#define GRUEN_LINKS 1
#define GRUEN_RECHTS 2

#define NUM_ITERATIONS_BLACK_POINTS 30


// Line defines

#define ELLIPSE_THICKNESS 30		// thickness of the ellipse in pixels
#define THRESH_BLACK 70

#endif
