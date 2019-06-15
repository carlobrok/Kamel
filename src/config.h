#ifndef _CONFIG_H
#define _CONFIG_H

//#define ON_PI				// Wenn definiert, geht der Videooutput über den VideoServer
#define VISUAL_DEBUG		// Wenn definiert, werden Debuglinien und Konturen in img_rgb gezeichnet

// Grünpunkt defines

//#define DEBUG_GRUEN

#define LOWER_GREEN Scalar(50, 60, 25)
#define UPPER_GREEN Scalar(90, 255, 115)

#define GRUEN_NICHT 0
#define GRUEN_BEIDE 3
#define GRUEN_LINKS 1
#define GRUEN_RECHTS 2

#define NUM_ITERATIONS_BLACK_POINTS 30


// Line defines

#define ELLIPSE_THICKNESS 30		// thickness of the ellipse in pixels


#endif
