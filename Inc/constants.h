//add constant from libmd25 use in main
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

/*****************************
 *         PARAMETERS        *
 *****************************/

// At the moment, we keep floats as we are not (yet) in optimization mode
#define KV2

#ifdef KV1
#define TICKS_PER_REVOLUTION 4096 	// Nb ticks per wheel revolution
#define WHEEL_DIAM 68               	// Diameter of the wheel (mm) UNUSED !!!
#define DIST_PER_REVOLUTION 216.2695  	// Distance traveled for a full wheel revolution (in mm)
#define TICKS_PER_DEG 91.472       	// Nb of diff tick (enc1 - enc2) it takes to rotate 1 deg (old value: 4.7)
#endif

#ifdef KV2
#define TICKS_PER_REVOLUTION 8192.0 	// Nb ticks per wheel revolution
#define WHEEL_RADIUS 19.91              // Diameter of the wheel (mm) UNUSED !!!
#define DIST_PER_REVOLUTION_THEORY WHEEL_RADIUS*M_PI // Distance traveled for a fulli wheel revolution (in mm)
#define DIST_PER_REVOLUTION DIST_PER_REVOLUTION_THEORY/1.2471814155578613 // Distance traveled for a full wheel revolution (in mm)
#define VOIE 205.10
#define TICKS_PER_DEG VOIE * 0.96007158213* TICKS_PER_REVOLUTION/(DIST_PER_REVOLUTION_THEORY*360.0) // Nb of diff tick (enc1 - enc2) it takes to rotate 1 deg (old value: 4.7)
#endif

#define TICKS_OVERFLOW 65536.0
#define TICKS_half_OVERFLOW TICKS_OVERFLOW/2.0
#endif
