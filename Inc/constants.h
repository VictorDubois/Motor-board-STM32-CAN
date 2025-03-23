//add constant from libmd25 use in main
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

/*****************************
 *         PARAMETERS        *
 *****************************/

// At the moment, we keep floats as we are not (yet) in optimization mode
// these constants must be floats, otherwise the Rz angle is discrete

#define TICKS_PER_REVOLUTION 8192.0 //360    // Nb ticks per wheel revolution
#define WHEEL_RADIUS 19.91               // Diameter of the wheel (mm) UNUSED !!!
#define DIST_PER_REVOLUTION_THEORY WHEEL_RADIUS*M_PI // 217.879 // 214.66 // 206.5//214.635// 210.481 //304.734 // Distance traveled for a full 	wheel revolution (in mm)
#define DIST_PER_REVOLUTION DIST_PER_REVOLUTION_THEORY/1.2471814155578613 // 217.879 // 214.66 // 206.5//214.635// 210.481 //304.734 // Distance traveled for a full 	wheel revolution (in mm)
#define VOIE 205.10
#define TICKS_PER_DEG VOIE * 0.96007158213* TICKS_PER_REVOLUTION/(DIST_PER_REVOLUTION_THEORY*360.0) //450.0//91.472 // 91.290 // 91.108345 // 91.001144 // 91.055 	// 89.28//89//89.06 //4.86          // Nb of diff tick (enc1 - enc2) it takes to rotate 1 deg (old value: 4.7)
#define TICKS_OVERFLOW 65536.0
#define TICKS_half_OVERFLOW TICKS_OVERFLOW/2.0
#endif

//0.96224690902
//(1.03775309098)
