/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  PID.h
 ********************* */

#ifndef PID_H
#define PID_H

#include <stdlib.h>

#define PID_KP 1.0
#define PID_KI 0.8
#define PID_KD 0.0
#define PIDA_KP 1.0
#define PIDA_KI 0.0001
#define PIDA_KD 0.0

float PID(float);
float PID_A(float);

#endif

