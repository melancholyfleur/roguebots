/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  error.h
 ********************* */

#ifndef ERROR_H
#define ERROR_H

#include <stdlib.h>
#include <stdio.h>
#include "TurretAPI.h"
#include "create_comms.h"

typedef struct
{
	float x;
	float y;
}waypoint;				// waypoint is an (x,y) coordinate on plane

float error_tx(create_comm_t*, int);
float error_ta(create_comm_t*, float);
float error_ir(turret_comm_t*);
float error_sonar(turret_comm_t*);


#endif

