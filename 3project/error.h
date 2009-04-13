/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  error.h
 ********************* */

#ifndef ERROR_H
#define ERROR_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "TurretAPI.h"
#include "create_comms.h"
#include "filter.h"


float error_ir(turret_comm_t*);
float error_sonar(turret_comm_t*);
float error_tx(create_comm_t*, float);
float error_ta(create_comm_t*, float);

#endif

