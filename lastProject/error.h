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

#define FRONT_SET 8
#define BACK_SET 4
#define RIGHT_SET 2
#define LEFT_SET 1

int* openDirs;

unsigned int currConfig;

float error_ir(turret_comm_t*);
float error_sonar(turret_comm_t*);
float error_tx(create_comm_t*, float, int);
float error_ta(create_comm_t*, float);
int* WhatDoISee(turret_comm_t*);

#endif

