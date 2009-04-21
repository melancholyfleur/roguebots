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

#define NONE 0
#define FRONT1 8
#define RIGHT1 2
#define BACK1 4
#define LEFT1 1
#define FRONTRIGHT 10
#define BACKRIGHT 6
#define BACKLEFT 5
#define FRONTLEFT 9
#define RIGHTLEFT 3
#define FRONTBACK 12
#define RIGHT3 14
#define BACK3 7
#define LEFT3 13
#define FRONT3 11
#define FOURFRONT 15

#define FRONT_SET 1
#define BACK_SET 2
#define RIGHT_SET 4
#define LEFT_SET 8

typedef enum {
	unsigned int NONE, FRONT1, RIGHT1, BACK1, LEFT1, FRONTRIGHT, BACKRIGHT, BACKLEFT, FRONTLEFT, RIGHTLEFT, FRONTBACK, RIGHT3, BACK3, LEFT3, FRONT3, FOURWALLS;
} currentConfiguration;

currentConfiguration currConfig;

float error_ir(turret_comm_t*);
float error_sonar(turret_comm_t*);
float error_tx(create_comm_t*, float, int);
float error_ta(create_comm_t*, float);
int WhatDoISee(turret_comm_t*);

#endif

