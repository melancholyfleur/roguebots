/* *********************
 *  Project 2
 *  Marylou Kunkle
 *  Lenny Kramer
 *  sensors.h
 ********************* */

#ifndef SENSORSH
#define SENSORSH

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "TurretAPI.h"
#include "create_comms.h"
#include <unistd.h>

#define FULLCONTROL 1

#define TIMEOUT 10000000

#define PID_KP 1.0
#define PID_KI 0.05
#define PID_KD 0.0
#define PIDA_KP 0.5
#define PIDA_KI 0.0001
#define PIDA_KD 0.0

#define TAPS  8			// how many filter taps

typedef struct
{
	float x;
	float y;
}waypoint;				// waypoint is an (x,y) coordinate on plane

typedef struct 
{
	float    coefficients[TAPS];
  	unsigned next_sample;
  	float    samples[TAPS];
} filter_t;

/* GLOBAL VARS */
turret_comm_t *r;
create_comm_t *c;
filter_t *filter;
float prev_error   = 0.0;         // previous error for x/y destination
float prev_error_a = 0.0;         // previous error for angle
float error_dist   = 0.0;         // current x error
float error_a      = 0.0;         // current angle error
float ir_r         = 0.0;         // right IR value
float ir_l         = 0.0;         // left IR value
float sonar_f      = 0.0;         // front sonar value
float sonar_b      = 0.0;         // back sonar value
float pid          = 0.0;         // pid controller value
float vx           = 0.0;         // x velocity
float va           = 0.0;         // angle velocity

waypoint waypoints[8];     //array of waypoints

filter_t *firFilterCreate();
float firFilter(filter_t*, float);
float error_t(create_comm_t*, waypoint);
float error_ta(create_comm_t*, float);
float PID(float);
float PID_A(float);
float Move(create_comm_t*, turret_comm_t*, waypoint);
float error_ir_LEFT();
float error_ir_RIGHT();
float error_sonar_FRONT(turret_comm_t*);
float error_sonar_BACK(turret_comm_t*);


#endif
