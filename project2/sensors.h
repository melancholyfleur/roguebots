/*
 *  sensors.h
 *  
 *
 *  Created by Marylou Kunkle on 2/24/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef SENSORSH
#define SENSORSH

#include <stdio.h>
#include <math.h>
#include <libplayerc/playerc.h>

#define PID_KP 1.0
#define PID_KI 0.05
#define PID_KD 0.0
#define PIDA_KP 0.5
#define PIDA_KI 0.0001
#define PIDA_KD 0.0

#define MOVE1 3.3
#define ANGLE1 0.0
#define TURN1 (M_PI/2)
#define MOVE2 3.04
#define ANGLE2 0.0
#define TURN2 (M_PI/2.5)
#define MOVE3 3.8
#define ANGLE3 0.0

#define TAPS  4			// how many filter taps

/* GLOBAL VARS */
playerc_client_t *client;         	// client used to connect to robot
playerc_position2d_t *position2d;	// store the px,py,pa values for robot
playerc_bumper_t *bumper;         	// bumper client for robot
playerc_sonar_t *turret_sonar;		// sonar client
playerc_ir_t *turret_ir;			// ir client
float prev_error   = 0.0;         // previous error for x/y destination
float prev_error_a = 0.0;         // previous error for angle
float error_x      = 0.0;         // current x error
float error_y      = 0.0;         // current y error
float error_a      = 0.0;         // current angle error
float ir_r         = 0.0;         // right IR value
float ir_l         = 0.0;         // left IR value
float pid          = 0.0;         // pid controller value
float vx           = 0.0;         // x velocity
float va           = 0.0;         // angle velocity

typedef struct 
{
	float    coefficients[TAPS];
  	unsigned next_sample;
  	float    samples[TAPS];
} filter_t;

filter_t *firFilterCreate();
float firFilter(filter_t*, float);
float error_tx(playerc_position2d_t*, float);
float error_ta(playerc_position2d_t*, float);
float PID(float);
float PID_A(float);
float Move(playerc_client_t*, float, float);
float Turn(playerc_client_t*, float);
float error_ir_LEFT();
float error_ir_RIGHT();

#endif