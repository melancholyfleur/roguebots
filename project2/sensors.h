/* *********************
 *  Project 2
 *  Marylou Kunkle
 *  Lenny Kramer
 *  sensors.h
 ********************* */

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

#define TAPS  4			// how many filter taps

typedef struct
{
	float x;
	float y;
} waypoint;				// waypoint is an (x,y) coordinate on plane

typedef struct 
{
	float    coefficients[TAPS];
  	unsigned next_sample;
  	float    samples[TAPS];
} filter_t;

/* GLOBAL VARS */
playerc_client_t *client;         	// client used to connect to robot
playerc_position2d_t *position2d;	// store the px,py,pa values for robot
playerc_bumper_t *bumper;         	// bumper client for robot
playerc_sonar_t *turret_sonar;		// sonar client
playerc_ir_t *turret_ir;			// ir client
float prev_error   = 0.0;         // previous error for x/y destination
float prev_error_a = 0.0;         // previous error for angle
float error_dist   = 0.0;         // current x error
float error_a      = 0.0;         // current angle error
float ir_r         = 0.0;         // right IR value
float ir_l         = 0.0;         // left IR value
float pid          = 0.0;         // pid controller value
float vx           = 0.0;         // x velocity
float va           = 0.0;         // angle velocity
waypoint waypoints[8];            // array of targets

waypoints[0].x = 0.0;				//origin
waypoints[0].y = 0.0;
waypoints[1].x = 7.3;
waypoints[1].y = 0.0;
waypoints[2].x = 7.62 + 7.3;
waypoints[2].y = 7.62;
waypoints[3].x = 18.89 + 7.62 + 7.3;
waypoints[3].y = 7.62;
waypoints[4].x = 18.89 + 7.62 + 7.3;
waypoints[4].y = 7.62 - 10.668;
waypoints[5].x = 18.89 + 7.62 + 7.3 + 3.9624;
waypoints[5].y = 7.62 - 10.668 - 9.144;
waypoints[6].x = 18.89 + 7.62 + 7.3 + 3.9624 - 22.86;
waypoints[6].y = 7.62 - 10.668 - 9.144;
waypoints[7].x = 18.89 + 7.62 + 7.3 + 3.9624 - 22.86;
waypoints[7].y = 7.62 - 10.668 - 9.144 + 12.192;

filter_t *firFilterCreate();
float firFilter(filter_t*, float);
float error_t(playerc_position2d_t*, waypoint);
float error_ta(playerc_position2d_t*, float);
float PID(float);
float PID_A(float);
float Move(playerc_client_t*, waypoint);
float Turn(playerc_client_t*, float);
float error_ir_LEFT();
float error_ir_RIGHT();

#endif