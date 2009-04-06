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
#include <string.h>
#include <math.h>
#include "TurretAPI.h"
#include "create_comms.h"
#include <unistd.h>

#define FULLCONTROL 1

#define TIMEOUT 100
#define BUFFER_DIST .5
#define PID_KP 1.0
#define PID_KI 0.8
#define PID_KD 0.0
#define PIDA_KP 0.5
#define PIDA_KI 0.0001
#define PIDA_KD 0.0

#define TAPS  8			// how many filter taps

#define WHICH_SENSOR 0		// 0 = sonar, 1 = ir

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
//char filename[]    = "waypoints.txt"; // file that contains (x,y) coords for waypoints
char *file_tokens  = NULL;	      // contains tokenized waypoint coordinates
float prev_error   = 0.0;             // previous error for x/y destination
float prev_error_a = 0.0;             // previous error for angle
float error_dist   = 0.0;             // current x error
float error_a      = 0.0;             // current angle error
float ir_r         = 0.0;             // right IR value
float ir_l         = 0.0;             // left IR value
float sonar_r      = 0.0;             // front sonar value
float sonar_l      = 0.0;             // back sonar value
float sonar_error  = 0.0;	      // error between sonar[0] and sonar[1]
float pid          = 0.0;             // pid controller value
float vx           = 0.0;             // x velocity
float va           = 0.0;             // angle velocity
float uX	   = 0.0;	      // 
float uY	   = 0.0;	      // U vector - calculate angle robot must turn
float uZ           = 0.0;	      //
float vX	   = 0.0;	      // 
float vY	   = 0.0;	      // V vector - calculate angle robot must turn
float vZ           = 0.0;	      //
float dot_product  = 0.0;	      // dot product of vectors U and V
float mag_u	   = 0.0;	      // magnitude of vector U
float mag_v	   = 0.0;	      // magnitude of vector V
float theta	   = 0.0;	      // theta value for angle betwee U and V
float dist         = 0.0;	      // distance between two cartisian coords
float new_angle    = 0.0;	      // new angle that the robot must turn
float curr_angle   = 0.0;
int position	   = 0;	              // ox, oy, and oa of robot
int num_waypoints  = 0;	              // number of waypoints to navigate to

waypoint waypoints[9];     //array of waypoints

filter_t *firFilterCreate();
float firFilter(filter_t*, float);
float error_t(create_comm_t*, waypoint);
float error_ta(create_comm_t*, float);
float PID(float);
float PID_A(float);
float error_ir();
float error_sonar();
float getDistance(waypoint, waypoint);
void Move(create_comm_t*, turret_comm_t*, waypoint[], int);

#endif
