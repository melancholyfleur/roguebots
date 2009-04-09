/* *********************
 *  Project 3
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
#include "PID.h"
#include "error.h"
#include "filter.h"

#define FULLCONTROL 1

#define TIMEOUT 100
#define BUFFER_DIST .5

/* GLOBAL VARS */
turret_comm_t *r;
create_comm_t *c;
//char filename[]    = "waypoints.txt"; // file that contains (x,y) coords for waypoints
//char *file_tokens  = NULL;	      // contains tokenized waypoint coordinates
float error_dist   = 0.0;             // current x error
float error_a      = 0.0;             // current angle error
float ir_r         = 0.0;             // right IR value
float ir_l         = 0.0;             // left IR value
float sonar_r      = 0.0;             // front sonar value
float sonar_l      = 0.0;             // back sonar value
float sonar_error  = 0.0;	      // error between sonar[0] and sonar[1]
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
char direction[10];

	/*
	angle[0] = M_PI;	//e to w
	angle[1] = -(M_PI/2);	//w to n
	angle[2] = -(M_PI/2);	//n to e
	angle[3] = (M_PI/2);	//e to n
	angle[4] = -(M_PI/2);	//n to e
	angle[5] = 0.0;		//e to e
	angle[6] = (M_PI/2);	//e to n
	angle[7] = 0.0;		//n to n
	angle[8] = -(M_PI/2);	//n to e
	angle[9] = 0.0;		//e to e
	*/

//float getDistance(waypoint, waypoint);
void Move(create_comm_t*, turret_comm_t*, direction[], int);
//float getAngle(float);
float error_ir();
float error_sonar();

#endif
