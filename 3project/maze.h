/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  maze.h
 ********************* */

#ifndef MAZEH
#define MAZEH

#include "PID.h"
#include "filter.h"
#include "TurretAPI.h"
#include "create_comms.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#define FULLCONTROL 1

#define TIMEOUT 100
#define BUFFER_DIST .5
#define WHICH_SENSOR 0		// 0 = sonar, 1 = ir
#define START_DIR 2
#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

/* Global Variables */
turret_comm_t *r;
create_comm_t *c;
float angle_error  = 0.0;
float sonar_error  = 0.0;
float ir_error     = 0.0;
float ir_r         = 0.0;
float ir_l         = 0.0;
float sonar_r      = 0.0;
float sonar_l      = 0.0;
float dist_error;
float vx           = 0.0;             // x velocity
float va           = 0.0;             // angle velocity
float uVectorX     = 0.0;	      // 
float uVectorY     = 0.0;	      // U vector - calculate angle robot must turn
float uVectorZ     = 0.0;	      //
float vVectorX     = 0.0;	      // 
float vVectorY     = 0.0;	      // V vector - calculate angle robot must turn
float vVectorZ     = 0.0;	      //
float dot_product  = 0.0;	      // dot product of vectors U and V
float mag_u        = 0.0;	      // magnitude of vector U
float mag_v	   = 0.0;	      // magnitude of vector V
float theta	   = 0.0;	      // theta value for angle betwee U and V
float dist         = 0.0;	      // distance between two cartisian coords
float new_angle    = 0.0;	      // new angle that the robot must turn
float curr_angle   = 0.0;
float delta        = 0.0;
int position	   = 0;	          // ox, oy, and oa of robot
int currDirection  = 0;
int nextDirection  = START_DIR;
int direction      = 0;
int directions[10];

/* Function Declarations */
void Turn(create_comm_t*);
void MoveToNeighboringCell(create_comm_t*, turret_comm_t*, int);
void AdjustPosition(create_comm_t*, turret_comm_t*);
float error_sonar(turret_comm_t *);
float error_ir(turret_comm_t *);
float error_tx(create_comm_t*, int);
float error_ta(create_comm_t*, float);

#endif


