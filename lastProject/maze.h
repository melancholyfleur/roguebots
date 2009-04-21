/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  maze.h
 ********************* */

#ifndef MAZEH
#define MAZEH

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include "TurretAPI.h"
#include "create_comms.h"
#include "filter.h"
#include "error.h"
#include "PID.h"

#define FULLCONTROL 1

#define TIMEOUT 100
#define BUFFER_DIST 0.02
#define WHICH_SENSOR 0		// 0 = sonar, 1 = ir
#define START_DIR 2
#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

typedef struct {
	float x;
	float y;
} currentPosition;

/* Global Variables */
turret_comm_t *r;
create_comm_t *c;
float dist_error     = 0.0;
float angle_error    = 0.0;
float vx             = 0.0;             // x velocity
float va             = 0.0;             // angle velocity
float uVectorX       = 0.0;	      // 
float uVectorY       = 0.0;	      // U vector - calculate angle robot must turn
float uVectorZ       = 0.0;	      //
float vVectorX       = 0.0;	      // 
float vVectorY       = 0.0;	      // V vector - calculate angle robot must turn
float vVectorZ       = 0.0;	      //
float dot_product    = 0.0;	      // dot product of vectors U and V
float mag_u          = 0.0;	      // magnitude of vector U
float mag_v	         = 0.0;	      // magnitude of vector V
float theta	         = 0.0;	      // theta value for angle betwee U and V
float dist           = 0.0;	      // distance between two cartisian coords
float new_angle      = 0.0;	      // new angle that the robot must turn
float curr_angle     = 0.0;
float target_angle   = 0.0;
float distToMove     = 0.0;
float distBtwnCells  = 0.7;
int position	       = 0;	          // ox, oy, and oa of robot
int currDirection    = 0;
int nextDirection    = START_DIR;
int direction        = 0;
int directions[10];
currentPosition currPos;

/* Function Declarations */
int Turn(create_comm_t*);
int MoveToNeighboringCell(create_comm_t*, turret_comm_t*, int);
void AdjustPosition(create_comm_t*, turret_comm_t*);
void signal_interrupt(int);

#endif


