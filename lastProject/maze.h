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
#include "map.h"

#define FULLCONTROL 1

#define TIMEOUT 100
#define BUFFER_DIST 0.02
#define WHICH_SENSOR 0		// 0 = sonar, 1 = ir

/* Global Variables */
turret_comm_t *r;
create_comm_t *c;
float dist_error     = 0.0;
float angle_error    = 0.0;
float vx             = 0.0;             // x velocity
float va             = 0.0;             // angle velocity
float distToMove     = 0.0;
float distBtwnCells  = 0.7;
int position	       = 0;	          // ox, oy, and oa of robot
float angle          = 0.0;

/* Function Declarations */
int Turn(create_comm_t*,float);
int MoveToNeighboringCell(create_comm_t*, turret_comm_t*);
void AdjustPosition(create_comm_t*, turret_comm_t*);
void signal_interrupt(int);

#endif

