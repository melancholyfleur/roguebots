/* *********************
 *  Project 2
 *  Marylou Kunkle
 *  Lenny Kramer
 *  maze.c
 ********************* */

#include "maze.h"

int main(int argc, const char **argv)
{
	printf("entered program\n");
	/* allocate device objects */
	c = create_create("/dev/ttyS2");
	r = turret_create();

	/* open the create serial comm  */
	if(create_open(c,FULLCONTROL) < 0) {
		printf("create open failed\n");
	 	return(-1);
	}

	/* Open the i2c device */
	if(turret_open(r) < 0) {
		printf("failed to connect to robostix\n");
		return(-1);
	}

	/* init the robostix board interfaces */
	printf("initialize turret\n");
	turret_init(r);	
	// sonar
	if(!WHICH_SENSOR){
		turret_SetServo(r,90);
	}
	//ir
	else{
	  	turret_SetServo(r,0);
	}
	
	filter = firFilterCreate();
	printf("created a filter\n");

	currDirection = START_DIR;
	directions[0] = WEST;
	directions[1] = NORTH;
	directions[2] = EAST;
	directions[3] = NORTH;
	directions[4] = EAST;
	directions[5] = EAST;
	directions[6] = NORTH;
	directions[7] = NORTH;
	directions[8] = EAST;
	directions[9] = EAST;

	signal(SIGINT,signal_interrupt);

	printf("entering move-for-loop\n");
	int i;
	for(i = 0; i < sizeof(directions); i++)
	{
		direction = directions[i];
		if(MoveToNeighboringCell(c, r, direction))
			break;	
	}
	
	// Shutdown and tidy up
	create_set_speeds(c,0.0,0.0);
	create_close(c);
	create_destroy(c);
	turret_close(r);
	turret_destroy(r);
	exit(0);
	
	return 0;
}

void signal_interrupt(int arg)
{
	printf("Received interrupt. Exiting Programming.\n");
	create_set_speeds(c,0.0,0.0);
	create_close(c);
	create_destroy(c);
	turret_close(r);
	turret_destroy(r);
	exit(0);
}

void AdjustPosition(create_comm_t* rb, turret_comm_t* tr) {
	float sonarErrorTemp = error_sonar(tr);
	printf("in AdjustPosition\nsonar error: %f\n", sonarErrorTemp);
	if(sonarErrorTemp < 0.0)
	{
		va += (M_PI/16);
		printf("increasing va: %f\n",va);
		create_set_speeds(rb,0.0,va);
	}
	else if(sonarErrorTemp > 0.0)
	{
		va -= (M_PI/16);
		printf("decreasing va: %f\n",va);
		create_set_speeds(rb,0.0,va);	
	}	
}

/* Move() 
 * client:  client to connect to robot
 * distance: x-coordinate that robot should aim for;
 * angle: angle that robot should stay at;
 */
int MoveToNeighboringCell(create_comm_t* device, turret_comm_t* turret, int target)
{
	printf("in movetoneighbor function\n");
	nextDirection = target;

	//this sets the distance the robot will move based on its direction
	//and the universal coordinate system
	position = create_get_sensors(device, TIMEOUT);
	currPos.x = device->ox;
	currPos.y = device->oy;
	if(Turn(device)){
		return 1;
	}
	if(target == NORTH){
		distToMove = currPos.y + distBtwnCells;
	}
	else if(target == SOUTH){
		distToMove = currPos.y + distBtwnCells;
	}
	else if(target == WEST){
		distToMove = currPos.x + distBtwnCells;
	}
	else if(target == EAST){
		if(currPos.x < 0.0){distToMove = distBtwnCells;}
		distToMove = distBtwnCells + currPos.x;
	}
	printf("distToMove: %f\n",distToMove);
	dist_error = error_tx(device, distToMove, target);
	printf("dist_error: %f\n",dist_error);
	//
	
	while(dist_error > BUFFER_DIST)
	{
		printf("in move while loop\n");
		position = create_get_sensors(device, TIMEOUT);
		
		dist_error = error_tx(device, distToMove, target);
		printf("dist_error: %f\n", dist_error);
		
		vx = PID(dist_error);
		printf("vx: %f\n", vx);
		
		va = PID_A(angle_error);
		printf("va: %f\n", va);
		
		if(vx > 1.0) {vx = 0.5;}

		create_set_speeds(device, vx, va);
		
		if(device->bumper_left == 1 || device->bumper_right == 1) 
		{
			create_set_speeds(device, 0.0, 0.0);
			printf("bumper brake\n");
			return 1;
		}

	}
	return 0;
	
	AdjustPosition(device, turret);
	unsigned int config = WhatDoISee(turret);
	printf("I see this: %u", config);
}

int Turn(create_comm_t* robot) {
	printf("in turn function\n");	
	// get delta angle
	if((currDirection == NORTH && nextDirection == WEST) ||
		(currDirection == SOUTH && nextDirection == WEST)  ||
		(currDirection == EAST && nextDirection == WEST))
	{
		target_angle = M_PI;
		printf("turning pi\n");
	}
		
	else if((currDirection == SOUTH && nextDirection == EAST) ||
		(currDirection == NORTH && nextDirection == EAST) ||
		(currDirection == WEST && nextDirection == EAST)   )
	{
		target_angle = 0.0;
		printf("turning 0.0\n");
	}
		
	else if((currDirection == EAST && nextDirection == NORTH) || 
		(currDirection == SOUTH && nextDirection == NORTH) ||
		(currDirection == WEST && nextDirection == NORTH)  )
	{
		target_angle = M_PI/2;
		printf("turning pi/2\n");
	}
	else if((currDirection == NORTH && nextDirection == SOUTH) ||
		(currDirection == WEST && nextDirection == SOUTH) )
	{
		target_angle = (3*M_PI)/2;
  	}
	else
	{
		target_angle = 0.0;
		printf("no need to turn\n");
	}


	// rotate bot by delta
	angle_error = error_ta(robot, target_angle);
	while (fabs(angle_error) > 0.3)
	{
		position = create_get_sensors(robot, TIMEOUT);
		printf("in turn while loop\n");
		angle_error = error_ta(robot, target_angle);
		printf("angle_error: %f\n",angle_error);
		va = PID_A(angle_error);
		printf("va: %f\n",va);
		create_set_speeds(robot, 0.0, va);
		if(robot->bumper_left == 1 || robot->bumper_right == 1) 
		{
			create_set_speeds(robot, 0.0, 0.0);
			printf("bumper brake\n");
			return 1;
		}
	}
	
	// reassign the nextDirection to now be the current direction
	currDirection = nextDirection;
	return 0;
}



