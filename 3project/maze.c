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
	
	printf("entering move-for-loop\n");
	int i;
	for(i = 0; i < sizeof(directions); i++)
	{
		direction = directions[i];
		MoveToNeighboringCell(c, r, direction);
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

void AdjustPosition(create_comm_t* rb, turret_comm_t* tr) {
	sonar_error = error_sonar(tr);
	printf("in AdjustPosition\nsonar error: %f\n", sonar_error);
	if(sonar_error > 0.0)
	{
		va += (M_PI/16);
		create_set_speeds(rb,vx,va);
	}
	else if(sonar_error < 0.0)
	{
		va -= (M_PI/16);
		create_set_speeds(rb,vx,va);	
	}	
}

/* Move() 
 * client:  client to connect to robot
 * distance: x-coordinate that robot should aim for;
 * angle: angle that robot should stay at;
 */
void MoveToNeighboringCell(create_comm_t* device, turret_comm_t* turret, int target)
{
	printf("in movetoneighbor function\n");
	nextDirection = target;
	Turn(device);
	position = create_get_sensors(device, TIMEOUT);
	dist_error = error_tx(device, distToMove);
	while(dist_error > BUFFER_DIST)
	{
		printf("in move while loop\n");
		position = create_get_sensors(device, TIMEOUT);
		
		dist_error = error_tx(device, distToMove);
		printf("dist_error: %f\n", dist_error);
		
		vx = PID(dist_error);
		printf("vx: %f\n", vx);
		
		va = PID_A(angle_error);
		printf("va: %f\n", va);
		
		create_set_speeds(device, vx, va);
		
		if(device->bumper_left == 1 || device->bumper_right == 1) 
		{
			create_set_speeds(device, 0.0, 0.0);
			break;
		}

		AdjustPosition(device, turret);
	}
}

void Turn(create_comm_t* robot) {
	printf("in turn function\n");	
	// get delta angle
	if( (currDirection == NORTH && nextDirection == WEST) ||
	    (currDirection == SOUTH && nextDirection == EAST) ||
	    (currDirection == EAST && nextDirection == NORTH) ||
		(currDirection == WEST && nextDirection == SOUTH) )
	{
		delta = M_PI/2;
		printf("turning pi/2\n");
	}
		
	else if( (currDirection == NORTH && nextDirection == SOUTH) ||
	         (currDirection == SOUTH && nextDirection == NORTH) ||
	         (currDirection == EAST && nextDirection == WEST)   ||
			 (currDirection == WEST && nextDirection == EAST)   )
	{
		delta = M_PI;
		printf("turning pi\n");
	}
		
	else if( (currDirection == NORTH && nextDirection == EAST) ||
             (currDirection == SOUTH && nextDirection == WEST)  ||
	         (currDirection == EAST && nextDirection == SOUTH)  ||
	         (currDirection == WEST && nextDirection == NORTH)  )
	{
		delta = -M_PI/2;
		printf("turning -pi/2\n");
	}
	else
	{
		delta = 0.0;
		printf("no need to turn\n");
	}


	// rotate bot by delta
	angle_error = error_ta(robot, delta);
	while (fabs(angle_error) > 0.1)
	{
		position = create_get_sensors(robot, TIMEOUT);
		printf("in turn while loop\n");
		angle_error = error_ta(robot, delta);
		printf("angle_error: %f\n",angle_error);
		va = PID_A(angle_error);
		//va = angle_error;
		//printf("va: %f\n",va);
		create_set_speeds(robot, 0.0, va);
	}
	
	// reassign the nextDirection to now be the current direction
	currDirection = nextDirection;
}


/* error_ir()
 * * return: value greater or less than zero
 * */
float error_ir(turret_comm_t *r)
{
	turret_get_ir(r);
	ir_r = firFilter(filter, r->ir[0]);
 	ir_l = firFilter(filter, r->ir[1]);
        ir_error = (ir_r - ir_l);
	return ir_error;
}
	   
/* error_sonar()
 * */
float error_sonar(turret_comm_t *s)
{
	turret_get_sonar(s);
	sonar_r = firFilter(filter, s->sonar[0]);
	sonar_l = firFilter(filter, s->sonar[1]);
	if(sonar_r > 70.0){
		sonar_error = (35.0 - sonar_l);
	}
	else if(sonar_l > 70.0){
		sonar_error = (35.0 - sonar_r);
	}  
	else{
		sonar_error = (sonar_r - sonar_l);
	}
	return sonar_error;
}

/* error_tx()
 * position2d: current px,py,pa positions for robot
 * targetx:    x-coordinate destination
 */
float error_tx(create_comm_t *position2d, float targetPos)
{
	return (targetPos - position2d->ox);
}

/* error_ta()
 * position2d:  current px,py,pa positions for robot
 * targetAngle: angle to turn to
 */
float error_ta(create_comm_t *position2d, float targetAngle)
{
  	return (targetAngle - position2d->oa);
}

