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

void AdjustPosition() {
	
}

/* Move() 
 * client:  client to connect to robot
 * distance: x-coordinate that robot should aim for;
 * angle: angle that robot should stay at;
 */
void MoveToNeighboringCell(create_comm_t *client, turret_comm_t *t, int target)
{
	nextDirection = target;
	Turn(client);
	while(dist_error > BUFFER_DIST)
	{
		position = create_get_sensors(client, TIMEOUT);
		
		dist_error = error_tx(client, target);
		printf("dist_error: %f\n", dist_error);
		
		vx = PID(sonar_error);
		printf("vx: %f\n", vx);
		
		va = PID_A(angle_error);
		printf("va: %f\n", va);
		
		create_set_speeds(client, vx, va);
		
		if(client->bumper_left == 1 || client->bumper_right == 1) 
		{
			create_set_speeds(client, 0.0, 0.0);
			break;
		}
		
		sonar_error = error_sonar(t);
		printf("sonar error: %f\n", sonar_error);
		
		if(sonar_error > 0.0)
		{
		
		}
		else if(sonar_error < 0.0)
		{
		
		}
	}
	create_set_speeds(client, vx, va);
}

void Turn(create_comm_t *client) {
	
	// get delta angle
	if( (currDirection == NORTH && nextDirection == WEST) ||
	    (currDirection == SOUTH && nextDirection == EAST) ||
	    (currDirection == EAST && nextDirection == NORTH) ||
		(currDirection == WEST && nextDirection == SOUTH) )
	{
		delta = M_PI/2;
	}
		
	else if( (currDirection == NORTH && nextDirection == SOUTH) ||
	         (currDirection == SOUTH && nextDirection == NORTH) ||
	         (currDirection == EAST && nextDirection == WEST)   ||
			 (currDirection == WEST && nextDirection == EAST)   )
	{
		delta = M_PI;
	}
		
	else if( (currDirection == NORTH && nextDirection == EAST) ||
             (currDirection == SOUTH && nextDirection == WEST)  ||
	         (currDirection == EAST && nextDirection == SOUTH)  ||
	         (currDirection == WEST && nextDirection == NORTH)  )
	{
		delta = -M_PI/2;
	}
	
	// rotate bot by delta
	angle_error = error_ta(client, delta);
	while (fabs(angle_error) > 0.1)
	{
		angle_error = error_ta(client, delta);
		va = PID_A(angle_error);
		create_set_speeds(client, 0, va);
	}
	
	// reassign the nextDirection to now be the current direction
	currDirection = nextDirection;
}
