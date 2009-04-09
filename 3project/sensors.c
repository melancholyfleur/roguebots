/* *********************
 *  Project 2
 *  Marylou Kunkle
 *  Lenny Kramer
 *  sensors.c
 ********************* */

#include "sensors.h"

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

	filter = firFilterCreate();

	direction[0] = "W";
	direction[1] = "N";
	direction[2] = "E";
	direction[3] = "N";
	direction[4] = "E";
	direction[5] = "E";
	direction[6] = "N";
	direction[7] = "N";
	direction[8] = "E";
	direction[9] = "E";
	
	int i;
	for(i = 0; i < 10; i++)
	{
		Move(c, r, direction, i);
	}

	/*
	// read points from file
	FILE *file = fopen(argv[1], "r");
	printf("just read file\n");
	if(file != NULL)
	{
		char line[32];
		char num_lines;

		// read the first line in the file.
		// this gives the number of waypoints
		printf("fgetting line nums\n");
		fgets(num_lines, 1, file);
		num_waypoints = atoi(num_lines);
		printf("allocating waypoint size\n");
		// allocate the size of *waypoints
		waypoints = malloc(num_waypoints * sizeof(waypoint));
		printf("malloc'd\n");
		// store the (x,y) coordinates in *waypoints
		int i = 0;
		printf("fgetting lines\n");
		while(fgets(line,sizeof(line),file) != NULL)
		{
			printf("foo\n");
			file_tokens = strtok(line, ",");
			printf("bar\n");
			waypoints[i].x = atof(file_tokens[0]);
			printf("freep\n");
			waypoints[i].y = atof(file_tokens[1]);
			printf("froop\n");
			i++;
		}
		fclose(file);
		
		// Move to each waypoint
		int j;
		for(j = 0; j < num_waypoints; j++) 
		{
			Move(c, r, waypoints[j], j);
		}
	}
	else
	{
		perror(argv[1]);
	}*/
	
	// Shutdown and tidy up
	create_set_speeds(c,0.0,0.0);
	create_close(c);
	create_destroy(c);
	turret_close(r);
	turret_destroy(r);
	exit(0);
	
	return 0;
}

/* getDistance()
 * Uses the distance formula to find the distance between two waypoints
 */
float getDistance(waypoint point_one, waypoint point_two) 
{
	return sqrt(pow(point_two.x-point_one.x, 2) + pow(point_two.y-point_one.y, 2));
}


float getAngle(float distance)
{
	printf("error entering getAngle\n");
	// the U vector will be constant at these values
	uX = BUFFER_DIST;
	uY = 0.0;
	uZ = 0.0;
	
	vX = BUFFER_DIST;
	vY = distance;
	vZ = 0.0;

	dot_product = (uX * vX) + (uY * vY); // uZ * vZ will always be zero
	mag_u = sqrt(pow(uX,2)); 	     // other values will be zero
	mag_v = sqrt(pow(vX,2) + pow(vY,2));

	theta = acos(dot_product/(mag_u * mag_v));

	return theta;
}

/* error_ir()
 * return: value greater or less than zero
 */
float error_ir()
{
	float ir_error;
	turret_get_ir(r);
	ir_r = firFilter(filter, r->ir[0]);
	ir_l = firFilter(filter, r->ir[1]);
	ir_error = (ir_r - ir_l);
	return ir_error;
}

/* error_sonar()
 */
float error_sonar()
{
	turret_get_sonar(r);
	sonar_r = firFilter(filter, r->sonar[0]);
	sonar_l = firFilter(filter, r->sonar[1]);
	if(sonar_r > 650.0){
		sonar_error = (542.5 - sonar_l);
	}
	else if(sonar_l > 650.0){
		sonar_error = (sonar_r - 542.5);
	}	
	else{
		sonar_error = (sonar_r - sonar_l);
	}
	return sonar_error;
}

/* Move() 
 * client:  client to connect to robot
 * distance: x-coordinate that robot should aim for;
 * angle: angle that robot should stay at;
 */
void Move(create_comm_t *client, turret_comm_t *t, direction d[], int pos)
{
	char curpos = d[pos-1];
	printf("entering move\n");
	error_dist = error_t(client, d[pos]);
	error_a = error_ta(client, curr_angle);
	while(error_dist > BUFFER_DIST)
	{
		position = create_get_sensors(client, TIMEOUT);
		
		error_dist = error_t(client, point[pos]);
		printf("error_dist: %f\n", error_dist);
		
		vx = PID(error_dist);
		printf("vx: %f\n", vx);
		
		va = PID_A(error_a);
		printf("va: %f\n", va);
		
		create_set_speeds(client, vx, va);
		
		if(client->bumper_left == 1 || client->bumper_right == 1) 
		{
			create_set_speeds(client, 0.0, 0.0);
			break;
		}
		
		sonar_error = error_sonar();
		printf("sonar error: %f\n", sonar_error);
		
		if(sonar_error > 0.0)
		{
			va += (M_PI/16);
			create_set_speeds(client,vx,va);
		}
		else if(sonar_error < 0.0)
		{
			va -= (M_PI/16);
			create_set_speeds(client,vx,va);
		}
		
		//usleep(500);
	}
		//if(error_dist <= BUFFER_DIST)
		//{
	dist = getDistance(point[pos], point[pos+1]);
	printf("dist: %f\n", dist);

	new_angle = getAngle(dist);
	printf("new_angle: %f\n", new_angle);
			
	if( ((point[pos+1].x > point[pos].x) && (point[pos+1].y >= point[pos].y)) ||
	    ((point[pos+1].x > point[pos].x) && (point[pos+1].y <= point[pos].y)) ||
	    ((point[pos+1].x < point[pos].x) && (point[pos+1].y >= point[pos].y)) ||
	    ((point[pos+1].x < point[pos].x) && (point[pos+1].y <= point[pos].y)) ||
	    ((point[pos+1].y < point[pos].y) && (point[pos+1].x >= point[pos].x)) ||
	    ((point[pos+1].y < point[pos].y) && (point[pos+1].x <= point[pos].x)) ||
	    ((point[pos+1].y > point[pos].y) && (point[pos+1].x <= point[pos].x)) ||
	    ((point[pos+1].y > point[pos].y) && (point[pos+1].x >= point[pos].x)) );
	{
		new_angle = new_angle * -1;	
	}
		//}
	va += new_angle;
	curr_angle = va;

	create_set_speeds(client, vx, va);
			
		
	
	printf("Leave Move\n");
}

/* Turn()
 * client: client to connect to robot
 * deg:    amount of degrees robot should turn
 */
/*float Turn(playerc_client_t *client, float deg)
{
  printf("Enter Turn\n");
  float error_a = error_ta(position2d, deg);
  printf("error_a = %f\n", error_a); 
  while(fabs(error_a) > 0.1)
  {
    // Find margin of error between current and target angles
    error_a = error_ta(position2d, deg);
   
    // Set angle velocity based on error
    va = PID_A(error_a);
    printf("error_a = %f angle = %f\n", error_a, va);
    playerc_position2d_set_cmd_vel(position2d, 0.0, 0.0, va, 1.0);
    
    // Test collision with each bumper
    if(bumper->bumpers[0]!=0 || bumper->bumpers[1]!=0)
    {
      playerc_position2d_set_cmd_vel(position2d, 0.0, 0.0, 0.0, 0.0);
      break;
    }
    playerc_client_read(client);
    printf("Turning : x = %f y = %f a = %f\n", position2d->px, position2d->py, position2d->pa);   
  }
  printf("Leave Turn\n");
}*/

