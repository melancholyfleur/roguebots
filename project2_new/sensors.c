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
	// sonar
	if(!WHICH_SENSOR){
		turret_SetServo(r,90);
	}
	//ir
	else{
	  	turret_SetServo(r,0);
	}
	printf("creating filter\n");
	filter = firFilterCreate();
	printf("created filter\n");

	waypoints[0].x = 7.3;
	waypoints[0].y = 0.0;
	waypoints[1].x = 7.3;
	waypoints[1].y = 7.62;
	waypoints[2].x = 26.19;
	waypoints[2].y = 7.62;
	waypoints[3].x = 26.19;
	waypoints[3].y = -3.04;
	waypoints[4].x = 30.15;
	waypoints[4].y = -3.04;
	waypoints[5].x = 30.15;
	waypoints[5].y = -12.18;
	waypoints[6].x = 7.3;
	waypoints[6].y = -12.18;
	waypoints[7].x = 7.3;
	waypoints[7].y = 0.0;
	waypoints[8].x = 0.0;
	waypoints[8].y = 0.0;

	int i;
	for(i = 0; i < 9; i++)
	{
		Move(c, r, waypoints, i);
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

/* error_tx()
 * position2d: current px,py,pa positions for robot
 * targetx:    x-coordinate destination
 */
float error_t(create_comm_t *position2d, waypoint point)
{
	return sqrt(pow((point.x - position2d->ox), 2.0) + pow((point.y - position2d->oy), 2.0));
}

/* error_ta()
 * position2d:  current px,py,pa positions for robot
 * targetAngle: angle to turn to
 */
float error_ta(create_comm_t *position2d, float targetAngle)
{
  	return (targetAngle - position2d->oa);
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
	sonar_f = firFilter(filter, r->sonar[0]);
	sonar_b = firFilter(filter, r->sonar[1]);
	sonar_error = (sonar_f - sonar_b);
	return sonar_error;
}

/* error_sonar_BACK()

float error_sonar_BACK(turret_comm_t *s)
{
	sonar_b = s->sonar[1];
	//return sonar_b;
	return firFilter(filter, sonar_b);
}
*/
/* PID()
 * pid_error: current error for destination
 */
float PID(float pid_error)
{
  float integral = integral + pid_error;
  float derivative = (pid_error - prev_error);
  pid = PID_KP*pid_error + PID_KI*integral + PID_KD*derivative;
  prev_error = pid_error;
  return pid;
}

/* PID_A()
 * pid_error_a: the current angle's error
 */
float PID_A(float pid_error_a)
{
  float integral = integral + pid_error_a;
  float derivative = (pid_error_a - prev_error_a);
  pid = PIDA_KP*pid_error_a + PIDA_KI*integral + PIDA_KD*derivative;
  prev_error_a = pid_error_a;
  return pid;
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


/* Move() 
 * client:  client to connect to robot
 * distance: x-coordinate that robot should aim for;
 * angle: angle that robot should stay at;
 */
float Move(create_comm_t *client, turret_comm_t *t, waypoint point[], int pos)
{
	printf("entering move\n");
	// get ir
	error_dist = error_t(client, point);
	error_a = error_ta(client, 0.0);
	while(error_dist > 0)
	{
		printf("error entering move while\n");
		position = create_get_sensors(client, TIMEOUT);
		printf("getting err_dist\n");
		error_dist = error_t(client, point);
		printf("should have error_dist\n");
		vx = PID(error_dist);
		printf("should have vx\n");
		va = PID_A(error_a);
		printf("should have vz\n");
		
		dist = getDistance(point, point[pos+1]);
		printf("should have dist\n");
		new_angle = getAngle(dist);
		printf("should have new_angle\n");
	
		if(error_dist <= BUFFER_DIST)
		{
			if( (point[pos+1].x > point[pos].x) && (point[pos+1].y >= point[pos].y) ||
			    (point[pos+1].x > point[pos].x) && (point[pos+1].y <= point[pos].y) ||
			    (point[pos+1].x < point[pos].x) && (point[pos+1].y >= point[pos].y) ||
			    (point[pos+1].x < point[pos].x) && (point[pos+1].y <= point[pos].y) ||
			    (point[pos+1].y < point[pos].y) && (point[pos+1].x >= point[pos].x) ||
			    (point[pos+1].y < point[pos].y) && (point[pos+1].x <= point[pos].x) ||
			    (point[pos+1].y > point[pos].y) && (point[pos+1].x <= point[pos].x) ||
			    (point[pos+1].y > point[pos].y) && (point[pos+1].x >= point[pos].x) );
			{
				new_angle = new_angle * -1;	
			}
		}
		va += new_angle;

		create_set_speeds(client, vx, va);
			
		if(sonar_error >= 3.0)
		{
			va -= (M_PI/16);
			create_set_speeds(client,vx,va);
		}
		else if(sonar_error < 3.0)
		{
			va += (M_PI/16);
			create_set_speeds(client,vx,va);
		}
		
		if(client->bumper_left == 1 || client->bumper_right == 1) 
		{
			create_set_speeds(client, 0.0, 0.0);
			break;
		}
		usleep(500);
	}
	/*
	printf("Enter Move\ngetting error_t\n");  
	create_get_sensors(client, TIMEOUT);
	error_dist = error_t(client, point);
	printf("just got error_t\n");

	while(1)
	{
		create_get_sensors(client, TIMEOUT);
		printf("in while loop\n");
		error_dist = error_t(client, point);
		vx = PID(error_dist);
		
		error_a = error_ta(client, 0.0);
		va = PID_A(error_a);
		float sensor_error;
		if(WHICH_SENSOR){
			sensor_error = error_ir(t);
		}
		else{
			sensor_error = error_sonar(t);
		}
		printf("getting sensor_error %d\n",sensor_error);
		if(sensor_error >= 3.0){	//too far to the left
			printf("decrementing va\n");
			va--;
		}
		if(sensor_error <= -3.0){	//too far to the right
			printf("incrementing va\n");
			va++;
		}
		printf("setting speeds\n");
		create_set_speeds(client, vx, va);
		printf("test bumpers\n");
		if(client->bumper_left || client->bumper_right)
		{
			create_set_speeds(client, 0.0, 0.0);
			break;
		}
		printf("Moving : x = %f y = %f a = %f\n", client->ox, client->oy, client->oa);   
		printf("ir = %d %d sonar = %d %d\n",r->ir[0],r->ir[1],r->sonar[0],r->sonar[1]);
		usleep(10000);
	}
	*/
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

/* firFilterCreate()
 * creates, allocates,  and iniitializes a new firFilter
 */
filter_t *firFilterCreate()
{
	int i;
  	filter_t *f = malloc(sizeof(filter));
  	for (i=0; i<TAPS; i++) {
    	f->samples[i] = 0;
    	f->coefficients[i] = 1.0/(float) TAPS; // user must set coef's
  	}
  	f->next_sample = 0;
}

/* firFilter 
 * inputs take a filter (f) and the next sample (val)
 * returns the next filtered sample
 * incorporates new sample into filter data array
 */

float firFilter(filter_t *f, float val)
{
 	float sum =0;
  	int i,j;

  	/* assign  new value to "next" slot */
  	f->samples[f->next_sample] = val;

  	/* calculate a  weighted sum
     i tracks the next coefficient
     j tracks the samples w/wrap-around */
  	for( i=0,j=f->next_sample; i<TAPS; i++) {
    	sum += f->coefficients[i]*f->samples[j++];
    	if(j==TAPS)  j=0;
  	}
  	if(++(f->next_sample) == TAPS) f->next_sample = 0;
  	return(sum);
}



