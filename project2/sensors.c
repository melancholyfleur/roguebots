/* *********************
 *  Project 2
 *  Marylou Kunkle
 *  Lenny Kramer
 *  sensors.c
 ********************* */

#include "sensors.h"

int main(int argc, const char **argv)
{
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
	turret_init(r);	
	turret_SetServo(r,90);

	filter = firFilterCreate();

	/* robot is set up and ready -- select tests and run */
	
	waypoints[0].x = 0.0;				//origin
	waypoints[0].y = 0.0;
	waypoints[1].x = 7.3;
	waypoints[1].y = 0.0;
	waypoints[2].x = (7.62 + 7.3);
	waypoints[2].y = 7.62;
	waypoints[3].x = (18.89 + 7.62 + 7.3);
	waypoints[3].y = 7.62;
	waypoints[4].x = (18.89 + 7.62 + 7.3);
	waypoints[4].y = (7.62 - 10.668);
	waypoints[5].x = (18.89 + 7.62 + 7.3 + 3.9624);
	waypoints[5].y = (7.62 - 10.668 - 9.144);
	waypoints[6].x = (18.89 + 7.62 + 7.3 + 3.9624 - 22.86);
	waypoints[6].y = (7.62 - 10.668 - 9.144);
	waypoints[7].x = (18.89 + 7.62 + 7.3 + 3.9624 - 22.86);
	waypoints[7].y = (7.62 - 10.668 - 9.144 + 12.192);
	
	//robot motions
	Move(c, r, waypoints[1]);
	Move(c, r, waypoints[2]);
	Move(c, r, waypoints[3]);
	Move(c, r, waypoints[4]);
	Move(c, r, waypoints[5]);
	Move(c, r, waypoints[6]);
	Move(c, r, waypoints[7]);
	//
	
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
 * targetX:    x-coordinate destination
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

/* error_ir_left()
 */
float error_ir_LEFT()
{
	ir_l = r->ir[1];
	return firFilter(filter, ir_l);
}

/* error_ir_right()
 */
float error_ir_RIGHT()
{
	ir_r = r->ir[0];
	return firFilter(filter, ir_r);
}

/* error_sonar_FRONT()
 */
float error_sonar_FRONT(turret_comm_t *t)
{
	sonar_f = t->sonar[0];
	//return sonar_f;
	return firFilter(filter, sonar_f);
}

/* error_sonar_BACK()
 */
float error_sonar_BACK(turret_comm_t *t)
{
	sonar_b = t->sonar[1];
	//return sonar_b;
	return firFilter(filter, sonar_b);
}

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

/* Move() 
 * client:  client to connect to robot
 * distance: x-coordinate that robot should aim for;
 * angle: angle that robot should stay at;
 */
float Move(create_comm_t *client, turret_comm_t *t, waypoint point)
{
	printf("Enter Move\ngetting error_t\n");  
	create_get_sensors(client, TIMEOUT);
	error_dist = error_t(client, point);
	printf("just got error_t\n");
	float sensor_error;

	while(error_dist > 0.5)
	{
		create_get_sensors(client, TIMEOUT);
		printf("in while loop\n");
		error_dist = error_t(client, point);
		vx = PID(error_dist);
		
		error_a = error_ta(client, 0.0);
		va = PID_A(error_a);
		printf("getting sensor_error %d\n",sensor_error);
		turret_get_sonar(t);
		sensor_error = (error_sonar_FRONT(t) - error_sonar_BACK(t));
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
		usleep(500);
	}
	
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
    	f->coefficients[i] = 1. /(float) TAPS; // user must set coef's
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



