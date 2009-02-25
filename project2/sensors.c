/* *********************
 *  Project 2
 *  Mary-Lou Kunkle
 *  Lenny Kramer
 *  
 ********************* */

#include "sensors.h"

int main(int argc, const char **argv)
{
	// Create a client object and connect to the server; the server must
  	// be running on "localhost" at port 6665
  	client = playerc_client_create(NULL, "gort", 9876);
  	if (playerc_client_connect(client) != 0)
  	{
    	fprintf(stderr, "error: %s\n", playerc_error_str());
    	return -1;
  	}

  	// Create a bumper proxy (device id "bumper:0" and subscribe
   	// in read mode
  	bumper = playerc_bumper_create(client, 0);
  	if(playerc_bumper_subscribe(bumper,PLAYERC_OPEN_MODE)!= 0)
  	{
    	fprintf(stderr, "error: %s\n", playerc_error_str());
    	return -1;
  	}

  	// Create a position2d proxy (device id "position2d:0") and susbscribe
  	// in read/write mode
  	position2d = playerc_position2d_create(client, 0);
  	if (playerc_position2d_subscribe(position2d, PLAYERC_OPEN_MODE) != 0)
  	{
    	fprintf(stderr, "error: %s\n", playerc_error_str());
    	return -1;
  	}

	// create proxy for sonars                                                                                      
	turret_sonar = playerc_sonar_create(client, 0);
	if (playerc_sonar_subscribe(turret_sonar, PLAYERC_OPEN_MODE) != 0)
	{
		fprintf(stderr, "sonar subscribe error: %s\n", playerc_error_str());
    	return -1;
  	}

  	// create proxy for ir sensors                                                                                  
  	turret_ir = playerc_ir_create(client, 1);
  	if (playerc_ir_subscribe(turret_ir, PLAYERC_OPEN_MODE) != 0)
  	{
    	fprintf(stderr, "ir subscribe error: %s\n", playerc_error_str());
    	return -1;
  	}

 	

	// Enable the robots motors
	playerc_position2d_enable(position2d, 1);
	playerc_client_read(client);
	
	//robot motions
	Move(client, MOVE1, ANGLE1);
	Turn(client, TURN1);
	Move(client, MOVE2, ANGLE2);
	Turn(client, TURN2);
	Move(client, MOVE3, ANGLE3);
	//
	
	// Shutdown and tidy up
	playerc_sonar_unsubscribe(turret_sonar);
	playerc_sonar_destroy(turret_sonar);
	playerc_ir_unsubscribe(turret_ir);
	playerc_ir_destroy(turret_ir);
	playerc_position2d_unsubscribe(position2d);
	playerc_position2d_destroy(position2d);
	playerc_client_disconnect(client);
	playerc_client_destroy(client);
	
	return 0;
}

/* error_tx()
 * position2d: current px,py,pa positions for robot
 * targetX:    x-coordinate destination
 */
float error_tx(playerc_position2d_t *position2d, float targetX)
{
  if(targetX < 0.0)
    return position2d->px - targetX;

  return targetX - position2d->px; 
}

/* error_ta()
 * position2d:  current px,py,pa positions for robot
 * targetAngle: angle to turn to
 */
float error_ta(playerc_position2d_t *position2d, float targetAngle)
{
  return targetAngle - position2d->pa;
}

/* error_ir_left()
 */
float error_ir_LEFT()
{
	ir_l = turret_ir->ranges.ranges[0];
	return firFilter(filter, ir_l);
}

/* error_ir_right()
 */
float error_ir_RIGHT()
{
	ir_r = turret_ir->ranges.ranges[1];
	return firFilter(filter, ir_r);
}

/* error_sonar()
 */
float error_sonar()
{
	
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
float Move(playerc_client_t *client, float distance, float angle)
{
	printf("Enter Move\n");

	error_x = error_tx(position2d, distance);   

	while(error_x > 0.1)
	{
		error_x = error_tx(position2d, distance);
		vx = PID(error_x);

		error_a = error_ta(position2d, angle);
		va = PID_A(error_a);

		playerc_position2d_set_cmd_vel(position2d, vx, 0, va, 1.0);

		if(bumper->bumpers[0]!=0 || bumper->bumpers[1]!=0)
		{
			playerc_position2d_set_cmd_vel(position2d, 0.0, 0.0, 0.0, 0.0);
			break;
		}
		playerc_client_read(client);
		printf("Moving : x = %f y = %f a = %f\n", position2d->px, position2d->py, position2d->pa);   
		printf("ir = %f %f sonar = %f %f\n",turret_ir->ranges.ranges[0], turret_ir->ranges.ranges[1],
			 turret_sonar->scan[0], turret_sonar->scan[1]);
	}
	printf("Leave Move\n");
}

/* Turn()
 * client: client to connect to robot
 * deg:    amount of degrees robot should turn
 */
float Turn(playerc_client_t *client, float deg)
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
}

/* firFilterCreate()
 * creates, allocates,  and iniitializes a new firFilter
 */
filter_t *firFilterCreate()
{
	int i;
  	filter *f = malloc(sizeof(filter));
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



