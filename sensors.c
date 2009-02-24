/* *********************
 *  Project 2
 *  Mary-Lou Kunkle
 *  Lenny Kramer
 *  
 ********************* */

#include <libplayerc/playerc.h>
#include <sensors.h>

#define TAPS  4 // how many filter taps

/* GLOBAL VARS */
playerc_client_t *client;         	// client used to connect to robot
playerc_position2d_t *position2d;	// store the px,py,pa values for robot
playerc_bumper_t *bumper;         	// bumper client for robot
playerc_sonar_t *turret_sonar;		// sonar client
playerc_ir_t *turret_ir;			// ir client

typedef struct 
{
	float    coefficients[TAPS];
  	unsigned next_sample;
  	float    samples[TAPS];
} filter_t;


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

 	playerc_client_read(client);
  	printf("ir = %f %f sonar = %f %f\n",turret_ir->ranges.ranges[0], turret_ir->ranges.ranges[1],
         turret_sonar->scan[0], turret_sonar->scan[1]);

	// Enable the robots motors
	playerc_position2d_enable(position2d, 1);
	playerc_client_read(client);
	
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
     i tracks the next coeficeint
     j tracks the samples w/wrap-around */
  	for( i=0,j=f->next_sample; i<TAPS; i++) {
    	sum += f->coefficients[i]*f->samples[j++];
    	if(j==TAPS)  j=0;
  	}
  	if(++(f->next_sample) == TAPS) f->next_sample = 0;
  	return(sum);
}
