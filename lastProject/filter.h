/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  filter.h
 ********************* */

#ifndef FILTER_H
#define FILTER_H

#include <unistd.h>
#include <stdlib.h>
#define TAPS  8			// how many filter taps

typedef struct 
{
	float    coefficients[TAPS];
  	unsigned next_sample;
  	float    samples[TAPS];
} filter_t;

filter_t *filter_sonarR;
filter_t *filter_sonarL;
filter_t *filter_irF;
filter_t *filter_irB;
filter_t *firFilterCreate();
float firFilter(filter_t*, float);

#endif

