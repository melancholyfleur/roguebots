/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  filter.c
 ********************* */

#include "filter.h"

/* firFilterCreate()
 * creates, allocates,  and iniitializes a new firFilter
 */
filter_t *firFilterCreate()
{
	int i;
  	filter_t *f = malloc(sizeof(filter_t));
  	for (i=0; i<TAPS; i++) {
    		f->samples[i] = 0;
    		f->coefficients[i] = 1.0/(float) TAPS; // user must set coef's
  	}
  	f->next_sample = 0;
	return f;
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
  		usleep(500);
	}
  	if(++(f->next_sample) == TAPS) f->next_sample = 0;
  	return(sum);
}



