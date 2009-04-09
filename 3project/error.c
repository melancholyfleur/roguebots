/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  error.c
 ********************* */

#include "error.h"
#include <math.h>


/* error_tx()
 * position2d: current px,py,pa positions for robot
 * targetx:    x-coordinate destination
 */
float error_tx(create_comm_t *position2d, int targetPos)
{
	return targetPos - position2d->ox;
}

/* error_ta()
 * position2d:  current px,py,pa positions for robot
 * targetAngle: angle to turn to
 */
float error_ta(create_comm_t *position2d, float targetAngle)
{
  	return (targetAngle - position2d->oa);
}



