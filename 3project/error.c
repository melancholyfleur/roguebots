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


