/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  error.c
 ********************* */

#include "error.h"

float sonar_error  = 0.0;
float ir_error     = 0.0;
float ir_r         = 0.0;
float ir_l         = 0.0;
float sonar_r      = 0.0;
float sonar_l      = 0.0;

/* error_ir()
 * * return: value greater or less than zero
 * */
float error_ir(turret_comm_t *r)
{
	turret_get_ir(r);
	ir_r = firFilter(filter, r->ir[0]);
 	ir_l = firFilter(filter, r->ir[1]);
        ir_error = (ir_r - ir_l);
	return ir_error;
}
	   
/* error_sonar()
 * */
float error_sonar(turret_comm_t *s)
{
	turret_get_sonar(s);
	sonar_r = firFilter(filter, s->sonar[0]);
	sonar_l = firFilter(filter, s->sonar[1]);
	if(sonar_r > 70.0){
		sonar_error = (35.0 - sonar_l);
	}
	else if(sonar_l > 70.0){
		sonar_error = (35.0 - sonar_r);
	}  
	else{
		sonar_error = (sonar_r - sonar_l);
	}
	return sonar_error;
}

/* error_tx()
 * position2d: current px,py,pa positions for robot
 * targetx:    x-coordinate destination
 */
float error_tx(create_comm_t *position2d, float targetPos)
{
	return (targetPos - fabs(position2d->ox));
}

/* error_ta()
 * position2d:  current px,py,pa positions for robot
 * targetAngle: angle to turn to
 */
float error_ta(create_comm_t *position2d, float targetAngle)
{
	printf("position2d->oa: %f\n",position2d->oa);
	return (targetAngle - fabs(position2d->oa));
}


