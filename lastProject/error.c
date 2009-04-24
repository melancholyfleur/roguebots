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
	ir_r = firFilter(filter_irB, r->ir[0]);
 	ir_l = firFilter(filter_irF, r->ir[1]);
        ir_error = (ir_r - ir_l);
	return ir_error;
}
	   
/* error_sonar()
 * */
float error_sonar(turret_comm_t *s)
{
	turret_get_sonar(s);
	printf("in error_sonar\n");
	sonar_r = firFilter(filter_sonarR, s->sonar[0]);
	printf("have sonar_r: %f\n",sonar_r);
	sonar_l = firFilter(filter_sonarL, s->sonar[1]);
	printf("have sonar_l: %f\n",sonar_l);
	if(sonar_r > 70.0){
		sonar_error = (sonar_l - 35.0);
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
float error_tx(create_comm_t *position2d, float targetPos, int direction)
{
	if(direction == 0){
		printf("position2d->ox: %f\n",position2d->ox);
		printf("position2d->oy: %f\n",position2d->oy);
		return (targetPos - fabs(position2d->oy));	
	}
	else if(direction == 1){
		printf("position2d->ox: %f\n",position2d->ox);
		printf("position2d->oy: %f\n",position2d->oy);
		return(targetPos - position2d->oy);
	}
	else if(direction == 2){
		printf("position2d->ox: %f\n",position2d->ox);
		printf("position2d->oy: %f\n",position2d->oy);
		return (targetPos - position2d->ox);
	}
	else{
		printf("position2d->ox: %f\n",position2d->ox);
		printf("position2d->oy: %f\n",position2d->oy);
		return (targetPos - fabs(position2d->ox));
	}
}

/* error_ta()
 * position2d:  current px,py,pa positions for robot
 * targetAngle: angle to turn to
 */
float error_ta(create_comm_t *position2d, float targetAngle)
{
	//printf("position2d->oa: %f\n",position2d->oa);
	return (targetAngle - position2d->oa);
}

/* WhatDoISee()
 */
int* WhatDoISee(turret_comm_t* turr){
	turret_get_sonar(turr);
	float right = firFilter(filter_sonarR, turr->sonar[0]);
	float left = firFilter(filter_sonarL, turr->sonar[1]);
	turret_get_ir(turr);
	float front = firFilter(filter_irF, turr->ir[1]);
	
	openDirs[0] = 0;	//front
	openDirs[1] = 1;	//right
	openDirs[2] = 2;	//left

	if(front < 45.0 && front > 0.0){
		printf("sees front\n");
		openDirs[0] = 1;
	}
	if(right < 45.0 && right > 0.0){
		printf("sees right\n");
		openDirs[1] = 1;
	}
	if(left < 45.0 && left > 0.0){
		printf("sees left\n");
		openDirs[2] = 1;
	}

	return openDirs;

}



