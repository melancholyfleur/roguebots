/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  map.h
 ********************* */

#ifndef MAP_H
#define MAP_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "TurretAPI.h"
#include "create_comms.h"

typedef struct{
	float fr;
	float lf;
	float rt;
	int myExit;		//0 = front, 1 = right, 2 = left
	struct probability* next;
}probability;

probability *head;

void setProbabilities(int*, int);
void decrementProbabilities();

#endif

