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
	probability* next;
}probability;

probability *head;

void setProbabilities(int*, int);


