/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  map.c
 ********************* */

#include "map.h"


void setProbabilities(int* options, int currRoom){
	if(options[0] == 1 && options[1] == 1 && options[2] == 2){
		head[currRoom].fr = 0.3;
		head[currRoom].rt = 0.3;
		head[currRoom].lf = 0.3;
	}
	else if(options[0] == 1 && options[1] == 1){	
		head[currRoom].fr = 0.5;
		head[currRoom].rt = 0.5;
	}
	else if(options[0] == 1 && options[2] == 1){
		head[currRoom].fr = 0.5;
		head[currRoom].lf = 0.5;
	}
	else if(options[1] == 1 && options[2] == 1){
		head[currRoom].lf = 0.5;
		head[currRoom].rt = 0.5;
	}
	else if(options[0] == 1){
		head[currRoom].fr = 1.0;
	}
	else if(options[1] == 1){
		head[currRoom].rt = 1.0;
	}
	else if(options[2] == 1){
		head[currRoom].lf = 1.0;
	}
}


