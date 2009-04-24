/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  map.c
 ********************* */

#include "map.h"

void decrementProbabilities(){
	int i = 0;
	while(head[i].next != NULL){
		if(head[i].myExit == 0){
			if(head[i].rt > 0.0 && head[i].lf > 0.0){
				head[i].fr -= 0.1;
				head[i].rt += 0.05;
				head[i].lf += 0.05;
			}
			else if(head[i].rt > 0.0){
				head[i].fr -= 0.1;
				head[i].rt += 0.1;
			}
			else if(head[i].lf > 0.0){
				head[i].fr -= 0.1;
				head[i].lf += 0.1;	
			}
			else{
				head[i].fr -= 0.1;
			}
		}
		if(head[i].myExit == 1){	
			if(head[i].fr > 0.0 && head[i].lf > 0.0){
				head[i].rt -= 0.1;
				head[i].fr += 0.05;
				head[i].lf += 0.05;
			}
			else if(head[i].fr > 0.0){
				head[i].rt -= 0.1;
				head[i].fr += 0.1;
			}
			else if(head[i].lf > 0.0){
				head[i].rt -= 0.1;
				head[i].lf += 0.1;	
			}
			else{
				head[i].rt -= 0.1;
			}
		}
		if(head[i].myExit == 2){	
			if(head[i].fr > 0.0 && head[i].rt > 0.0){
				head[i].lf -= 0.1;
				head[i].fr += 0.05;
				head[i].rt += 0.05;
			}
			else if(head[i].fr > 0.0){
				head[i].lf -= 0.1;
				head[i].fr += 0.1;
			}
			else if(head[i].rt > 0.0){
				head[i].lf -= 0.1;
				head[i].rt += 0.1;	
			}
			else{
				head[i].lf -= 0.1;
			}
		}
		head++;
		i++;
	}
}

probability* setProbabilities(int* options, int currRoom){
	//need to implement: if this is a new room?
	if(options[0] == 1 && options[1] == 1 && options[2] == 1){
		head[currRoom].fr = 0.33;
		head[currRoom].rt = 0.33;
		head[currRoom].lf = 0.33;
	}
	else if(options[0] == 1 && options[1] == 1){	
		head[currRoom].fr = 0.5;
		head[currRoom].rt = 0.5;
		head[currRoom].lf = 0.0;
	}
	else if(options[0] == 1 && options[2] == 1){
		head[currRoom].fr = 0.5;
		head[currRoom].rt = 0.0;
		head[currRoom].lf = 0.5;
	}
	else if(options[1] == 1 && options[2] == 1){
		head[currRoom].fr = 0.0;
		head[currRoom].rt = 0.5;
		head[currRoom].lf = 0.5;
	}
	else if(options[0] == 1){
		head[currRoom].fr = 1.0;
		head[currRoom].rt = 0.0;
		head[currRoom].lf = 0.0;
	}
	else if(options[1] == 1){
		head[currRoom].fr = 0.0;
		head[currRoom].rt = 1.0;
		head[currRoom].lf = 0.0;
	}
	else if(options[2] == 1){
		head[currRoom].fr = 0.0;
		head[currRoom].rt = 0.0;
		head[currRoom].lf = 1.0;
	}
	return head;
}


