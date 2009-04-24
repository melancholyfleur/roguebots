/* *********************
 *  Project 2
 *  Marylou Kunkle
 *  Lenny Kramer
 *  maze.c
 ********************* */

#include "maze.h"

int main(int argc, const char **argv)
{
	printf("entered program\n");
	/* allocate device objects */
	c = create_create("/dev/ttyS2");
	r = turret_create();

	/* open the create serial comm  */
	if(create_open(c,FULLCONTROL) < 0) {
		printf("create open failed\n");
	 	return(-1);
	}

	/* Open the i2c device */
	if(turret_open(r) < 0) {
		printf("failed to connect to robostix\n");
		return(-1);
	}

	/* init the robostix board interfaces */
	printf("initialize turret\n");
	turret_init(r);	
	// sonar
	if(!WHICH_SENSOR){
		turret_SetServo(r,90);
	}
	//ir
	else{
	  	turret_SetServo(r,0);
	}
	
	filter_sonarR = firFilterCreate();
	filter_sonarL = firFilterCreate();
	filter_irF = firFilterCreate();
	filter_irB = firFilterCreate();
	printf("created a filter\n");

	signal(SIGINT,signal_interrupt);
	int* whereAmI;
	int i = 0;
	int j = 0;
	head = (probability *)malloc(sizeof(probability));
	int atADeadEnd = 1;
	while(1){
		whereAmI = WhatDoISee(r);
		for(j = 0; j < sizeof(whereAmI); j++){
			if(whereAmI[j] == 1){
				atADeadEnd = 0;
			}
		}
		if(atADeadEnd){
			char* input = NULL;
			printf("Am I at the goal?\n");
			scanf("%s\n", input);
			if(input == "n"){
				printf("Dead end. Try again.\n");
				decrementProbabilities();
				i = 0;
			}
			else if(input == "y"){
				// adjust values positively
				printf("Goal has been found.");
				break;
			}
		}
		else		// bot has some options
		{
			printf("we have options\n");
			probability* currProb = setProbabilities(whereAmI, i);
			// front
			if(whereAmI[0] != 0)
			{
				randProb = fmod(rand(),(currProb[i].fr + 1.0));
				probArray[0] = randProb;
			}
			else
				probArray[0] = 0.0;
				  
			// right
			if(whereAmI[1] != 0)
			{
				randProb = fmod(rand(),(currProb[i].rt + 1.0));
				probArray[1] = randProb;
			}
			else
				probArray[1] = 0.0;
				
			// left
			if(whereAmI[2] != 0)
			{
				randProb = fmod(rand(),(currProb[i].lf + 1.0));
				probArray[2] = randProb;
			}
			else
			{
				probArray[2] = 0.0;
			}
			int dir = whereToTurn(probArray);
			MoveToNeighboringCell(c,r,dir);
		}
		i++;
	}
	
	// Shutdown and tidy up
	create_set_speeds(c,0.0,0.0);
	create_close(c);
	create_destroy(c);
	turret_close(r);
	turret_destroy(r);
	exit(0);
	
	return 0;
}

int whereToTurn(float a[]){
	float max;
	max = a[0];
	int i = 0;
	int direction = 0;
	for(i = 0; i < sizeof(a); i++){
		if(max>a[i]){
			max = a[i];
		}
		direction = i;
	}
	return direction;
}

void signal_interrupt(int arg)
{
	printf("Received interrupt. Exiting Programming.\n");
	create_set_speeds(c,0.0,0.0);
	create_close(c);
	create_destroy(c);
	turret_close(r);
	turret_destroy(r);
	exit(0);
}

void AdjustPosition(create_comm_t* rb, turret_comm_t* tr) {
	float sonarErrorTemp = error_sonar(tr);
	printf("in AdjustPosition\nsonar error: %f\n", sonarErrorTemp);
	if(sonarErrorTemp < 0.0)
	{
		va += (M_PI/16);
		printf("increasing va: %f\n",va);
		create_set_speeds(rb,vx,va);
	}
	else if(sonarErrorTemp > 0.0)
	{
		va -= (M_PI/16);
		printf("decreasing va: %f\n",va);
		create_set_speeds(rb,vx,va);	
	}	
}

/* Move() 
 * client:  client to connect to robot
 * distance: x-coordinate that robot should aim for;
 * angle: angle that robot should stay at;
 */
int MoveToNeighboringCell(create_comm_t* device, turret_comm_t* turret, int target)
{
	printf("in movetoneighbor function\n");
	
	position = create_get_sensors(device, TIMEOUT);
	currPos.x = device->ox;
	currPos.y = device->oy;
	if(Turn(device,target)){
		return 1;
	}
	if(target == 0 || target == 1){
		distToMove = currPos.y + distBtwnCells;
	}
	else if(target == 3){
		distToMove = currPos.x + distBtwnCells;
	}
	else if(target == 2){
		if(currPos.x < 0.0){distToMove = distBtwnCells;}
		distToMove = distBtwnCells + currPos.x;
	}
	printf("distToMove: %f\n",distToMove);
	dist_error = error_tx(device, distToMove, target);
	printf("dist_error: %f\n",dist_error);
	//
	
	while(dist_error > BUFFER_DIST)
	{
		printf("in move while loop\n");
		position = create_get_sensors(device, TIMEOUT);
		
		dist_error = error_tx(device, distToMove, target);
		printf("dist_error: %f\n", dist_error);
		
		vx = PID(dist_error);
		if(vx > 1.0) {vx = 0.5;}
		printf("vx: %f\n", vx);
		
		va = PID_A(angle_error);
		printf("va: %f\n", va);
		

		create_set_speeds(device, vx, va);
		
		//AdjustPosition(device, turret);
		
		if(device->bumper_left == 1 || device->bumper_right == 1) 
		{
			create_set_speeds(device, 0.0, 0.0);
			printf("bumper brake\n");
			return 1;
		}
	}
	
	return 0;	
}

int Turn(create_comm_t* robot, int target) {
	printf("in turn function\n");	

	//get delta
	if(target == 0){
		target_angle = 0;
	}
	else if(target == 1){
		target_angle = -M_PI/2;
	}
	else if(target == 2){
		target_angle = M_PI/2;
	}

	// rotate bot by delta
	angle_error = error_ta(robot, target_angle);
	while (fabs(angle_error) > 0.3)
	{
		position = create_get_sensors(robot, TIMEOUT);
		printf("in turn while loop\n");
		angle_error = error_ta(robot, target_angle);
		printf("angle_error: %f\n",angle_error);
		va = PID_A(angle_error);
		printf("va: %f\n",va);
		create_set_speeds(robot, 0.0, va);
		if(robot->bumper_left == 1 || robot->bumper_right == 1) 
		{
			create_set_speeds(robot, 0.0, 0.0);
			printf("bumper brake\n");
			return 1;
		}
	}
	return 0;
}

/* WhatDoISee()
 */
int* WhatDoISee(turret_comm_t* turr){
	turret_get_sonar(turr);
	float right = firFilter(filter_sonarR, turr->sonar[0]);
	float left = firFilter(filter_sonarL, turr->sonar[1]);
	turret_get_ir(turr);
	float front = firFilter(filter_irF, turr->ir[1]);
	
	openDirs[0] = 1;	//front
	openDirs[1] = 1;	//right
	openDirs[2] = 1;	//left

	if(front < 45.0 && front > 0.0){
		printf("sees front\n");
		openDirs[0] = 0;
	}
	if(right < 45.0 && right > 0.0){
		printf("sees right\n");
		openDirs[1] = 0;
	}
	if(left < 45.0 && left > 0.0){
		printf("sees left\n");
		openDirs[2] = 0;
	}

	return openDirs;

}


