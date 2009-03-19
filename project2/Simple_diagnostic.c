#include <stdio.h>
#include "TurretAPI.h"
#include "create_comms.h"

#define FULLCONTROL 1
#define TEST_SONAR 1
#define TEST_IR 2
#define TEST_TURRET 3
#define TEST_MOVE 4

#define INCREMENT .1
int main(int argc,char **argv) {
  turret_comm_t *r;
  create_comm_t *c;
  int i2c_fd;
  int command;
  int i,angle;
  double vx,va;
  char ch;

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
  turret_init(r);
  
  /* robot is set up and ready -- select tests and run */

    for(command = show_menu(); command != 0; command = show_menu() )
      switch(command) {
      case TEST_SONAR:
	printf("testing Sonars, next 500 readings\n");
	for(i=0;i<5;i++) {
	  if((!turret_get_sonar(r))) 
	    printf("sensor read timeout\n");
	  else
	    printf("%d\t%d\n",r->sonar[0],r->sonar[1]);
	}
	break;
	  
      case TEST_IR:
	printf("testing IR, next 5 readings\n");
	for(i=0;i<5;i++) {
	  if(!turret_get_ir(r))
	    printf("sensor read timeout\n");
	  else
	    printf("%d\t%d\n",r->ir[0],r->ir[1]);
	}
	break;
	
      case TEST_TURRET:
	printf("testing Servo\nEnter angle: ");
	for(angle == 0;(angle <180) | (angle >180); scanf("%d",&angle)) {
	  printf("setting servo to %d degrees\n",angle);
	  turret_SetServo(r,angle);
	}
	break;

      case TEST_MOVE:
	printf("i- speed up\nn-slow down\nk- angle right\nj - angle left\n, x- stop\n");
	vx=0.;va=0.;
	for( ch=getchar(); ch != 'x'; ch=getchar()) {
	  switch(ch) {
	  case 'i': vx += INCREMENT; break;
	  case 'n': vx -= INCREMENT; break;
	  case 'j': va += INCREMENT; break;
	  case 'k': va -= INCREMENT; break;
	  case 'x': va = 0.; vx = 0.; break;
	  default: break;
	  }
	  if(vx > 1.) vx = 1.;
	  if(va > 1.) va = 1.;
	  create_set_speeds(c,vx,va);
	}
	break; 

      default:
	command = -1;
	break;
      }
    
}

int show_menu()
{
  int cmd;

  printf("\nSelect test\n\n");
  printf("\n1  - Test Sonar\n");
  printf("\n2  - Test IR\n");
  printf("\n3  - Test Turret Motor\n");
  printf("\n4  - Test Move\n");
  
  printf("\n-1 - Exit\n");

  scanf("%d",&cmd);
  return(cmd);
}
