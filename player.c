/* **************
   Project 1
   Marylou Kunkle
   Lenny Kramer
   1/28/09
**************** */

#include "player.h"

/* error_tx()
 * position2d: current px,py,pa positions for robot
 * targetX:    x-coordinate destination
 */
float error_tx(playerc_position2d_t *position2d, float targetX)
{
  if(targetX < 0.0)
    return position2d->px - targetX;

  return targetX - position2d->px; 
}

/* error_ta()
 * position2d:  current px,py,pa positions for robot
 * targetAngle: angle to turn to
 */
float error_ta(playerc_position2d_t *position2d, float targetAngle)
{
  return targetAngle - position2d->pa;
}

/* PID()
 * pid_error: current error for destination
 */
float PID(float pid_error)
{
  float integral = integral + pid_error;
  float derivative = (pid_error - prev_error);
  pid = PID_KP*pid_error + PID_KI*integral + PID_KD*derivative;
  prev_error = pid_error;
  return pid;
}

/* PID_A()
 * pid_error_a: the current angle's error
 */
float PID_A(float pid_error_a)
{
  float integral = integral + pid_error_a;
  float derivative = (pid_error_a - prev_error_a);
  pid = PIDA_KP*pid_error_a + PIDA_KI*integral + PIDA_KD*derivative;
  prev_error_a = pid_error_a;
  return pid;
}

/* Move() 
 * client:  client to connect to robot
 * distance: x-coordinate that robot should aim for;
 * angle: angle that robot should stay at;
 */
float Move(playerc_client_t *client, float distance, float angle)
{
	printf("Enter Move\n");

	error_x = error_tx(position2d, distance);   
  
	while(error_x > 0.1)
	{
		error_x = error_tx(position2d, distance);
		vx = PID(error_x);

		error_a = error_ta(position2d, angle);
		va = PID_A(error_a);

		playerc_position2d_set_cmd_vel(position2d, vx, 0, va, 1.0);
	   
		if(bumper->bumpers[0]!=0 || bumper->bumpers[1]!=0)
		{
			playerc_position2d_set_cmd_vel(position2d, 0.0, 0.0, 0.0, 0.0);
			break;
		}
		playerc_client_read(client);
		printf("Moving : x = %f y = %f a = %f\n", position2d->px, position2d->py, position2d->pa);   
	}
	printf("Leave Move\n");
}

/* Turn()
 * client: client to connect to robot
 * deg:    amount of degrees robot should turn
 */
float Turn(playerc_client_t *client, float deg)
{
  printf("Enter Turn\n");
  float error_a = error_ta(position2d, deg);
  printf("error_a = %f\n", error_a); 
  while(fabs(error_a) > 0.1)
  {
    // Find margin of error between current and target angles
    error_a = error_ta(position2d, deg);
   
    // Set angle velocity based on error
    va = PID_A(error_a);
    printf("error_a = %f angle = %f\n", error_a, va);
    playerc_position2d_set_cmd_vel(position2d, 0.0, 0.0, va, 1.0);
    
    // Test collision with each bumper
    if(bumper->bumpers[0]!=0 || bumper->bumpers[1]!=0)
    {
      playerc_position2d_set_cmd_vel(position2d, 0.0, 0.0, 0.0, 0.0);
      break;
    }
    playerc_client_read(client);
    printf("Turning : x = %f y = %f a = %f\n", position2d->px, position2d->py, position2d->pa);   
  }
  printf("Leave Turn\n");
}

/* Main()*/
int main(int argc, const char **argv)
{
  // Create a client object and connect to the server; the server must
  // be running on "localhost" at port 6665
  client = playerc_client_create(NULL, "gort", 9876);
  if (playerc_client_connect(client) != 0)
  {
    fprintf(stderr, "error: %s\n", playerc_error_str());
    return -1;
  }

  // Create a bumper proxy (device id "bumper:0" and subscribe
   // in read mode
  bumper = playerc_bumper_create(client, 0);
  if(playerc_bumper_subscribe(bumper,PLAYERC_OPEN_MODE)!= 0)
  {
    fprintf(stderr, "error: %s\n", playerc_error_str());
    return -1;
  }

  // Create a position2d proxy (device id "position2d:0") and susbscribe
  // in read/write mode
  position2d = playerc_position2d_create(client, 0);
  if (playerc_position2d_subscribe(position2d, PLAYERC_OPEN_MODE) != 0)
  {
    fprintf(stderr, "error: %s\n", playerc_error_str());
    return -1;
  }
  // Enable the robots motors
  playerc_position2d_enable(position2d, 1);
  playerc_client_read(client);
  
  // Point 1 to Point 2
  Move(client, MOVE1, ANGLE1);
  Turn(client, TURN1);

  // Point 2 to Point 3
  Move(client, MOVE2, ANGLE2);
  Turn(client, TURN2);

  // Point 3 to Point 4
  Move(client, MOVE3, ANGLE3);
  Turn(client, TURN3);

  // Point 4 to Point 5
  Move(client, MOVE4, ANGLE4);

  // Shutdown and tidy up
  playerc_position2d_unsubscribe(position2d);
  playerc_position2d_destroy(position2d);
  playerc_client_disconnect(client);
  playerc_client_destroy(client);

  return 0;
}
