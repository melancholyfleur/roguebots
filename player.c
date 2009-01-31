/* **************
   Project 1
   Mary-Lou Kunkle
   Lenny Kramer
   1/28/09
**************** */

#include <stdio.h>
#include <math.h>
#include <libplayerc/playerc.h>

#define R (.5)
#define ANGLE ((2)*(M_PI))
#define CIRC ((ANGLE)*(R))

/* GLOBAL VARS */
playerc_client_t *client;         // client used to connect to robot
playerc_position2d_t *position2d; // store the px,py,pa values for robot
playerc_bumper_t *bumper;         // bumper client for robot
float prev_error   = 0.0;         // previous error for x/y destination
float prev_error_a = 0.0;         // previous error for angle
float error_x      = 0.0;         // current x error
float error_y      = 0.0;         // current y error
float error_a      = 0.0;         // current angle error
float pid          = 0.0;         // pid controller value
float vx           = 0.0;         // x velocity
float va           = 0.0;         // angle velocity

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

/* error_ty()
 * position2d: current px,py,pa positions for robot
 * targetY:    y-coordinate destination
 */
float error_ty(playerc_position2d_t *position2d, float targetY) 
{
  return targetY - position2d->py;
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
  float dt = 1.0;
  float Kp = 1.0;
  float Ki = 0.05;
  float Kd = 0.0;
  float integral = integral + pid_error * dt;
  float derivative = (pid_error - prev_error)/dt;
  pid = Kp*pid_error + Ki*integral + Kd*derivative;
  prev_error = pid_error;
  return pid;
}

/* PID_A()
 * pid_error_a: the current angle's error
 */
float PID_A(float pid_error_a)
{
  float dt = 1.0;
  float Kp = 0.5;
  float Ki = 0.0001;
  float Kd = 0.0;
  float iArray[5];
  float integral = integral + pid_error_a * dt;
  float derivative = (pid_error_a - prev_error_a)/dt;
  pid = Kp*pid_error_a + Ki*integral + Kd*derivative;
  prev_error_a = pid_error_a;
  return pid;
}

/* Move() 
 * client:  client to connect to robot
 * targetX: x-coordinate that robot should aim for;
 * targetY: y-coordinate that robot should aim for;
 * angle:   the angle that the robot should stay on course at;
 */
float Move(playerc_client_t *client, float targetX, float targetY,  float angle, int point)
{
  printf("Enter Move\n");
 
    if(point == 1 || point == 3)
    {
      error_x = error_tx(position2d, targetX);
    }

    else if(point == 2 || point == 4)
    {
      error_y = error_ty(position2d, targetY);    
    }
  
  while(error_x > 0.1 || error_y > 0.1)
  {
    if(point == 1 || point == 3)
    {
      error_x = error_tx(position2d, targetX);
      vx = PID(error_x);
    }

    else if(point == 2 || point == 4)
    {
      error_y = error_ty(position2d, targetY);
      vx = PID(error_y);
    }

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
    // Find maring of error between current and target angles
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
  int i;
  float curr_px;
  float curr_py;
  float curr_pa;

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
  Move(client, 3.3, 0.0, 0.0, 1);
  Turn(client, (M_PI/2));

  // Point 2 to Point 3
  curr_px = position2d->px;
  curr_pa = position2d->pa;
  Move(client, curr_px, 3.04, curr_pa, 2);
  Turn(client, curr_pa + (M_PI/2.5));

  // Point 3 to Point 4
  curr_px = position2d->px;
  curr_pa = position2d->pa;
  Move(client, curr_px - 3.8, 0.0, curr_pa,3);
  Turn(client, (M_PI/2.2));

  // Point 4 to Point 5
  curr_px = position2d->px;
  curr_py = position2d->py;
  curr_pa = position2d->pa;
  Move(client, 0.0, curr_py+6.81, curr_pa, 4);

  // Shutdown and tidy up
  playerc_position2d_unsubscribe(position2d);
  playerc_position2d_destroy(position2d);
  playerc_client_disconnect(client);
  playerc_client_destroy(client);

  return 0;
}
