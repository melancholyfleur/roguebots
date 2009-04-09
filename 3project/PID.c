/* *********************
 *  Project 3
 *  Marylou Kunkle
 *  Lenny Kramer
 *  PID.c
 ********************* */

#include "PID.h"

float pid          = 0.0;             // pid controller value
float prev_error   = 0.0;             // previous error for x/y destination
float prev_error_a = 0.0;             // previous error for angle

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


