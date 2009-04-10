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
float integral     = 0.0;
float integral_a   = 0.0;
float derivative   = 0.0;
float derivative_a = 0.0;


/* PID()
 * pid_error: current error for destination
 */
float PID(float pid_error)
{
  integral = integral + pid_error;
  derivative = (pid_error - prev_error);
  pid = PID_KP*pid_error + PID_KI*integral + PID_KD*derivative;
  prev_error = pid_error;
  return pid;
}

/* PID_A()
 * pid_error_a: the current angle's error
 */
float PID_A(float pid_error_a)
{
  integral_a = integral_a + pid_error_a;
  derivative_a = (pid_error_a - prev_error_a);
  pid = PIDA_KP*pid_error_a + PIDA_KI*integral_a + PIDA_KD*derivative_a;
  prev_error_a = pid_error_a;
  return pid;
}


