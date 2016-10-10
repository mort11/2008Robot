#ifndef _outputs_h
#define _outputs_h

#include "mort_defines.h"

/** PWM Defines **/
#define FRONT_LEFT      pwm01
#define FRONT_RIGHT     pwm02
#define BACK_LEFT       pwm03
#define BACK_RIGHT      pwm04

#define ARM_MOTOR       pwm05
#define ROLLER_MOTOR    pwm06

// Used to drive the 2005 robot
#define RIGHT_MOTOR_1   pwm01
#define RIGHT_MOTOR_2   pwm02
#define LEFT_MOTOR_1    pwm03
#define LEFT_MOTOR_2    pwm04

/** Function Defines **/

// Based on the code from ChiefDelphi, rounds off imidiatly, use comparison of mec(1,127,1) to mec(0,127,0) to see
void Mec_Drive_1(int, int, int);

// Writen based upon the white paper on CheifDephi, no rounding until the final assignment
void Mec_Drive_2(int, int, int);

// Both of these are utilized by Mec_Drive_2()
int abs(int);
int absmax(int,int);

int Limit (int, int, int);
int Deadband(int, int);
void Service_Arm(int);
void Service_Rollers(int);

void Non_Mec_Drive(int, int);

#endif
