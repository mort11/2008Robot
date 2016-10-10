/*******************************************************************************
* FILE NAME: autonomous.h
*
* DESCRIPTION:
*  This is the include file that defines all I/O used in the MORT 2008 Robot
*  It contains aliases and function prototypes used in all MORT 'C' routines.
*
*******************************************************************************/

#ifndef _autonomous_h
#define _autonomous_h

#define AUTOMODE_IR     1
#define AUTOMODE_NO_D   2
#define AUTOMODE_D      3
#define AUTOMODE_EPIC_FAIL   4

#define CIRCLE_FIELD_FORWARD_SPEED          40
#define CIRCLE_FIELD_DISTANCE_HUG           1000   // increases as it gets closer
#define CIRCLE_FIELD_DISTANCE_NOWALL        50
#define CIRCLE_FIELD_Z_GAIN                 7.0
#define CIRCLE_FIELD_Z_LIMIT                20
#define CIRCLE_FIELD_TURN_LEFT_SPEED        (-1*40)

#define DRIVE_FORWARD   0
#define TURN            1
#define TURN_FORWARD    2
#define EXECUTE         3

#define IR_ANGLE    5
#define ANGLE_GAIN -170/90 // (measured_angle / actual_angle) used to multiply with IR_ANGLE to convert angle to count

void Autonomous_Init(void);
void Autonomous(void);
void Autonomous_Spin(void);
void Drive_Straight(void);
void Drive_Straight_Delay(void);
void Drive_Straight_Gyro(void);
void Drive_Continuous(void);
void Drive_Continuous_IR(void);
int Get_Auto_Mode(void);

void IR_Drive(void);
void IR_Drive2(void);

#endif
