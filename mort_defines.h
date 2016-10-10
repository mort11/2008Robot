/************************************************************************************
* FILE NAME: MORT_DEFINES.h
*
* DESCRIPTION:
*  This is the include file that defines all I/O used in the MORT 2005 Robot
*  It contains aliases and function prototypes used in all MORT 'C' routines.
*
************************************************************************************/

#ifndef _MORT_DEFINES_H
#define _MORT_DEFINES_H_

/************************************************************************************
			                        MORT DEFINES
*************************************************************************************/

/**********************************CODE SWITCH DEFINES*******************************/
#define DEBUG_ROBOT

#define TOWER_LIMIT_ENABLED  1

#define ON      1
#define OFF     0

/*******************************USER CONTROL DEFINES*********************************/

//#define DRIVE_X2      p1_x
#define DRIVE_X         p2_x
//#define DRIVE_Z2      p1_wheel

#define DRIVE_Y         p1_y
#define DRIVE_Z         p1_x


#define DRIVE_TRIG      p1_sw_trig

#define TOWER_JOYSTICK          p3_y
#define TOWER_BUTTON_ROLLERS    (!p3_sw_top)
#define TOWER_BUTTON_LOW        p4_sw_aux1
#define TOWER_BUTTON_MIDDLE     p4_sw_top
#define TOWER_BUTTON_HIGH       p4_sw_trig

/************************************PWM DEFINES*************************************/
#define FRONT_LEFT      pwm03
#define FRONT_RIGHT     pwm04
#define BACK_LEFT       pwm02
#define BACK_RIGHT      pwm01

#define ROLLER_MOTOR    pwm06
#define TOWER_MOTOR_1   pwm07
#define TOWER_MOTOR_2   pwm05

/**********************************CONSTANT DEFINES**********************************/

#define HEIGHT_FLAG_HIGH        3
#define HEIGHT_FLAG_MIDDLE      2
#define HEIGHT_FLAG_LOW         1
#define HEIGHT_FLAG_MANUAL      0

#define HEIGHT_COUNT_HIGH       570
#define HEIGHT_COUNT_MIDDLE     122 //122

#define TOWER_ENCODER_LIMIT             575 //555
#define TOWER_ENCODER_DEADZONE          3
#define TOWER_HOLD_POSITION_DEADZONE    1
#define TOWER_ENCODER_GAIN              1
#define TOWER_HOLD_POSITION_GAIN        4

#define TOWER_DOWN_SPEED            40

// Scaling constants
#define DRIVE_X_MAX             130         // Joystick left
#define DRIVE_X_OFFSET          92
#define DRIVE_X_MIN             (-1 * 69)   // Joystick right

#define DRIVE_Y_MAX             117         // Joystick up
#define DRIVE_Y_OFFSET          88
#define DRIVE_Y_MIN             (-1 * 67)   // Joystick down

#define DRIVE_Z_MAX             107         // Joystick twist left
#define DRIVE_Z_OFFSET          68
#define DRIVE_Z_MIN             (-1 * 68)   // Joystick twist right

#define TOWER_JOYSTICK_MAX      57
#define TOWER_JOYSTICK_OFFSET   49
#define TOWER_JOYSTICK_MIN      (-1 * 47)      // Pushed forward

#define DRIVE_X_DEADBAND        15
#define DRIVE_Y_DEADBAND        15
#define DRIVE_Z_DEADBAND        25
#define TOWER_JOYSTICK_DEADBAND 15

#define PWM_DEADZONE_LEFT       12.0
#define PWM_DEADZONE_RIGHT      12.0

#define GYRO_CORRECTION_DIVISOR 4
//dashboard
#define NUM_PACKETS             3

/**********************************SENSOR DEFINES*************************************/
#define BRAKE_1         rc_dig_out14
#define BRAKE_2         rc_dig_out15
#define BRAKE_3         rc_dig_out16
#define BRAKE_4         rc_dig_out17

#define TOWER_BALL_SWITCH   (!rc_dig_in03)

#define TOWER_LIMIT_DOWN    (!rc_dig_in02)

#define IR_LEFT_FRONT   1 //channels passed to ADC code
#define IR_LEFT_BACK    2
#define IR_FRONT_LEFT   4
#define IR_FRONT_RIGHT  3

#define PROGRAM_BUTTON  rc_dig_in18

#define ROBOCOACH_1     rc_dig_in08
#define ROBOCOACH_2     rc_dig_in05
#define ROBOCOACH_3     rc_dig_in04
#define ROBOCOACH_4     rc_dig_in07

/***********************************OI FEEDBACK***************************************/
#define HEIGHT_LED_HIGH     Relay2_green
#define HEIGHT_LED_MIDDLE   Relay1_green
#define HEIGHT_LED_LOW      Relay1_red

/************************************ EXTERNS ****************************************/
extern unsigned char height_flag;
extern unsigned char tower_at_target;
extern unsigned char auto_state;

/*******************************FUNCTION PROTOTYPES***********************************/

void Mec_Drive(int, int, int);
int abs(int);
int absmax(int,int);

int Limit (int, int, int);
int Deadband(int, int);
void Service_Joysticks(void);
void Service_Tower(void);
void Service_Height_Flags(void);
void Service_Brakes(unsigned char);
void Service_Leds(void);
void Service_Program_Button(void);
void Service_Dashboard_Data(void);

void Drive_Straight(void);

void Service_Tower2(void);

#endif
